#include "KinectToJSON.h"

// forward declare of non-external functions
void bodyReaderThreadLoop();
void processBodies(const unsigned int &bodyCount, IBody **bodies, const Vector4 clipPlane);
bool detectTPose(Joint *joints);
int formatVector(const CameraSpacePoint Position, const int idx);
int formatQuaternion(const Vector4 Orientation, const int idx);

// file scope variables
IBodyFrameReader *bodyFrameReader = nullptr;
void (*applicationCallback)(char *) = nullptr;
CONFIG localConfig; // copy of main's; assigned when locked for thread safe transfer
bool rolling = false;
int nBodies = 0; // used to beep when changed
int frame = 0;
bool clipPlaneEval = false; // used to print the height of the first non-zero reading of camera height, floor clip pane W

// threading, and mutex for sharing applicationCallback & config across threads
std::thread bodyThread;
std::mutex mu;

// storage for json
#define BUF_SZ 10000
char json[BUF_SZ];

// vars from main.cpp
extern IKinectSensor *sensor;
extern CONFIG config;

// constants, in the order reported by body frame reader
const char SPINE_BASE[] = "SpineBase";         //  0
const char SPINE_MID[] = "SpineMid";           //  1
const char NECK[] = "Neck";                    //  2 
const char HEAD[] = "Head";                    //  3

const char SHOULDER_LEFT[] = "ShoulderLeft";   //  4
const char ELBOW_LEFT[] = "ElbowLeft";         //  5
const char WRIST_LEFT[] = "WristLeft";         //  6
const char HAND_LEFT[] = "HandLeft";           //  7

const char SHOULDER_RIGHT[] = "ShoulderRight"; //  8
const char ELBOW_RIGHT[] = "ElbowRight";       //  9
const char WRIST_RIGHT[] = "WristRight";       // 10
const char HAND_RIGHT[] = "HandRight";         // 11

const char HIP_LEFT[] = "HipLeft";             // 12
const char KNEE_LEFT[] = "KneeLeft";           // 13
const char ANKLE_LEFT[] = "AnkleLeft";         // 14
const char FOOT_LEFT[] = "FootLeft";           // 15

const char HIP_RIGHT[] = "HipRight";           // 16
const char KNEE_RIGHT[] = "KneeRight";         // 17
const char ANKLE_RIGHT[] = "AnkleRight";       // 18
const char FOOT_RIGHT[] = "FootRight";         // 19

const char SPINE_SHOULDER[] = "SpineShoulder"; // 20

const char HAND_TIP_LEFT[] = "HandTipLeft";    // 21
const char THUMB_LEFT[] = "ThumbLeft";         // 22

const char HAND_TIP_RIGHT[] = "HandTipRight";  // 23
const char THUMB_RIGHT[] = "ThumbRight";       // 24

const char *JOINT_NAMES[] = { 
	SPINE_BASE, SPINE_MID, NECK, HEAD,
	SHOULDER_LEFT, ELBOW_LEFT, WRIST_LEFT, HAND_LEFT,
	SHOULDER_RIGHT, ELBOW_RIGHT, WRIST_RIGHT, HAND_RIGHT,
	HIP_LEFT, KNEE_LEFT, ANKLE_LEFT, FOOT_LEFT,
	HIP_RIGHT, KNEE_RIGHT, ANKLE_RIGHT, FOOT_RIGHT,
	SPINE_SHOULDER,
	HAND_TIP_LEFT, THUMB_LEFT,
	HAND_TIP_RIGHT, THUMB_RIGHT
};

const char NOT_TRACKED[] = "Not Tracked"; // used for both join & hand states
const char INFERRED[] = "Inferred";
const char TRACKED[] = "Tracked";
const char *TRACK_STATES[] = { NOT_TRACKED, INFERRED, TRACKED };

const char UNKNOWN[] = "Unknown";  // 0
const char OPEN[] = "Open";        // 2
const char CLOSED[] = "Closed";    // 3
const char LASSO[] = "Lasso";      // 4
const char *HAND_STATES[] = { UNKNOWN, NOT_TRACKED, OPEN, CLOSED, LASSO };

/**
 * Begin tracking up to 6 bodies.
 * @param cb - The callback function which is passed the JSON as an argument.
 */
DllExport HRESULT beginBodyTracking( void (*cb)(char *) ) {
	// openSensor must have sucessfully been called first
	if (sensor == nullptr) {
		std::cerr << "Sensor Not open.\n";
		return E_ABORT;
    }

	//Get a body frame source from which we can get our body frame reader
	IBodyFrameSource *bodyFrameSource = nullptr;
	HRESULT hr = sensor->get_BodyFrameSource(&bodyFrameSource);

	if (SUCCEEDED(hr)) {
		hr = bodyFrameSource->OpenReader(&bodyFrameReader);
	}

	if (SUCCEEDED(hr)) {
		// assign the arg to a file scope version
		mu.lock();
		applicationCallback = cb;

		// make sure config can be assigned in the threadh which called openSensor(), and be visible in the body reader thread
		localConfig.mirror = config.mirror;
		localConfig.TPoseStart = config.TPoseStart;
		rolling = false;
		frame = 0;
		clipPlaneEval = false;
		mu.unlock();

		// start the body thread
		bodyThread = std::thread (&bodyReaderThreadLoop);
	}
	
	//We're done with bodyFrameSource, so we'll release it
	SafeRelease(bodyFrameSource);

	return hr;
}

/**
 * stop body, if it had been started.
 */
DllExport void endBodyTracking(){
	mu.lock();
	bool running = applicationCallback != nullptr;
	applicationCallback = nullptr; // causes thread loop to fall out
	mu.unlock();

	if (running) {
		// block till thread is done, then release reader
		bodyThread.join();
		bodyFrameReader->Release();
	}
	bodyFrameReader = nullptr;
}

void bodyReaderThreadLoop() {
	bool timeToQuit;

	// loop forever till told to stop
	while (1) {
		mu.lock();
		timeToQuit = applicationCallback == nullptr;
		mu.unlock();

		if (timeToQuit) break;

		IBodyFrame *bodyFrame = nullptr;

		// this only produces a good hr 30 times a second, based on bodyFrame->get_RelativeTime()
		HRESULT hr = bodyFrameReader->AcquireLatestFrame(&bodyFrame);

		if (SUCCEEDED(hr)) {
			Vector4 clipPlane;
			hr = bodyFrame->get_FloorClipPlane(&clipPlane);

			if (SUCCEEDED(hr)) {
				IBody *bodies[BODY_COUNT] = { 0 };
				hr = bodyFrame->GetAndRefreshBodyData(_countof(bodies), bodies);

				if (SUCCEEDED(hr)) {
					processBodies(BODY_COUNT, bodies, clipPlane);
					//After body processing is done, we're done with our bodies so release them.
					for (unsigned int bodyIndex = 0; bodyIndex < _countof(bodies); bodyIndex++) {
						SafeRelease(bodies[bodyIndex]);
					}
				}
			}
			SafeRelease(bodyFrame);
		}
	}
}

/**
 * Actual function which generates the JSON from the body reader.  If no bodies are found, it does not write any JSON.
 */
void processBodies(const unsigned int &bodyCount, IBody **bodies, const Vector4 clipPlane) {
	// increment the frame #, once rolling, even if the does not get written
	if (rolling) frame++;

	// get camera angle from clip plane to
	float cameraAngleRadians = atan(clipPlane.z / clipPlane.y);
	float cosCameraAngle = cos(cameraAngleRadians);
	float sinCameraAngle = sin(cameraAngleRadians);

	//	Matrix clipPlaneTransform = clipPlaneTransformMat(clipPlane);
	if (!clipPlaneEval && clipPlane.w != 0) {
		setCameraHeight(clipPlane.w);
		clipPlaneEval = true;
	}

	int idx = sprintf_s(json, BUF_SZ, "{\n\"floorClipPlane\": ");
	idx += formatQuaternion(clipPlane, idx);
	idx += sprintf_s(&json[idx], BUF_SZ - idx, ", \"cameraHeight\": %.3f", getCameraHeight());
	idx += sprintf_s(&json[idx], BUF_SZ - idx, ",\n\"frame\": %d", frame);

	int bodiesFound = 0;
	int completeBodiesFound = 0;
	idx += sprintf_s(&json[idx], BUF_SZ - idx, ",\n\"bodies\": [");
	for (unsigned int bodyIndex = 0; bodyIndex < bodyCount; bodyIndex++) {
		IBody *body = bodies[bodyIndex];

		//Get the tracking status for the body, if it's not tracked we'll skip it
		BOOLEAN isTracked = false;
		HRESULT hr = body->get_IsTracked(&isTracked);
		if (FAILED(hr) || isTracked == false) {
			continue;
		}
		
		UINT64 id;
		hr = body->get_TrackingId(&id);
		if (FAILED(hr)) {
			continue;
		}

		Joint joints[JointType_Count];
		hr = body->GetJoints(_countof(joints), joints);
		if (FAILED(hr)) {
			continue;
		}

		JointOrientation rotations[JointType_Count];
		hr = body->GetJointOrientations(_countof(rotations), rotations);
		if (FAILED(hr)) {
			continue;
		}

		HandState leftHandState = HandState_Unknown;
		HandState rightHandState = HandState_Unknown;

		body->get_HandLeftState(&leftHandState);
		body->get_HandRightState(&rightHandState);

		if (!rolling && !detectTPose(joints)) {
			continue;
		}

        // verify no joints untracked before writing anything
		bool unTrackedFound = false;
		for (unsigned int i = 0; i < JointType_Count; i++) {
			if ((!joints[i].TrackingState) || (i == 0 && joints[i].TrackingState < 2)) {
				unTrackedFound = true;
				break;
			}
		}
		if (!unTrackedFound) {
			completeBodiesFound++;
		}

		// all set to write out body
		if (bodiesFound++ > 1) {
			idx += sprintf_s(&json[idx], BUF_SZ - idx, ",");
		}
		idx += sprintf_s(&json[idx], BUF_SZ - idx, "\n\t{");
		idx += sprintf_s(&json[idx], BUF_SZ - idx, "\n\t\"id\": %I64d,", id);
		idx += sprintf_s(&json[idx], BUF_SZ - idx, "\n\t\"bones\": {");

		// write out the joints
		for (unsigned int i = 0; i < JointType_Count; i++) {
			CameraSpacePoint position = joints[i].Position;
			int trackStateIdx = joints[i].TrackingState;
			Vector4 orientation = rotations[i].Orientation;

			if (!localConfig.mirror) {
				position.X *= -1;
				// Not using rotation data, so not actually tested
				// When Blender does a reflection paste on a quaternion, it just flips the signs of both Y & Z
				orientation.y *= -1;
				orientation.z *= -1;
			}

			// adjust Y for the camera distance from the floor
			position.Y = clipPlane.w + position.Y * cosCameraAngle + position.Z * sinCameraAngle;

			// adjust for camera location rotation (squedo code) TO DO
//			Matrix cameraSpaceMatrix = compose(orientation, position);
//			Matrix worldSpaceMatrix = multiply(cameraSpaceMatrix, clipPlaneTransform);
//			decompose(worldSpaceMatrix, &orientation, &position);

			if (i > 0) {
				idx += sprintf_s(&json[idx], BUF_SZ - idx, ",");
			}
			idx += sprintf_s(&json[idx], BUF_SZ - idx, "\n\t\t\"%s\": {", JOINT_NAMES[i]);
			idx += sprintf_s(&json[idx], BUF_SZ - idx, "\n\t\t\t\"state\": \"%s\"", TRACK_STATES[trackStateIdx]);
			idx += sprintf_s(&json[idx], BUF_SZ - idx, ",\n\t\t\t\"location\": ");
			idx += formatVector(position, idx);
			idx += sprintf_s(&json[idx], BUF_SZ - idx, ",\n\t\t\t\"rotation\": ");
			idx += formatQuaternion(orientation, idx);
			idx += sprintf_s(&json[idx], BUF_SZ - idx, "\n\t\t}");
		}
		idx += sprintf_s(&json[idx], BUF_SZ - idx, "\n\t}");

		idx += sprintf_s(&json[idx], BUF_SZ - idx, ",\n\t\"hands\": {");
		idx += sprintf_s(&json[idx], BUF_SZ - idx, "\n\t\t\"left\": \"%s\"", HAND_STATES[leftHandState]);
		idx += sprintf_s(&json[idx], BUF_SZ - idx, ",\n\t\t\"right\": \"%s\"", HAND_STATES[rightHandState]);
		idx += sprintf_s(&json[idx], BUF_SZ - idx, "\n\t}");
	}
	idx += sprintf_s(&json[idx], BUF_SZ - idx, "\n}\n]\n}\n");

	// indicate when # of bodies changes
	if (nBodies != completeBodiesFound) {
		nBodies  = completeBodiesFound;
		MessageBeep(MB_OK);
	}

	// do not callback when no bodies actually found
	if (bodiesFound == 0) return;

	mu.lock();
	// double check not told to stop after frame processing began
	if (applicationCallback != nullptr) applicationCallback(json);
	mu.unlock();
}

bool detectTPose(Joint *joints) {
	float shoulderLeft = joints[4].Position.Y;
	float shoulderRight = joints[8].Position.Y;

	float handTipLeft = joints[21].Position.Y;
	float handTipRight = joints[23].Position.Y;
	if (!localConfig.TPoseStart) {
		rolling = true;
	}
	else {
		rolling = shoulderLeft <= handTipLeft && shoulderRight <= handTipRight;
	}

	// make a beep signalling action detected
	if (rolling) {
		MessageBeep(MB_OK);
		std::cout << printf("TPose data- shoulderLeft: %f, shoulerRight: %f, handTipLeft: %f, handTipRight: %f\n", shoulderLeft, shoulderRight, handTipLeft, handTipRight);
	}
	return rolling;
}

int formatVector(const CameraSpacePoint Position, const int idx) {
	return sprintf_s(&json[idx], BUF_SZ - idx, "{\"x\":%.3f,\"y\":%.3f,\"z\":%.3f}", Position.X, Position.Y, Position.Z);
}

int formatQuaternion(const Vector4 Orientation, const int idx) {
    return sprintf_s(&json[idx], BUF_SZ - idx, "{\"x\":%.3f,\"y\":%.3f,\"z\":%.3f,\"w\":%.3f}", Orientation.x, Orientation.y, Orientation.z, Orientation.w);
}