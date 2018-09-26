#include "KinectToJSON.h"

// file scope variables
IKinectSensor *sensor = nullptr;
CONFIG config;
float cameraHeight;

/**
* The callback used by main.
*/
void toConsole(char * msg) {
	std::cout << msg;
}

/**
 * Throw away console printer for testing.
 */
int main(int argc, char *argv[]) {
	char buffer[10];

	HRESULT hr = openSensor('\1', 'F');

	if (SUCCEEDED(hr)) {
		hr = beginBodyTracking(&toConsole);
	}

	if (SUCCEEDED(hr)) {
		printf("cntrl-c when done: ");
		scanf_s("%9s", buffer, (unsigned)_countof(buffer) );
		closeSensor();
	}
	return 0;
}

/**
 * Open the sensor using these simple char settings for easy mapping to python ctypes.
 * @param {char / ctypes.c_char} actionPoseStart - anything other than 0, is true.  Too many issues passing a bool from Python
 * @param {char / ctypes.c_char} Forward_or_Mirror - F for matching with the actual side, or M for as looks in a mirror
 */
DllExport HRESULT openSensor(char actionPoseStart, char Forward_or_Mirror) {
	cameraHeight = -1;

	//Get the default Kinect sensor
	HRESULT hr = GetDefaultKinectSensor(&sensor);

	//If the function succeeds, open the sensor
	if (SUCCEEDED(hr)) {
		hr = sensor->Open();
	}

	if (sensor == nullptr || FAILED(hr)) {
		std::cerr << "Cannot find any sensors.\n";
	
	} else {
		// ready to go; transfer args to a CONFIG struct
		config.mirror = Forward_or_Mirror != 'F';
		config.TPoseStart = actionPoseStart;

		std::cout << "Sensor opened with these settings- Mirror: '";
		std::cout << (config.mirror ? "True" : "False");
		std::cout << "', T Pose Start: '";
		std::cout << (config.TPoseStart ? "True" : "False");
		std::cout << "'\n";
	}
	return hr;
}

void setCameraHeight(float height) {
	cameraHeight = height;
}

float getCameraHeight() {
	return cameraHeight;
}

/**
 * Close the sensor & stop any tracking that may be occurring.
 * @returns the camera height, in millis so can be an int
 */
DllExport HRESULT closeSensor() {
	if (sensor != nullptr) {
		// it is the responsibility of a tracking file to determine if it was actually started.
		endBodyTracking();

		// close sensor last to avoid exceptions in any of the trackers
		sensor->Close();
	}
	sensor = nullptr;
	return (HRESULT) (cameraHeight * 1000);
}