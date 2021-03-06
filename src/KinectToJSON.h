#pragma once

// Including SDKDDKVer.h defines the highest available Windows platform.

// If you wish to build your application for a previous Windows platform, include WinSDKVer.h and
// set the _WIN32_WINNT macro to the platform you wish to support before including SDKDDKVer.h.
#include <SDKDDKVer.h>

#include <stdio.h>
#include <iostream>
#include <Kinect.h>

#include <thread>
#include <mutex>

#define DllExport   __declspec( dllexport )

typedef struct {
	bool mirror;
	bool TPoseStart;
	bool worldSpace;
} CONFIG;

// entry points of KinectToJSON.cpp
DllExport HRESULT openSensor(char actionPoseStart, char Forward_or_Mirror, char Camera_or_WorldSpace);
DllExport HRESULT closeSensor();
void setCameraHeight(float height);
float getCameraHeight();

// entry points BodyTracking.cpp
DllExport HRESULT beginBodyTracking( void(*cb)(char *) );
DllExport void endBodyTracking();

// Safe release for interfaces
template<class Interface>
inline void SafeRelease(Interface *& pInterfaceToRelease)
{
	if (pInterfaceToRelease != NULL)
	{
		pInterfaceToRelease->Release();
		pInterfaceToRelease = NULL;
	}
}