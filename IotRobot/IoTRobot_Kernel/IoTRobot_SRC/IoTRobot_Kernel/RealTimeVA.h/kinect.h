#include <string>
#include <iostream>
using namespace std;

//openNI
#include <XnCppWrapper.h>

#pragma comment(lib, "openNI.lib")

class kinect
{
public:	//methods

	kinect();
	~kinect();
	//*************************************
	// for openNI error handling
	//*************************************
	void CheckOpenNIError( XnStatus result, string status );

	bool Initialization(unsigned int width = 640,
						unsigned int height = 480,
						unsigned int frameRate = 30);

	void GenerateImage();

	bool WaitNoneUpdateAll();


public:	//variables
	//--------------------- openNI ---------------------
	xn::ImageMetaData imageMD;

private:
	xn::Context context; 
	xn::ImageGenerator imageGenerator;
	XnMapOutputMode mapMode;
};