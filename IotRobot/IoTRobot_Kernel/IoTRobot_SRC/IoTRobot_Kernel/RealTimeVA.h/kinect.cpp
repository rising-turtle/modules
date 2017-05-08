#include "stdafx.h"
#include "kinect.h"

kinect::kinect()
{

}

kinect::~kinect()
{
	context.StopGeneratingAll();
	context.Shutdown();
}

void kinect::CheckOpenNIError( XnStatus result, string status )
{ 
	if( result != XN_STATUS_OK ) 
		cerr<<status<<" Error: "<< xnGetStatusString( result ) << endl;
}

bool kinect::Initialization(unsigned int width, unsigned int height, unsigned int frameRate)
{
	XnStatus result = XN_STATUS_OK;  

	result = context.Init(); 
	CheckOpenNIError( result, "initialize context" );  
	cout<<"context.Init() result: "<<result<<endl;
	if (result != XN_STATUS_OK)
	{
		goto exitEntry;
	}

	result = imageGenerator.Create( context ); 
	CheckOpenNIError( result, "Create image generator" );
	cout<<"imageGenerator.Create( context )"<<endl;
	if (result != XN_STATUS_OK)
	{
		goto exitEntry;
	}

	mapMode.nXRes = width;  
	mapMode.nYRes = height; 
	mapMode.nFPS = frameRate; 
	result = imageGenerator.SetMapOutputMode( mapMode ); 
	cout<<"imageGenerator.SetMapOutputMode( mapMode )"<<endl;
	if (result != XN_STATUS_OK)
	{
		goto exitEntry;
	}

	result = context.StartGeneratingAll();
	cout<<"context.StartGeneratingAll()"<<endl;
	if (result != XN_STATUS_OK)
	{
		goto exitEntry;
	}

	result = context.WaitNoneUpdateAll(); 
	cout<<"context.WaitNoneUpdateAll()"<<endl;
	if (result != XN_STATUS_OK)
	{
		goto exitEntry;
	}

exitEntry:
	return result == XN_STATUS_OK? true : false;
}

void kinect::GenerateImage()
{
	//get meta data
	imageGenerator.GetMetaData(imageMD);
}

bool kinect::WaitNoneUpdateAll()
{
	return context.WaitNoneUpdateAll() == XN_STATUS_OK? true : false;
}