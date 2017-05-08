#ifndef IOTROBOT_USBCAMERA_H
#define IOTROBOT_USBCAMERA_H


//#define CAMERA_OPENCV
#define CAMERA_DIRECTSHOW

#define CAMERA_WIDTH 320
#define CAMERA_HEIGHT 240
#pragma once
#include "opencv/cv.h"
#include "opencv/highgui.h"


#include "CameraDS.h"

using namespace cv;

typedef void (*USBCameraCallBack_ImgData)(unsigned char*pucRTPData,void *pContext);


class IoTRobot_USBCamera
{
public:
	IoTRobot_USBCamera();
	~IoTRobot_USBCamera();


	int USBCameraInit(int nIdx,USBCameraCallBack_ImgData cbImgData,void *pContext);
	int USBCameraRun();
	int USBCameraStop();
	int USBCameraUninit();
	//Mat frame;						//for camera
	//Mat  m_rgb8u(480,640,CV_8UC3);	//for Kinect
	
protected:
private:
#ifdef CAMERA_OPENCV

	VideoCapture m_cvCamCapture;			//for camera
#endif

#ifdef CAMERA_DIRECTSHOW
	unsigned char *m_pucQVGABuff;
	CCameraDS* m_pCCameraDS;
	Mat* m_Frame;
#endif

	USBCameraCallBack_ImgData m_cbImgData;
	void *m_pContext;
	static UINT ThreadGetUSBCameraData(LPVOID lpParam);
	HANDLE m_hThreadGetUSBCameraData;
				//foe directShow camera
	bool m_bStopGetUSBCameraData;
	Mat m_cvOneFrame;
};

#endif