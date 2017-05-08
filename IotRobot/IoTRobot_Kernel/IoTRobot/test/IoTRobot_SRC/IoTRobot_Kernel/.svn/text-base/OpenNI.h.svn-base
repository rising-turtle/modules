#ifndef IOT_OPENNI_H
#define IOT_OPENNI_H

#pragma once
#include "InternalDefine.h"
#include "IoTRobot_NetServer.h"

class COpenNI
{
public:
	COpenNI();
	~COpenNI();


public:

	void COpenNITest();

public:

	static IoT_CallBackFuncSet m_stCallBackFuncSet;
	pcl::Grabber* m_pcPCLGrabber;

	void Cloud_cb (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud);
	void Image_cb (const boost::shared_ptr<openni_wrapper::Image>& img);

	static void RGB_Depth_cb(unsigned char*pucRGB,unsigned short *pusDepth,void *pContext);

	static IoTRobot_NetServer m_cNetServer;
	void OpenNIInit(IoT_CallBackFuncSet stCallBackSet);
	void OpenNIRun();
	void OpenNIUnint();

	int GetOneFrame();


	static UINT ThreadNetServer(LPVOID lpParam);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr IoT_ConvertToXYZRGBPointCloud(unsigned char*pucRGB,unsigned short *pusDepth) const;
};

#endif