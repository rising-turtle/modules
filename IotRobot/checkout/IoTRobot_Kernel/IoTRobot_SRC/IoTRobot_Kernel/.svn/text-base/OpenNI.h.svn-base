#ifndef IOT_OPENNI_H
#define IOT_OPENNI_H

#pragma once
#include "InternalDefine.h"
//#include "IoTRobot_NetServer.h"
#include "IoTRobot_NetServer_Interface.h"
#include "IoTRobot_NetServer_Define.h"

#ifdef _DEBUG
#pragma comment(lib,"jrtplib_d.lib")
#pragma comment(lib,"jthread_d.lib")
//#pragma comment(lib,"RTP/jrtplib.lib")
//#pragma comment(lib,"RTP/jthread.lib")
#else
#pragma comment(lib,"jrtplib.lib")
#pragma comment(lib,"jthread.lib")
#endif

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

	static  IoTRobot_NetServer_Interface m_cNetServer;
	void OpenNIInit(IoT_CallBackFuncSet stCallBackSet);
	void OpenNIRun();
	void OpenNIUnint();


	int GetDeviceState(int *pnState);

	int GetOneFrame();


	static UINT ThreadNetServer(LPVOID lpParam);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr IoT_ConvertToXYZRGBPointCloud(unsigned char*pucRGB,unsigned short *pusDepth) const;
};

#endif