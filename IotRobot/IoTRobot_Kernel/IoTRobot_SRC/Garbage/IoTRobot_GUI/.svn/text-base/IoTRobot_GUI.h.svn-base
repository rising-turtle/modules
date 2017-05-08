// IoTRobot_GUI.h : main header file for the IoTRobot_GUI DLL
//

#pragma once

#ifndef __AFXWIN_H__
	#error "include 'stdafx.h' before including this file for PCH"
#endif


#include "resource.h"		// main symbols
/*#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/common/time.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/openni_camera/openni_exception.h>
#include <pcl/pcl_config.h>
#include <pcl/io/openni_camera/openni_image.h>
#include <pcl/pcl_macros.h>*/

// CIoTRobot_GUIApp
// See IoTRobot_GUI.cpp for the implementation of this class
//
//typedef void (*OpenNICallBack_XYZRGB)(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud,void *pContext);
//typedef void (*OpenNICallBack_RGB)(const boost::shared_ptr<openni_wrapper::Image>& img,void *pContext);
//typedef void (*OpenNICallBack_RGB24)(unsigned char *pucRGB24,void *pContext);

class CIoTRobot_GUIApp : public CWinApp
{
public:
	CIoTRobot_GUIApp();

	int AppRun();
	int AppGetCB();
	int AppMessageLoop();

//	static UINT ThreadDrawRGBFrame(LPVOID lpParam);

//	static void CallBack_RGB24(unsigned char *pucRGB24,void *pContext);
//	static void CallBack_XYZRGB(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud,void *pContext);
//	static void CallBack_RGB(const boost::shared_ptr<openni_wrapper::Image>& img,void *pContext);



// Overrides
public:
	virtual BOOL InitInstance();

	afx_msg void ShowViewInMap();
	afx_msg void OnlyShownMap();
	afx_msg void OnlyShownView();
	afx_msg void ShowMapInView();
	DECLARE_MESSAGE_MAP()


	CDocTemplate * m_pDisplayTemplate;
	CDocTemplate * m_pOptionTemplate;
	CDocTemplate * m_pMessageTemplate;


	//	CChildFrame* p11Frame;
	//	CChildFrame* p12Frame;
	//	CChildFrame* p13Frame;



public:
	CStatic m_staticImg;
	//CStatic m_staticImg2;
	//static bool m_bRun;
	RECT m_rectImg;

	//int m_nCurrentShowStyle;

	IoTRobot_GUIParam m_stInterfaceParam;
	
};
extern CIoTRobot_GUIApp theApp;