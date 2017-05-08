// IoTRobot_GUI.h : main header file for the IoTRobot_GUI application
//
#pragma once

#ifndef __AFXWIN_H__
	#error "include 'stdafx.h' before including this file for PCH"
#endif

#include "resource.h"       // main symbols


// CIoTRobot_GUIApp:
// See IoTRobot_GUI.cpp for the implementation of this class
//

class CIoTRobot_GUIApp : public CWinApp
{
public:
	CIoTRobot_GUIApp();

	IoTRobot_GUICMD *m_pstGUICmd;

	int AppRun(void *pGUICmd);
	int AppGetCB();
	int AppMessageLoop();
	IoTRobot_GUIParam m_stInterfaceParam;
// Overrides
public:
	//virtual BOOL InitInstance();

// Implementation
	afx_msg void OnAppAbout();
	afx_msg void OnIMU();
	DECLARE_MESSAGE_MAP()
};

extern CIoTRobot_GUIApp theApp;