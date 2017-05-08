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


#include "DlgSLAMSetting.h"
#include "DlgControlStick.h"
class CIoTRobot_GUIApp : public CWinApp
{
public:
	CIoTRobot_GUIApp();

	IoTRobot_GUICMD stGUICmd;

	int AppRun(void *pGUICmd);
	int AppGetCB();
	int AppMessageLoop();
	int UpdateMobileState(IoTRobot_MobileState* pstMobileState);
	IoTRobot_GUIParam m_stInterfaceParam;
	SendCMD2Kernel m_pCSendCMD2Kernel;
	CDlgSLAMSetting m_CDlgSlamSetting;


	int m_nCurrentRunMode;
	int m_nIsCnc;

	HWND m_hWnd;
// Overrides
public:
	//virtual BOOL InitInstance();

// Implementation
	afx_msg void OnAppAbout();
	afx_msg void OnIMU();
	afx_msg void OnManualMB();
	
	afx_msg void OnSLAMSetting();

	afx_msg void OnMessageBox();
	afx_msg void OnOptionButtons();
	afx_msg void OnIDrive();
	afx_msg void OnAutoDrive();
	afx_msg void OnShowWholeMap();
	afx_msg void OnHorizonCutPC();
	afx_msg void OnControlStick();
	afx_msg void OnResetSession();
	afx_msg void OnResetRobot();

	afx_msg void OnEmergencyBreak();
	DECLARE_MESSAGE_MAP()

	bool m_bShowGlobalMap;
	CDlgControlStick m_CControlStick;


//	bool m_bStopTransferCtrlCmd;
	char m_cCtrlCmd[20];
//	bool m_bNewCtrlCmdCome;
	static UINT ThreadControlStick(LPVOID lpParam);
//	static UINT ThreadTransferCtrlCmd(LPVOID lpParam);

public:
	void TransferCtrlCmd(int nAnlge,int nGear,int Rot_Self,int nType);

	int m_nIDirveValid;


//	virtual BOOL PreTranslateMessage(MSG* pMsg);
};

extern CIoTRobot_GUIApp theApp;