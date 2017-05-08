// IoTRobot_GUI.cpp : Defines the class behaviors for the application.
//

#include "stdafx.h"
#include "IoTRobot_GUI.h"
#include "MainFrm.h"

#include "IoTRobot_GUIDoc.h"
#include "IoTRobot_GUIView.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#endif


// CIoTRobot_GUIApp

BEGIN_MESSAGE_MAP(CIoTRobot_GUIApp, CWinApp)
	ON_COMMAND(ID_APP_ABOUT, &CIoTRobot_GUIApp::OnAppAbout)
	// Standard file based document commands
	ON_COMMAND(ID_FILE_NEW, &CWinApp::OnFileNew)
	ON_COMMAND(ID_FILE_OPEN, &CWinApp::OnFileOpen)
	// Standard print setup command
	ON_COMMAND(ID_FILE_PRINT_SETUP, &CWinApp::OnFilePrintSetup)
	ON_COMMAND(ID_BUTTON_IMU,&CIoTRobot_GUIApp::OnIMU)
	ON_COMMAND(ID_BUTTON_MANUAL_MB,&CIoTRobot_GUIApp::OnManualMB)

	ON_COMMAND(ID_BUTTON_SLAMSETTING,&CIoTRobot_GUIApp::OnSLAMSetting)
	ON_COMMAND(ID_BUTTON_MESSAGE_LIST,&CIoTRobot_GUIApp::OnMessageBox)
	ON_COMMAND(ID_BUTTON_CONTROL_BUTTON,&CIoTRobot_GUIApp::OnOptionButtons)
	ON_COMMAND(IDC_BUTTON_IDRIVE,&CIoTRobot_GUIApp::OnIDrive)
	ON_COMMAND(IDC_BUTTON_AUTO,&CIoTRobot_GUIApp::OnAutoDrive)
	ON_COMMAND(ID_BUTTON_SHOW_WHOLE_MAP,&CIoTRobot_GUIApp::OnShowWholeMap)
	ON_COMMAND(ID_BUTTON_2DMAP,&CIoTRobot_GUIApp::OnHorizonCutPC)
	ON_COMMAND(ID_BUTTON_STOP,&CIoTRobot_GUIApp::OnEmergencyBreak)
	ON_COMMAND(ID_BUTTON_RESET_SESSION,&CIoTRobot_GUIApp::OnResetSession)
	ON_COMMAND(ID_BUTTON_RESET_ROBOT,&CIoTRobot_GUIApp::OnResetRobot)
END_MESSAGE_MAP()


// CIoTRobot_GUIApp construction

CIoTRobot_GUIApp::CIoTRobot_GUIApp()
{
	// TODO: add construction code here,
	// Place all significant initialization in InitInstance
	m_nIsCnc=0;
	m_bShowGlobalMap=false;
	m_nIDirveValid=1;
}


// The one and only CIoTRobot_GUIApp object

CIoTRobot_GUIApp theApp;

int CIoTRobot_GUIApp::AppGetCB()
{
	m_stInterfaceParam.pPIPCallBack=CIoTRobot_GUIView::CallBack_RobotView;
	m_stInterfaceParam.pPCCallBack=OpenGL::CallBack_PointCloud;
	m_stInterfaceParam.pSLAMPCCallBack=OpenGL::CallBack_SLAM_PC;
	m_stInterfaceParam.pStateCallBack=OpenGL::CallBack_RunState;
	m_stInterfaceParam.pPathCallBack=OpenGL::CallBack_Path;
	m_stInterfaceParam.pData2OpenGL=OpenGL::CallBack_GetDataFromNetServer;

	m_stInterfaceParam.pPlaneCallBack=OpenGL::CallBack_Session_Planes;
	return 1;
}
// CIoTRobot_GUIApp initialization
int CIoTRobot_GUIApp::AppMessageLoop()
{
	AFX_MANAGE_STATE(AfxGetStaticModuleState()); 
	theApp.Run();
	return 1;
}
int CIoTRobot_GUIApp::AppRun(void *pGUICmd)
{

	AFX_MANAGE_STATE(AfxGetStaticModuleState()); 
	// InitCommonControlsEx() is required on Windows XP if an application
	// manifest specifies use of ComCtl32.dll version 6 or later to enable
	// visual styles.  Otherwise, any window creation will fail.
	INITCOMMONCONTROLSEX InitCtrls;
	InitCtrls.dwSize = sizeof(InitCtrls);
	// Set this to include all the common control classes you want to use
	// in your application.
	InitCtrls.dwICC = ICC_WIN95_CLASSES;
	InitCommonControlsEx(&InitCtrls);

	CWinApp::InitInstance();

	// Initialize OLE libraries
	if (!AfxOleInit())
	{
		AfxMessageBox(IDP_OLE_INIT_FAILED);
		return FALSE;
	}
	AfxEnableControlContainer();
	// Standard initialization
	// If you are not using these features and wish to reduce the size
	// of your final executable, you should remove from the following
	// the specific initialization routines you do not need
	// Change the registry key under which our settings are stored
	// TODO: You should modify this string to be something appropriate
	// such as the name of your company or organization
	SetRegistryKey(_T("Local AppWizard-Generated Applications"));
	LoadStdProfileSettings(4);  // Load standard INI file options (including MRU)
	// Register the application's document templates.  Document templates
	//  serve as the connection between documents, frame windows and views
	CSingleDocTemplate* pDocTemplate;
	pDocTemplate = new CSingleDocTemplate(
		IDR_MAINFRAME,
		RUNTIME_CLASS(CIoTRobot_GUIDoc),
		RUNTIME_CLASS(CMainFrame),       // main SDI frame window
		RUNTIME_CLASS(CIoTRobot_GUIView));
	if (!pDocTemplate)
		return FALSE;
	AddDocTemplate(pDocTemplate);



	// Parse command line for standard shell commands, DDE, file open
	CCommandLineInfo cmdInfo;
	ParseCommandLine(cmdInfo);


	// Dispatch commands specified on the command line.  Will return FALSE if
	// app was launched with /RegServer, /Register, /Unregserver or /Unregister.
	if (!ProcessShellCommand(cmdInfo))
		return FALSE;

	// The one and only window has been initialized, so show and update it
	m_pMainWnd->ShowWindow(SW_SHOW);
	m_pMainWnd->UpdateWindow();

	m_hWnd=m_pMainWnd->GetSafeHwnd();

	CMenu * pmenu = ((CMainFrame *)AfxGetMainWnd())->GetMenu();
	CMenu * psub = pmenu->GetSubMenu(2);
	psub->EnableMenuItem(ID_VIEW_DISPLAYBAR, MF_ENABLED );
	psub ->CheckMenuItem(2,MF_BYPOSITION| MF_CHECKED);



//	m_pstGUICmd=(IoTRobot_GUICMD *)pGUICmd;
//	m_pstGUICmd->nSLAMCMD=1;
//	OpenGL::m_bUseIMU=m_pstGUICmd->nSLAMCMD;

	memset(&stGUICmd,0,sizeof(IoTRobot_GUICMD));


	// call DragAcceptFiles only if there's a suffix
	//  In an SDI app, this should occur after ProcessShellCommand
	return TRUE;
}



// CAboutDlg dialog used for App About

class CAboutDlg : public CDialog
{
public:
	CAboutDlg();

// Dialog Data
	enum { IDD = IDD_ABOUTBOX };

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support

// Implementation
protected:
	DECLARE_MESSAGE_MAP()
};

CAboutDlg::CAboutDlg() : CDialog(CAboutDlg::IDD)
{
}

void CAboutDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
}

BEGIN_MESSAGE_MAP(CAboutDlg, CDialog)
END_MESSAGE_MAP()

// App command to run the dialog
void CIoTRobot_GUIApp::OnAppAbout()
{
	CAboutDlg aboutDlg;
	aboutDlg.DoModal();
}

void CIoTRobot_GUIApp::OnManualMB()
{
	IoTRobot_Message MSG;
	MSG.cCommand=SLAM_MSG_REBUILD_MAP;
	MSG.cDestModule=SLAM_MODULE;
	MSG.cFromModule=GUI_MODULE;
	MSG.nParam1=2;
	MSG.nParam2=0;


	//stGUICmd.stSLAMCMD.cOpenIMU=(stGUICmd.stSLAMCMD.cOpenIMU?false:true);
	//MSG.nParam1=stGUICmd.stSLAMCMD.cOpenIMU;
	m_pCSendCMD2Kernel(&MSG);
}



void CIoTRobot_GUIApp::OnSLAMSetting()
{
	IoTRobot_Message MSG;
	MSG.cCommand=SLAM_MSG_SETTINGS;
	MSG.cDestModule=SLAM_MODULE;
	MSG.cFromModule=GUI_MODULE;
	MSG.nParam1=0;
	MSG.nParam2=0;


	m_CDlgSlamSetting.DoModal();

	m_pCSendCMD2Kernel(&MSG);
}



void CIoTRobot_GUIApp::OnIMU()
{
	IoTRobot_Message MSG;
	MSG.cCommand=SLAM_MSG_IMU;
	MSG.cDestModule=SLAM_MODULE;
	MSG.cFromModule=GUI_MODULE;
	MSG.nParam1=0;
	MSG.nParam2=0;


	stGUICmd.stSLAMCMD.cOpenIMU=(stGUICmd.stSLAMCMD.cOpenIMU?false:true);
	MSG.nParam1=stGUICmd.stSLAMCMD.cOpenIMU;
	m_pCSendCMD2Kernel(&MSG);

//	CMainFrame *pMainFrm=(CMainFrame *)theApp.m_pMainWnd;
//	pMainFrm->m_wndDisplayToolBar.GetToolBarCtrl().SetState(ID_BUTTON_IMU,TBSTATE_INDETERMINATE);
}

int CIoTRobot_GUIApp::UpdateMobileState(IoTRobot_MobileState* pstMobileState)
{
	BOOL bRtn;
	if (pstMobileState->cType==0)
	{
		m_nIsCnc=1;
		if (pstMobileState->stDeviceState.cIMU==0)
		{
			CMainFrame *pMainFrm=(CMainFrame *)theApp.m_pMainWnd;
			pMainFrm->m_bIMUApply=false;
			//bRtn=pMainFrm->m_wndControlToolBar.GetToolBarCtrl().EnableButton(ID_BUTTON_IMU,FALSE);
			//bRtn=pMainFrm->m_wndDisplayToolBar.GetToolBarCtrl().EnableButton(ID_BUTTON_COMPASS,FALSE);
			pMainFrm->m_wndControlToolBar.GetToolBarCtrl().SetState(ID_BUTTON_IMU,TBSTATE_INDETERMINATE);
			pMainFrm->m_wndDisplayToolBar.GetToolBarCtrl().SetState(ID_BUTTON_COMPASS,TBSTATE_INDETERMINATE);
			CIoTRobot_GUIView *pView=(CIoTRobot_GUIView *)pMainFrm->GetActiveView(); 
			pView->m_cOpenGL.m_bCompass=false;
		}
	}
/*	else if (pstMobileState->cType==1)
	{
		if (pstMobileState->stDeviceState.cIMU==0)
		{
			m_nIDirveValid=0;
			CMainFrame *pMainFrm=(CMainFrame *)theApp.m_pMainWnd;
			CIoTRobot_GUIView *pView=(CIoTRobot_GUIView *)pMainFrm->GetActiveView(); 
			if (pView->m_bOpenRobotView)
			{
				pView->OnDisplayPC_View();
			}
			IoTRobot_Message MSG;
			//stop idrive
			MSG.cCommand=3;
			MSG.cDestModule=4;
			MSG.cFromModule=GUI_MODULE;
			MSG.nParam1=0;
			MSG.nParam2=0;
			m_pCSendCMD2Kernel(&MSG);
			Sleep(20);

			Sleep(20);
		}
		else
		{
			m_nIDirveValid=1;
		}
		
	}*/
	return 0;
}

void CIoTRobot_GUIApp::OnMessageBox()
{
	CMainFrame *pMainFrm=(CMainFrame *)theApp.m_pMainWnd;
	pMainFrm->ShowControlBar(&pMainFrm->m_wndMessageBar,!pMainFrm->m_wndMessageBar.IsVisible(),false);
}
void CIoTRobot_GUIApp::OnOptionButtons()
{
	CMainFrame *pMainFrm=(CMainFrame *)theApp.m_pMainWnd;
	pMainFrm->ShowControlBar(&pMainFrm->m_wndOptionBar,!pMainFrm->m_wndOptionBar.IsVisible(),false);
}


void CIoTRobot_GUIApp::OnIDrive()
{
//	if (m_nIDirveValid==1)
	{
		IoTRobot_Message MSG;
		HWND hwnd=m_pMainWnd->GetSafeHwnd();
		MSG.cCommand=HOSTPORTAL_MSG_IDRIVE;
		MSG.cDestModule=HOSTPORTAL_MODULE;
		MSG.cFromModule=GUI_MODULE;
		MSG.nParam1=0;
		MSG.nParam2=0;
		m_pCSendCMD2Kernel(&MSG);


		MSG.cCommand=3;
		MSG.cDestModule=4;
		MSG.cFromModule=GUI_MODULE;
		MSG.nParam1=1;
		MSG.nParam2=0;
		MSG.pcParam3=(char *)m_hWnd;
		m_pCSendCMD2Kernel(&MSG);

		//	m_bStopTransferCtrlCmd=false;
		//	LPDWORD ID=0;
		//	::CreateThread(NULL,0,(LPTHREAD_START_ROUTINE)ThreadTransferCtrlCmd,this,0,ID);

		/*	MSG.cCommand=HOSTPORTAL_MSG_IDRIVE;
		MSG.cDestModule=HOSTPORTAL_MODULE;
		MSG.cFromModule=GUI_MODULE;
		MSG.nParam1=0;
		MSG.nParam2=0;
		m_pCSendCMD2Kernel(&MSG);*/

		m_nCurrentRunMode=IDRIVE_MODE;
	}
}

void CIoTRobot_GUIApp::OnAutoDrive()
{


	//Path Info
	IoTRobot_Message MSG;
	CMainFrame *pMainFrm=(CMainFrame *)theApp.m_pMainWnd;
	CIoTRobot_GUIView *pView=(CIoTRobot_GUIView *)pMainFrm->GetActiveView(); 

	if (pView->m_cOpenGL.m_nManualPath)
	{
		AfxMessageBox("Plz Press ManualPath Button to Close Edit Path Function Firstly");
	}
	else
	{
		MSG.cCommand=HOSTPORTAL_MSG_SPECIFIEDPAH;
		MSG.cDestModule=HOSTPORTAL_MODULE;
		MSG.cFromModule=GUI_MODULE;
		MSG.nParam1=0;
		MSG.nParam2=pView->m_cOpenGL.m_nMileStoneLen;
		MSG.pcParam3=(char*)pView->m_cOpenGL.m_fMileStonePos;

		m_pCSendCMD2Kernel(&MSG);

		Sleep(20);

		//stop idrive
		MSG.cCommand=3;
		MSG.cDestModule=4;
		MSG.cFromModule=GUI_MODULE;
		MSG.nParam1=0;
		MSG.nParam2=0;
		m_pCSendCMD2Kernel(&MSG);
		Sleep(20);


		MSG.cCommand=HOSTPORTAL_MSG_AUTODRIVE;
		MSG.cDestModule=HOSTPORTAL_MODULE;
		MSG.cFromModule=GUI_MODULE;
		MSG.nParam1=0;
		MSG.nParam2=0;
		m_pCSendCMD2Kernel(&MSG);
		Sleep(20);
	}





	//m_bStopTransferCtrlCmd=true;
	//
}	

void CIoTRobot_GUIApp::OnShowWholeMap()
{
	m_bShowGlobalMap=(m_bShowGlobalMap?false:true);
	IoTRobot_Message MSG;
	MSG.cCommand=SLAM_MSG_IMU;
	MSG.cDestModule=DMAP_MODULE;
	MSG.cFromModule=GUI_MODULE;
	if (m_bShowGlobalMap)
	{
		MSG.nParam1=1;
		MSG.nParam2=0;
		if(m_nIsCnc)
			m_pCSendCMD2Kernel(&MSG);
		CMainFrame *pMainFrm=(CMainFrame *)theApp.m_pMainWnd;
		pMainFrm->m_wndDisplayToolBar.GetToolBarCtrl().SetState(ID_BUTTON_SHOW_WHOLE_MAP,TBSTATE_PRESSED);
	}
	else
	{
		MSG.nParam1=0;
		MSG.nParam2=0;
		if(m_nIsCnc)
			m_pCSendCMD2Kernel(&MSG);
		CMainFrame *pMainFrm=(CMainFrame *)theApp.m_pMainWnd;
		pMainFrm->m_wndDisplayToolBar.GetToolBarCtrl().SetState(ID_BUTTON_SHOW_WHOLE_MAP,TBSTYLE_FLAT);
	}
	
	//m_wndToolBar.GetToolBarCtrl().SetState(ID_BUTTON_XX,   TBSTATE_PRESSED);

}
// CIoTRobot_GUIApp message handlers

void CIoTRobot_GUIApp::OnHorizonCutPC()
{
	//CIoTRobot_GUIApp *pApp=(CIoTRobot_GUIApp *)pView->m_pApp;
	CMainFrame *pMainFrm=(CMainFrame *)theApp.m_pMainWnd;
	CIoTRobot_GUIView *pView=(CIoTRobot_GUIView *)pMainFrm->GetActiveView(); 
	pView->m_cOpenGL.m_nHorizonCutPC=(pView->m_cOpenGL.m_nHorizonCutPC?false:true);
	if (pView->m_cOpenGL.m_nHorizonCutPC)
	{
		pMainFrm->m_wndDisplayToolBar.GetToolBarCtrl().SetState(ID_BUTTON_2DMAP,TBSTATE_PRESSED);
	}
	else
	{
		pMainFrm->m_wndDisplayToolBar.GetToolBarCtrl().SetState(ID_BUTTON_2DMAP,TBSTYLE_FLAT);
	}
}


void CIoTRobot_GUIApp::OnEmergencyBreak()
{
	IoTRobot_Message MSG;
	MSG.cCommand=HOSTPORTAL_MSG_EMERGENCYBREAK;
	MSG.cDestModule=HOSTPORTAL_MODULE;
	MSG.cFromModule=GUI_MODULE;
	MSG.nParam1=0;
	MSG.nParam2=0;
	MSG.pcParam3=NULL;

	m_pCSendCMD2Kernel(&MSG);
}

/*
void CIoTRobot_GUIApp::OnControlStick()
{
	LPDWORD ID=0;
	::CreateThread(NULL,0,(LPTHREAD_START_ROUTINE)ThreadControlStick,this,0,ID);
}

UINT CIoTRobot_GUIApp::ThreadControlStick(LPVOID lpParam)
{
	AFX_MANAGE_STATE(AfxGetStaticModuleState()); 
	CIoTRobot_GUIApp *pApp=(CIoTRobot_GUIApp *)lpParam;
	pApp->m_CControlStick.DoModal();
	return 0;
}*/

void CIoTRobot_GUIApp::TransferCtrlCmd(int nAnlge,int nGear,int Rot_Self,int nType)
{
	//m_bNewCtrlCmdCome=true;

	printf("nGear:  %d    nAngle:   %d\n",nGear,nAnlge);
	memcpy(m_cCtrlCmd,&nAnlge,4);
	memcpy(m_cCtrlCmd+4,&nGear,4);
	memcpy(m_cCtrlCmd+8,&Rot_Self,4);
	memcpy(m_cCtrlCmd+12,&nType,4);

	IoTRobot_Message MSG;
	MSG.cCommand=12;
	MSG.cDestModule=HOSTPORTAL_MODULE;
	MSG.cFromModule=GUI_MODULE;
	MSG.pcParam3=m_cCtrlCmd;
	m_pCSendCMD2Kernel(&MSG);
}


void CIoTRobot_GUIApp::OnResetSession()
{
	IoTRobot_Message MSG;
	MSG.cCommand=CMDCTRL_MSG_RESETSESSION;
	MSG.cDestModule=CMDCTRL_MODULE;
	MSG.cFromModule=GUI_MODULE;
	MSG.pcParam3=0;
	MSG.nParam1=0;
	MSG.nParam2=0;
	m_pCSendCMD2Kernel(&MSG);
}


void CIoTRobot_GUIApp::OnResetRobot()
{

}
/*
UINT CIoTRobot_GUIApp::ThreadTransferCtrlCmd(LPVOID lpParam)
{

	CIoTRobot_GUIApp *pApp=(CIoTRobot_GUIApp *)lpParam;
	IoTRobot_Message MSG;
	MSG.cCommand=12;
	MSG.cDestModule=HOSTPORTAL_MODULE;
	MSG.cFromModule=GUI_MODULE;
	MSG.pcParam3=pApp->m_cCtrlCmd;
	while (!pApp->m_bStopTransferCtrlCmd)
	{
		if (pApp->m_bNewCtrlCmdCome)
		{
			pApp->m_pCSendCMD2Kernel(&MSG);
			pApp->m_bNewCtrlCmdCome=false;
		}
		Sleep(10);
	}
	return 0;
}*/