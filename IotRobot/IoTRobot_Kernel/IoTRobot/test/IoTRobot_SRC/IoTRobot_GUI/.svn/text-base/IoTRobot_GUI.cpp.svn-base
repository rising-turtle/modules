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
END_MESSAGE_MAP()


// CIoTRobot_GUIApp construction

CIoTRobot_GUIApp::CIoTRobot_GUIApp()
{
	// TODO: add construction code here,
	// Place all significant initialization in InitInstance
}


// The one and only CIoTRobot_GUIApp object

CIoTRobot_GUIApp theApp;

int CIoTRobot_GUIApp::AppGetCB()
{
	m_stInterfaceParam.pPIPCallBack=OpenGL::CallBack_PIPQVGA;
	m_stInterfaceParam.pPCCallBack=OpenGL::CallBack_PointCloud;
	m_stInterfaceParam.pSLAMPCCallBack=OpenGL::CallBack_SLAM_PC;
	m_stInterfaceParam.pStateCallBack=OpenGL::CallBack_RunState;
	m_stInterfaceParam.pPathCallBack=OpenGL::CallBack_Path;
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

	CMenu * pmenu = ((CMainFrame *)AfxGetMainWnd())->GetMenu();
	CMenu * psub = pmenu->GetSubMenu(2);
	psub->EnableMenuItem(ID_VIEW_DISPLAYBAR, MF_ENABLED );
	psub ->CheckMenuItem(2,MF_BYPOSITION| MF_CHECKED);



	m_pstGUICmd=(IoTRobot_GUICMD *)pGUICmd;
	m_pstGUICmd->nSLAMCMD=1;
	OpenGL::m_bUseIMU=m_pstGUICmd->nSLAMCMD;


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


void CIoTRobot_GUIApp::OnIMU()
{
	if (!OpenGL::m_bNoIMUInfo)
	{
		m_pstGUICmd->nSLAMCMD=(m_pstGUICmd->nSLAMCMD?false:true);
		OpenGL::m_bUseIMU=m_pstGUICmd->nSLAMCMD;
	}

}




// CIoTRobot_GUIApp message handlers

