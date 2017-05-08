// IoTRobot_GUI.cpp : Defines the initialization routines for the DLL.
//

#include "stdafx.h"
#include "IoTRobot_GUI.h"
#include "MainFrm.h"

#include "ChildFrm.h"
#include "IoTRobot_GUIDoc.h"
#include "IoTRobot_GUIView.h"
#include "scbarg.h"

/*#include <vtkConeSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkImageViewer.h>
#include <../io/vtkBMPReader.h>
#include <../Filtering/vtkImageData.h>
#include <../Filtering/vtkPointData.h>

#include <vtkGraphicsFactory.h>
#include <vtkWin32RenderWindowInteractor.h>
#include <vtkCommand.h>



#include "scbarg.h"
#include "VTKTest.h"*/

#ifdef _DEBUG
#define new DEBUG_NEW
#endif

//pcl::visualization::CloudViewer * CIoTRobot_GUIApp::m_pcCloudViewer;

//
//TODO: If this DLL is dynamically linked against the MFC DLLs,
//		any functions exported from this DLL which call into
//		MFC must have the AFX_MANAGE_STATE macro added at the
//		very beginning of the function.
//
//		For example:
//
//		extern "C" BOOL PASCAL EXPORT ExportedFunction()
//		{
//			AFX_MANAGE_STATE(AfxGetStaticModuleState());
//			// normal function body here
//		}
//
//		It is very important that this macro appear in each
//		function, prior to any calls into MFC.  This means that
//		it must appear as the first statement within the 
//		function, even before any object variable declarations
//		as their constructors may generate calls into the MFC
//		DLL.
//
//		Please see MFC Technical Notes 33 and 58 for additional
//		details.
//

// CIoTRobot_GUIApp

BEGIN_MESSAGE_MAP(CIoTRobot_GUIApp, CWinApp)
	ON_COMMAND(ID_BUTTON_MAP_VIEW,&CIoTRobot_GUIApp::ShowViewInMap)
	ON_COMMAND(ID_BUTTON_MAP,&CIoTRobot_GUIApp::OnlyShownMap)
	ON_COMMAND(ID_BUTTON_VIEW,&CIoTRobot_GUIApp::OnlyShownView)
	ON_COMMAND(ID_BUTTON_VIEW_MAP,&CIoTRobot_GUIApp::ShowMapInView)
END_MESSAGE_MAP()


// CIoTRobot_GUIApp construction

//bool CIoTRobot_GUIApp::m_bRun;

CIoTRobot_GUIApp::CIoTRobot_GUIApp()
{
	//m_bRun=FALSE;
}


// The one and only CIoTRobot_GUIApp object

CIoTRobot_GUIApp theApp;

/*vtkImageViewer *g_vtkImageViewer;
vtkBMPReader *g_vtkBMPReader;
vtkImageData *g_vtkImageData;
unsigned char *g_RGBBuff;

typedef struct DrawFrame
{
	CIoTRobot_GUIApp *pGUIApp;
	RECT rect;
	CDC *pDC;
}DrawFrame;

DrawFrame g_stDrawFrame;


vtkImageViewer *imageViewer;
vtkBMPReader *BMPReader1,*BMPReader2;
UINT ThreadDrawFrame(LPVOID lpParam)
{
	DrawFrame *pstDrawFrame=(DrawFrame *)lpParam;
	unsigned char *pucGetFromMB=0;
	int count=0;



	Sleep(10000);
	BMPReader1=vtkBMPReader::New();
	BMPReader1->SetFileName("d:\\2.bmp");

	//	this->BMPReader1=vtkBMPReader::New();
	//	this->BMPReader1->SetFileName("d:\\1.bmp");

	imageViewer=vtkImageViewer::New();
	imageViewer->SetParentId(pstDrawFrame->pGUIApp->m_staticImg2.m_hWnd);

	imageViewer->SetColorLevel(100);
	imageViewer->SetColorWindow(256);
	imageViewer->SetSize(320,240);
	imageViewer->SetPosition(0,0);
	BMPReader1->Update();
	vtkImageData *imageData=BMPReader1->GetOutput();
	imageViewer->SetInput(imageData);
	unsigned char *data=(unsigned char *)imageData->GetScalarPointer();
	//	vtkRenderWindowInteractor *renin=vtkRenderWindowInteractor::New();

	//IoTInteractor *renin=IoTInteractor::New();
	//IoTInteractor *renin=vtkSmartPointer<IoTInteractor>::New ();
	//imageViewer->SetupInteractor(renin);
	//renin->Initialize();
	//renin->Start();

	int i=0;
	while (1)
	{
		//{
		//GetRGBData(g_GUIOmage0);

		//}

	
		i++;
		//imageData->Update();

	//	memcpy(data,g_GUIOmage0,320*240*3);
		//	imageData->Update();

		//if(g_bOpenImg))
		LPMSG msg=NULL;
		while (PeekMessage(msg,0,0,0,0))
		{
			TranslateMessage (msg);
			DispatchMessage (msg);
		}
		imageViewer->Render();
		//renin->Start();

		//memcpy(data,g_GUIOmage,50);
		//	memset(g_GUIOmage,0,640*480*3);
		//	if(g_bOpenImg)
		//	pstDrawFrame->pGUIApp->DrawFrameRGB24(g_GUIOmage0,320,240,pstDrawFrame->pDC,pstDrawFrame->rect);
		Sleep(10);
	}


	return 1;
}*/

int CIoTRobot_GUIApp::AppGetCB()
{
	//CMainFrame *pMain=(CMainFrame *)AfxGetApp()->m_pMainWnd;
	CIoTRobot_GUIView *pView=(CIoTRobot_GUIView *)((CMainFrame *)m_pMainWnd)->GetActiveView();
	m_stInterfaceParam.pPIPCallBack=pView->CallBack_PIPQVGA;
	m_stInterfaceParam.pPCCallBack=pView->CallBack_PointCloud;
	return 1;
}
int CIoTRobot_GUIApp::AppMessageLoop()
{
	AFX_MANAGE_STATE(AfxGetStaticModuleState()); 
	theApp.Run();
	return 1;
}

bool g_bStart2Run=false;
RECT m_rectImg2;
static HWND ParentHWnd;
int CIoTRobot_GUIApp::AppRun()
{
	AFX_MANAGE_STATE(AfxGetStaticModuleState()); 
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
	CMultiDocTemplate* pDocTemplate;
	pDocTemplate = new CMultiDocTemplate(IDR_IoTRobot_GUITYPE,
		RUNTIME_CLASS(CIoTRobot_GUIDoc),
		RUNTIME_CLASS(CChildFrame), // custom MDI child frame
		RUNTIME_CLASS(CIoTRobot_GUIView));
	if (!pDocTemplate)
		return FALSE;
	AddDocTemplate(pDocTemplate);

	// create main MDI Frame window
	CMainFrame* pMainFrame = new CMainFrame;
	if (!pMainFrame || !pMainFrame->LoadFrame(IDR_MAINFRAME))
	{
		DWORD dwError =0;

		dwError=GetLastError();
		delete pMainFrame;
		return FALSE;
	}
	m_pMainWnd = pMainFrame;
	// call DragAcceptFiles only if there's a suffix
	//  In an MDI app, this should occur immediately after setting m_pMainWnd


	// Parse command line for standard shell commands, DDE, file open
	CCommandLineInfo cmdInfo;
	ParseCommandLine(cmdInfo);


	// Dispatch commands specified on the command line.  Will return FALSE if
	// app was launched with /RegServer, /Register, /Unregserver or /Unregister.
	if (!ProcessShellCommand(cmdInfo))
		return FALSE;
	// The main window has been initialized, so show and update it
	pMainFrame->ShowWindow(m_nCmdShow);
	//pMainFrame->ShowWindow(SW_SHOWMAXIMIZED);
	pMainFrame->UpdateWindow();



	/*if (!pMainFrame->m_wndOptionBar.Create(_T("OptionBar"), pMainFrame, CSize(240,100),TRUE,123))
	{
		TRACE0("Failed to create mybar\n");
		return -1;
	}
	if (!pMainFrame->m_wndMessageBar.Create(_T("MessageBar"), pMainFrame,CSize(240,100),TRUE,124))
	{
		TRACE0("Failed to create mybar\n");
		return -1;
	}


	pMainFrame->m_wndOptionBar.SetBarStyle(pMainFrame->m_wndOptionBar.GetBarStyle() |
		CBRS_TOOLTIPS | CBRS_FLYBY | CBRS_SIZE_DYNAMIC |CBRS_BORDER_ANY );

	pMainFrame->m_wndMessageBar.SetBarStyle(pMainFrame->m_wndMessageBar.GetBarStyle() |
		CBRS_TOOLTIPS | CBRS_FLYBY | CBRS_SIZE_DYNAMIC |CBRS_BORDER_ANY);



	pMainFrame->m_wndOptionBar.EnableDocking(CBRS_ALIGN_ANY);
	pMainFrame->m_wndMessageBar.EnableDocking(CBRS_ALIGN_ANY);





	CRect rc(0, 0, 0, 0);
	CSize sizeMax(0, 0);


	pMainFrame->m_DlgOptions.Create(IDD_DLG_OPTIONS,&pMainFrame->m_wndOptionBar);

	pMainFrame->m_DlgOptions.UpdateWindow();
	if(!pMainFrame->m_wndEdit.Create(WS_VSCROLL|WS_CHILD|WS_VISIBLE|ES_AUTOVSCROLL|ES_MULTILINE|ES_WANTRETURN,
		CRect(0,0,0,0),&pMainFrame->m_wndMessageBar,101))
		return -1;
	pMainFrame->m_wndEdit.ModifyStyleEx(0,WS_EX_CLIENTEDGE);


	CRect rectMainFrm;
	CRect rectMessageBar;

	pMainFrame->DockControlBar(&pMainFrame->m_wndMessageBar, AFX_IDW_DOCKBAR_RIGHT);///Í£¿¿ÔÚÓÒ±ß
	pMainFrame->RecalcLayout();

	pMainFrame->GetWindowRect(rectMainFrm);
	pMainFrame->m_wndMessageBar.GetWindowRect(rectMessageBar);
	rectMessageBar.OffsetRect(0, 1);//
	pMainFrame->DockControlBar(&pMainFrame->m_wndOptionBar, AFX_IDW_DOCKBAR_RIGHT,rectMessageBar);///Ò²Í£¿¿ÔÚÓÒ±ß




	if (!pMainFrame->m_wndDisplayBar.Create(_T("DisplayBar"), pMainFrame,CSize(rectMessageBar.left-rectMainFrm.left,600),TRUE,125))
	{
		TRACE0("Failed to create mybar\n");
		return -1;
	}

	pMainFrame->m_wndDisplayBar.SetBarStyle(pMainFrame->m_wndDisplayBar.GetBarStyle() |
		CBRS_TOOLTIPS | CBRS_FLYBY | CBRS_SIZE_DYNAMIC |CBRS_BORDER_ANY);
	pMainFrame->m_wndDisplayBar.EnableDocking(CBRS_ALIGN_ANY);

	pMainFrame->DockControlBar(&pMainFrame->m_wndDisplayBar,AFX_IDW_DOCKBAR_LEFT);




	printf("Mainframe  width  height  : %d   %d \n",rectMainFrm.right,  rectMainFrm.bottom);
	//RECT rectImg;
	m_rectImg.left=0;
	m_rectImg.top=0;
	m_rectImg.bottom=rectMessageBar.bottom;
	m_rectImg.right=rectMessageBar.left-rectMainFrm.left;

	m_staticImg.Create(L"",WS_VISIBLE|WS_CHILD|SS_BLACKFRAME|WS_CLIPCHILDREN,m_rectImg,&pMainFrame->m_wndDisplayBar,IDC_STATIC_IMG1);
	
	//pMainFrame->UpdateWindow();
	//CIoTRobot_GUIView *pView=(CIoTRobot_GUIView *)((CMainFrame *)m_pMainWnd)->GetActiveView();
	//pView->InitializeOpenGL();

/*	m_rectImg2.bottom=240;
	m_rectImg2.right=rectMessageBar.left-rectMainFrm.left;
	m_rectImg2.left=m_rectImg2.right-320;
	m_rectImg2.top=0;

	m_staticImg2.Create(L"",WS_VISIBLE|WS_CHILD|SS_WHITEFRAME|WS_BORDER|WS_CLIPSIBLINGS,m_rectImg2,&m_staticImg,IDC_STATIC_IMG2);

*/


	/*m_rectImg2.bottom=240;
	m_rectImg2.right=rectMessageBar.left-rectMainFrm.left;
	m_rectImg2.left=m_rectImg2.right-320;
	m_rectImg2.top=0;

	m_staticImg2.Create(L"",WS_VISIBLE|WS_CHILD|SS_WHITEFRAME|WS_BORDER|WS_CLIPSIBLINGS,m_rectImg2,&pMainFrame->m_wndDisplayBar,IDC_STATIC_IMG2);




	m_rectImg.left=0;
	m_rectImg.top=0;
	m_rectImg.bottom=rectMessageBar.bottom;
	m_rectImg.right=rectMessageBar.left-rectMainFrm.left;

	m_staticImg.Create(L"",WS_VISIBLE|WS_CHILD|SS_BLACKFRAME|WS_CLIPCHILDREN,m_rectImg,&m_staticImg2,IDC_STATIC_IMG1);*/

//	pMainFrame->UpdateWindow();



/*	g_vtkBMPReader=vtkBMPReader::New();
	g_vtkBMPReader->SetFileName("d:\\2.bmp");


	g_vtkImageViewer=vtkImageViewer::New();
	g_vtkImageViewer->SetParentId(m_staticImg.m_hWnd);

	g_vtkImageViewer->SetColorLevel(100);



	g_vtkImageViewer->SetColorWindow(256);
	g_vtkImageViewer->SetSize(320,240);
	g_vtkImageViewer->SetPosition(0,0);
	g_vtkBMPReader->Update();
	g_vtkImageData=g_vtkBMPReader->GetOutput();
	g_vtkImageViewer->SetInput(g_vtkImageData);

	//SetWindowPos(m_staticImg2.m_hWnd,HWND_TOPMOST,m_rectImg2.left,m_rectImg2.top,m_rectImg2.right,m_rectImg2.bottom,SWP_SHOWWINDOW);


	//LPDWORD ID=0;
	//::CreateThread(NULL,0,(LPTHREAD_START_ROUTINE)ThreadDrawFrame,&g_stDrawFrame,0,ID);


	ParentHWnd=m_staticImg2.m_hWnd;
	pMainFrame->UpdateWindow();
	g_bStart2Run=true;*/
	return 1;
}

// CIoTRobot_GUIApp initialization
BOOL CIoTRobot_GUIApp::InitInstance()
{
	CWinApp::InitInstance();
	return TRUE;
}


/*void CIoTRobot_GUIApp::CallBack_XYZRGB(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud,void *pContext)
{
	//m_pcCloudViewer->showCloud(cloud);
}

int g_nFinishShowThisFrame;
int g_nCurrentShowStyle=0;

void CIoTRobot_GUIApp::CallBack_RGB24(unsigned char *pucRGB24,void *pContext)
{
	//printf("Visulization call back run!!!!\n");
	

}
void CIoTRobot_GUIApp::CallBack_RGB(const boost::shared_ptr<openni_wrapper::Image>& img,void *pContext)
{
	if(m_bRun)
	{
		img->fillRGB(640,480,(unsigned char *)g_vtkImageData->GetScalarPointer());
		LPMSG msg=NULL;
		while (PeekMessage(msg,0,0,0,0))
		{
			TranslateMessage (msg);
			DispatchMessage (msg);
		}
		g_vtkImageViewer->Render();
	}

}*/


void CIoTRobot_GUIApp::ShowViewInMap()
{

}


//Current Show Type 0
void CIoTRobot_GUIApp::OnlyShownMap()
{

}


//Current Show Type 2
void CIoTRobot_GUIApp::OnlyShownView()
{

}


//Current Show Type 3
void CIoTRobot_GUIApp::ShowMapInView()
{

}

/*typedef struct DrawFrame
{
	CIoTRobot_GUIApp *pGUIApp;
	RECT rect;
	CDC *pDC;
}DrawFrame;
DrawFrame g_stDrawFrame;



UINT CIoTRobot_GUIApp::ThreadDrawRGBFrame(LPVOID lpParam)
{
	BMPReader1=vtkBMPReader::New();
	BMPReader1->SetFileName("d:\\1.bmp");


	imageViewer=vtkImageViewer::New();
	imageViewer->SetParentId(pstDrawFrame->pGUIApp->m_staticImg2.m_hWnd);

	imageViewer->SetColorLevel(100);
	imageViewer->SetColorWindow(256);
	imageViewer->SetSize(320,240);
	imageViewer->SetPosition(0,0);
	BMPReader1->Update();
	vtkImageData *imageData=BMPReader1->GetOutput();
	imageViewer->SetInput(imageData);
	unsigned char *data=(unsigned char *)imageData->GetScalarPointer();


	return 1;
}
*/