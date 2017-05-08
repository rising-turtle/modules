// MainFrm.cpp : implementation of the CMainFrame class
//

#include "stdafx.h"
#include "IoTRobot_GUI.h"

#include "MainFrm.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#endif


// CMainFrame

IMPLEMENT_DYNCREATE(CMainFrame, CFrameWnd)

BEGIN_MESSAGE_MAP(CMainFrame, CFrameWnd)
	ON_WM_CREATE()
	ON_WM_LBUTTONDOWN()
/*	ON_UPDATE_COMMAND_UI(ID_VIEW_DISPLAYBAR, &CMainFrame::OnUpdateViewDisplaybar)
	ON_COMMAND(ID_VIEW_DISPLAYBAR, &CMainFrame::OnViewDisplaybar)
	ON_COMMAND(ID_VIEW_CONTROLBAR, &CMainFrame::OnViewControlbar)
	ON_UPDATE_COMMAND_UI(ID_VIEW_CONTROLBAR, &CMainFrame::OnUpdateViewControlbar)

	ON_COMMAND(ID_BUTTON_MESSAGE_LIST,&CMainFrame::OnMessageBar)
	ON_COMMAND(ID_BUTTON_CONTROL_BUTTON,&CMainFrame::OnControlBar)
	ON_UPDATE_COMMAND_UI(ID_DISPLAYBAR_IMU, &CMainFrame::OnUpdateDisplaybarImu)
	ON_COMMAND(ID_DISPLAYBAR_IMU, &CMainFrame::OnDisplaybarImu)
	ON_COMMAND(ID_DISPLAYBAR_MANUALMAP, &CMainFrame::OnDisplaybarManualmap)
	ON_UPDATE_COMMAND_UI(ID_DISPLAYBAR_MANUALMAP, &CMainFrame::OnUpdateDisplaybarManualmap)*/
	ON_COMMAND(ID_CONTROLBAR_IMU, &CMainFrame::OnControlbarImu)
	ON_UPDATE_COMMAND_UI(ID_CONTROLBAR_IMU, &CMainFrame::OnUpdateControlbarImu)
	ON_COMMAND(ID_CONTROLBAR_MANUALMAPBUILDER, &CMainFrame::OnControlbarManualmapbuilder)
	ON_UPDATE_COMMAND_UI(ID_CONTROLBAR_MANUALMAPBUILDER, &CMainFrame::OnUpdateControlbarManualmapbuilder)
	ON_COMMAND(ID_CONTROLBAR_SLAMSETTINGS, &CMainFrame::OnControlbarSlamsettings)
	ON_UPDATE_COMMAND_UI(ID_CONTROLBAR_SLAMSETTINGS, &CMainFrame::OnUpdateControlbarSlamsettings)
	ON_COMMAND(ID_DISPLAYBAR_PATH, &CMainFrame::OnDisplaybarPath)
	ON_UPDATE_COMMAND_UI(ID_DISPLAYBAR_PATH, &CMainFrame::OnUpdateDisplaybarPath)
	ON_COMMAND(ID_DISPLAYBAR_PIP_BUTTON, &CMainFrame::OnDisplaybarPipButton)
	ON_UPDATE_COMMAND_UI(ID_DISPLAYBAR_PIP_BUTTON, &CMainFrame::OnUpdateDisplaybarPipButton)
	ON_COMMAND(ID_DISPLAYBAR_STATUSLIGHT, &CMainFrame::OnDisplaybarStatuslight)
	ON_UPDATE_COMMAND_UI(ID_DISPLAYBAR_STATUSLIGHT, &CMainFrame::OnUpdateDisplaybarStatuslight)
	ON_COMMAND(ID_TOOLBAR_MESSAGEBOX, &CMainFrame::OnToolbarMessagebox)
	ON_UPDATE_COMMAND_UI(ID_TOOLBAR_MESSAGEBOX, &CMainFrame::OnUpdateToolbarMessagebox)
	ON_COMMAND(ID_TOOLBAR_OPTIONBUTTONS, &CMainFrame::OnToolbarOptionbuttons)
	ON_UPDATE_COMMAND_UI(ID_TOOLBAR_OPTIONBUTTONS, &CMainFrame::OnUpdateToolbarOptionbuttons)
	ON_COMMAND(ID_DISPLAYBAR_COMPASS, &CMainFrame::OnDisplaybarCompass)
	ON_UPDATE_COMMAND_UI(ID_DISPLAYBAR_COMPASS, &CMainFrame::OnUpdateDisplaybarCompass)
	ON_COMMAND(ID_DISPLAYBAR_TRIANGULATION, &CMainFrame::OnDisplaybarTriangulation)
	ON_UPDATE_COMMAND_UI(ID_DISPLAYBAR_TRIANGULATION, &CMainFrame::OnUpdateDisplaybarTriangulation)
	ON_WM_CLOSE()
END_MESSAGE_MAP()

static UINT indicators[] =
{
	ID_SEPARATOR,           // status line indicator
	ID_INDICATOR_CAPS,
	ID_INDICATOR_NUM,
	ID_INDICATOR_SCRL,
};


// CMainFrame construction/destruction

CMainFrame::CMainFrame()
{
	m_bIMUApply=true;
	// TODO: add member initialization code here
}

CMainFrame::~CMainFrame()
{
}


void   CMainFrame::DockControlBarLeftOf(CControlBar*   Bar,   CControlBar*   LeftOf) 
{ 
	CRect   rect; 
	DWORD   dw; 
	UINT   n; 

	//   get   MFC   to   adjust   the   dimensions   of   all   docked   ToolBars 
	//   so   that   GetWindowRect   will   be   accurate 
	RecalcLayout(TRUE); 

	LeftOf-> GetWindowRect(&rect); 
	rect.OffsetRect(1,0); 
	dw=LeftOf-> GetBarStyle(); 
	n   =   0; 
	n   =   (dw&CBRS_ALIGN_TOP)   ?   AFX_IDW_DOCKBAR_TOP :   n; 
	n   =   (dw&CBRS_ALIGN_BOTTOM &&   n==0)   ?   AFX_IDW_DOCKBAR_BOTTOM :   n; 
	n   =   (dw&CBRS_ALIGN_LEFT &&   n==0)   ?   AFX_IDW_DOCKBAR_LEFT :   n; 
	n   =   (dw&CBRS_ALIGN_RIGHT &&   n==0)   ?   AFX_IDW_DOCKBAR_RIGHT :   n; 

	//   When   we   take   the   default   parameters   on   rect,   DockControlBar   will   dock 
	//   each   Toolbar   on   a   seperate   line.   By   calculating   a   rectangle,   we   in   effect 
	//   are   simulating   a   Toolbar   being   dragged   to   that   location   and   docked. 
	DockControlBar(Bar,n,&rect); 
}
int CMainFrame::OnCreate(LPCREATESTRUCT lpCreateStruct)
{
	if (CFrameWnd::OnCreate(lpCreateStruct) == -1)
		return -1;
	

	CMainFrame:: m_bAutoMenuEnable=false;
	if (!m_wndToolBar.CreateEx(this, TBSTYLE_FLAT, WS_CHILD | WS_VISIBLE | CBRS_TOP
		| CBRS_GRIPPER | CBRS_TOOLTIPS | CBRS_FLYBY | CBRS_SIZE_DYNAMIC) ||
		!m_wndToolBar.LoadToolBar(IDR_MAINFRAME))
	{
		TRACE0("Failed to create toolbar\n");
		return -1;      // fail to create
	}




	if (!m_wndDisplayToolBar.CreateEx(this, TBSTYLE_FLAT, WS_CHILD | WS_VISIBLE | CBRS_TOP
		| CBRS_GRIPPER | CBRS_TOOLTIPS | CBRS_FLYBY | CBRS_SIZE_DYNAMIC,CRect(0,   0,   0,   0),ID_VIEW_DISPLAYBAR) ||
		!m_wndDisplayToolBar.LoadToolBar(IDR_DISPLAY_TOOLBAR))
	{
		TRACE0("Failed to create toolbar\n");
		return -1;      // fail to create
	}



	if (!m_wndControlToolBar.CreateEx(this, TBSTYLE_FLAT, WS_CHILD | WS_VISIBLE | CBRS_TOP
		| CBRS_GRIPPER | CBRS_TOOLTIPS | CBRS_FLYBY | CBRS_SIZE_DYNAMIC) ||
		!m_wndControlToolBar.LoadToolBar(IDR_CONTROL_TOOLBAR))
	{
		TRACE0("Failed to create toolbar\n");
		return -1;      // fail to create
	}
	m_wndToolBar.EnableDocking(CBRS_ALIGN_ANY);
	m_wndDisplayToolBar.EnableDocking(CBRS_ALIGN_ANY);
	m_wndControlToolBar.EnableDocking(CBRS_ALIGN_ANY);
	EnableDocking(CBRS_ALIGN_ANY);
	DockControlBar(&m_wndDisplayToolBar);
	DockControlBarLeftOf(&m_wndToolBar,&m_wndDisplayToolBar);
	DockControlBarLeftOf(&m_wndControlToolBar,&m_wndDisplayToolBar);


	m_stBarButtons.bOpenCompass=1;
	m_stBarButtons.bOpenPIP=1;
	m_stBarButtons.bOpenPath=1;
	m_stBarButtons.bOpenStatusLight=1;
	m_stBarButtons.bOpenTriangulation=1;

	m_stBarButtons.bOpenIMU=1;
	m_stBarButtons.bOpenSLAMSetting=1;


	m_stBarButtons.bOpenManualMB=0;
	m_wndControlToolBar.GetToolBarCtrl().HideButton(ID_BUTTON_MANUAL_MB);

	m_stBarButtons.bOpenMessageBox=1;
	m_stBarButtons.bOpenOptionButtons=1;

	

//	m_wndDisplayToolBar.SetButtonStyle(0,TBBS_CHECKBOX);
//	m_wndDisplayToolBar.SetButtonStyle(1,TBBS_CHECKBOX);
//	m_wndDisplayToolBar.SetButtonStyle(2,TBBS_CHECKBOX);
//	m_wndDisplayToolBar.SetButtonStyle(3,TBBS_CHECKBOX);

	if (!m_wndStatusBar.Create(this) ||
		!m_wndStatusBar.SetIndicators(indicators,
		  sizeof(indicators)/sizeof(UINT)))
	{
		TRACE0("Failed to create status bar\n");
		return -1;      // fail to create
	}

	// TODO: Delete these three lines if you don't want the toolbar to be dockable




	if (!m_wndOptionBar.Create(_T("OptionBar"), this, CSize(185,100),TRUE,123))
	{
		TRACE0("Failed to create mybar\n");
		return -1;
	}
	if (!m_wndMessageBar.Create(_T("MessageBar"), this,CSize(185,100),TRUE,124))
	{
		TRACE0("Failed to create mybar\n");
		return -1;
	}


	m_wndOptionBar.SetBarStyle(m_wndOptionBar.GetBarStyle() |
		CBRS_TOOLTIPS | CBRS_FLYBY | CBRS_SIZE_DYNAMIC |CBRS_BORDER_ANY );

	m_wndMessageBar.SetBarStyle(m_wndMessageBar.GetBarStyle() |
		CBRS_TOOLTIPS | CBRS_FLYBY | CBRS_SIZE_DYNAMIC |CBRS_BORDER_ANY);



	m_wndOptionBar.EnableDocking(CBRS_ALIGN_ANY);
	m_wndMessageBar.EnableDocking(CBRS_ALIGN_ANY);

	
/*	RECT rect;
	rect.left=0;
	rect.top=0;
	rect.right=0;
	rect.bottom=0;
	m_ButtonIDrive.Create("IDrive",WS_CHILD|WS_VISIBLE,rect,&m_wndOptionBar,567);*/

/*	rect.left=15;
	rect.top=0;
	rect.right=10;
	rect.bottom=5;
	m_ButtonAutoDrive.Create("IDrive",WS_CHILD|WS_VISIBLE,rect,&m_wndOptionBar,1000);*/


	int nRtn=m_DlgOptions.Create(IDD_DLGOPTIONS,&m_wndOptionBar);
	m_DlgOptions.ShowWindow(SW_SHOW);




	if(!m_wndEdit.Create(WS_VSCROLL|WS_CHILD|WS_VISIBLE|ES_AUTOVSCROLL|ES_MULTILINE|ES_WANTRETURN,
		CRect(0,0,0,0),&m_wndMessageBar,101))
		return -1;
	m_wndEdit.ModifyStyleEx(0,WS_EX_CLIENTEDGE);


	CRect rectMainFrm;
	CRect rectMessageBar;

	this->DockControlBar(&m_wndMessageBar, AFX_IDW_DOCKBAR_RIGHT);///Í£¿¿ÔÚÓÒ±ß
	this->RecalcLayout();

	this->GetWindowRect(rectMainFrm);
	m_wndMessageBar.GetWindowRect(rectMessageBar);
	rectMessageBar.OffsetRect(0, 1);//
	this->DockControlBar(&m_wndOptionBar, AFX_IDW_DOCKBAR_RIGHT,rectMessageBar);///Ò²Í£¿¿ÔÚÓÒ±ß




/*	if (!m_wndDisplayBar.Create(_T("DisplayBar"), this,CSize(rectMessageBar.left-rectMainFrm.left,600),TRUE,125))
	{
		TRACE0("Failed to create mybar\n");
		return -1;
	}

	m_wndDisplayBar.SetBarStyle(m_wndDisplayBar.GetBarStyle() |
		CBRS_TOOLTIPS | CBRS_FLYBY | CBRS_SIZE_DYNAMIC |CBRS_BORDER_ANY);
	m_wndDisplayBar.EnableDocking(CBRS_ALIGN_ANY);

	this->DockControlBar(&m_wndDisplayBar,AFX_IDW_DOCKBAR_LEFT);*/

	//m_bOpenManualMB=false;
	//m_wndControlToolBar.GetToolBarCtrl().EnableButton(ID_BUTTON_IMU,FALSE);
	//m_wndDisplayToolBar.GetToolBarCtrl().EnableButton(ID_BUTTON_COMPASS,FALSE);

	return 0;
}

BOOL CMainFrame::PreCreateWindow(CREATESTRUCT& cs)
{
	if( !CFrameWnd::PreCreateWindow(cs) )
		return FALSE;
	// TODO: Modify the Window class or styles here by modifying
	//  the CREATESTRUCT cs

	cs.x=50;
	cs.y=50;
	cs.cx=840;
	cs.cy=590;



	return TRUE;
}


// CMainFrame diagnostics

#ifdef _DEBUG
void CMainFrame::AssertValid() const
{
	CFrameWnd::AssertValid();
}

void CMainFrame::Dump(CDumpContext& dc) const
{
	CFrameWnd::Dump(dc);
}

#endif //_DEBUG


// CMainFrame message handlers




void CMainFrame::OnLButtonDown(UINT nFlags, CPoint point)
{
	// TODO: Add your message handler code here and/or call default

	int ff=10;
	CFrameWnd::OnLButtonDown(nFlags, point);
}


/*
void CMainFrame::OnUpdateViewDisplaybar(CCmdUI *pCmdUI)
{
	pCmdUI->Enable();
	pCmdUI->SetCheck(m_wndDisplayToolBar.IsVisible());
	// TODO: Add your command update UI handler code here
}

void CMainFrame::OnViewDisplaybar()
{
	ShowControlBar(&m_wndDisplayToolBar, !m_wndDisplayToolBar.IsVisible(), FALSE);
	// TODO: Add your command handler code here
}

void CMainFrame::OnViewControlbar()
{
	ShowControlBar(&m_wndControlToolBar, !m_wndControlToolBar.IsVisible(), FALSE);
	// TODO: Add your command handler code here
}

void CMainFrame::OnUpdateViewControlbar(CCmdUI *pCmdUI)
{
	pCmdUI->Enable();
	pCmdUI->SetCheck(m_wndControlToolBar.IsVisible());
	// TODO: Add your command update UI handler code here
}
*/
/*
void CMainFrame::OnMessageBar()
{
	ShowControlBar(&m_wndMessageBar, !m_wndMessageBar.IsVisible(), FALSE);
}

void CMainFrame::OnControlBar()
{
	ShowControlBar(&m_wndOptionBar, !m_wndOptionBar.IsVisible(), FALSE);
}


void CMainFrame::OnUpdateDisplaybarImu(CCmdUI *pCmdUI)
{
	pCmdUI->Enable();
	pCmdUI->SetCheck(1);
	//m_wndDisplayToolBar.GetToolBarCtrl().SetState(ID_BUTTON_IMU,TBSTATE_ENABLED);
	// TODO: Add your command update UI handler code here
}

void CMainFrame::OnDisplaybarImu()
{
	
	//m_wndDisplayToolBar.GetToolBarCtrl().SetState(ID_BUTTON_IMU,TBSTATE_INDETERMINATE);
	// TODO: Add your command handler code here
}

void CMainFrame::OnDisplaybarManualmap()
{
	m_bOpenManualMB=(m_bOpenManualMB?false:true);
	if (m_bOpenManualMB)m_wndControlToolBar.GetToolBarCtrl().SetState(ID_BUTTON_MANUAL_MB,TBSTATE_ENABLED);
	else
	{
		m_wndControlToolBar.GetToolBarCtrl().HideButton(ID_BUTTON_MANUAL_MB);
	}
	//m_wndControlToolBar.GetToolBarCtrl().SetState(ID_BUTTON_MANUAL_MB,TBSTATE_INDETERMINATE);
	// TODO: Add your command handler code here
}

void CMainFrame::OnUpdateDisplaybarManualmap(CCmdUI *pCmdUI)
{
	pCmdUI->Enable();
	pCmdUI->SetCheck(m_bOpenManualMB);
	// TODO: Add your command update UI handler code here
}
*/
void CMainFrame::OnControlbarImu()
{
	// TODO: Add your command handler code here
	m_stBarButtons.bOpenIMU=(m_stBarButtons.bOpenIMU?false:true);
	if (m_stBarButtons.bOpenIMU)m_wndControlToolBar.GetToolBarCtrl().SetState(ID_BUTTON_IMU,TBSTATE_ENABLED);
	else
	{
		m_wndControlToolBar.GetToolBarCtrl().HideButton(ID_BUTTON_IMU);
	}
	UpdateBarView();
}

void CMainFrame::OnUpdateControlbarImu(CCmdUI *pCmdUI)
{
	pCmdUI->Enable();
	pCmdUI->SetCheck(m_stBarButtons.bOpenIMU);
	// TODO: Add your command update UI handler code here
}

void CMainFrame::OnControlbarManualmapbuilder()
{
	m_stBarButtons.bOpenManualMB=(m_stBarButtons.bOpenManualMB?false:true);
	CIoTRobot_GUIApp *pAPP= (CIoTRobot_GUIApp*)::AfxGetApp();

	IoTRobot_Message MSG;
	MSG.cCommand=SLAM_MSG_REBUILD_MAP;
	MSG.cDestModule=SLAM_MODULE;
	MSG.cFromModule=GUI_MODULE;


	
	if (m_stBarButtons.bOpenManualMB)
	{
		MSG.nParam1=2;
		MSG.nParam2=0;
		m_wndControlToolBar.GetToolBarCtrl().SetState(ID_BUTTON_MANUAL_MB,TBSTATE_ENABLED);
	}
	else
	{
		MSG.nParam1=1;
		MSG.nParam2=0;
		m_wndControlToolBar.GetToolBarCtrl().HideButton(ID_BUTTON_MANUAL_MB);
	}
	pAPP->m_pCSendCMD2Kernel(&MSG);
	UpdateBarView();
	// TODO: Add your command handler code here
}

void CMainFrame::OnUpdateControlbarManualmapbuilder(CCmdUI *pCmdUI)
{
	pCmdUI->Enable();
	pCmdUI->SetCheck(m_stBarButtons.bOpenManualMB);
	// TODO: Add your command update UI handler code here
}

void CMainFrame::OnControlbarSlamsettings()
{
	m_stBarButtons.bOpenSLAMSetting=(m_stBarButtons.bOpenSLAMSetting?false:true);
	if (m_stBarButtons.bOpenSLAMSetting)m_wndControlToolBar.GetToolBarCtrl().SetState(ID_BUTTON_SLAMSETTING,TBSTATE_ENABLED);
	else
	{
		m_wndControlToolBar.GetToolBarCtrl().HideButton(ID_BUTTON_SLAMSETTING);
	}
	UpdateBarView();

	// TODO: Add your command handler code here
}

void CMainFrame::OnUpdateControlbarSlamsettings(CCmdUI *pCmdUI)
{
	pCmdUI->Enable();
	pCmdUI->SetCheck(m_stBarButtons.bOpenSLAMSetting);
	// TODO: Add your command update UI handler code here
}

void CMainFrame::OnDisplaybarPath()
{
	m_stBarButtons.bOpenPath=(m_stBarButtons.bOpenPath?false:true);
	if (m_stBarButtons.bOpenPath)m_wndDisplayToolBar.GetToolBarCtrl().SetState(ID_BUTTON_PATH,TBSTATE_ENABLED);
	else
	{
		m_wndDisplayToolBar.GetToolBarCtrl().HideButton(ID_BUTTON_PATH);
	}
	UpdateBarView();
	// TODO: Add your command handler code here
}

void CMainFrame::OnUpdateDisplaybarPath(CCmdUI *pCmdUI)
{
	pCmdUI->Enable();
	pCmdUI->SetCheck(m_stBarButtons.bOpenPath);
	// TODO: Add your command update UI handler code here
}



void CMainFrame::OnDisplaybarPipButton()
{
	m_stBarButtons.bOpenPIP=(m_stBarButtons.bOpenPIP?false:true);
	if (m_stBarButtons.bOpenPIP)m_wndDisplayToolBar.GetToolBarCtrl().SetState(ID_BUTTON_PC_VIEW,TBSTATE_ENABLED);
	else
	{
		m_wndDisplayToolBar.GetToolBarCtrl().HideButton(ID_BUTTON_PC_VIEW);
	}
	UpdateBarView();
	// TODO: Add your command handler code here
}

void CMainFrame::OnUpdateDisplaybarPipButton(CCmdUI *pCmdUI)
{
	pCmdUI->Enable();
	pCmdUI->SetCheck(m_stBarButtons.bOpenPIP);
	// TODO: Add your command update UI handler code here
}

void CMainFrame::OnDisplaybarStatuslight()
{
	m_stBarButtons.bOpenStatusLight=(m_stBarButtons.bOpenStatusLight?false:true);
	if (m_stBarButtons.bOpenStatusLight)m_wndDisplayToolBar.GetToolBarCtrl().SetState(ID_BUTTON_STATE_LIGHT,TBSTATE_ENABLED);
	else
	{
		m_wndDisplayToolBar.GetToolBarCtrl().HideButton(ID_BUTTON_STATE_LIGHT);
	}
	UpdateBarView();
	// TODO: Add your command handler code here
}

void CMainFrame::OnUpdateDisplaybarStatuslight(CCmdUI *pCmdUI)
{
	pCmdUI->Enable();
	pCmdUI->SetCheck(m_stBarButtons.bOpenStatusLight);
	// TODO: Add your command update UI handler code here
}

void CMainFrame::OnToolbarMessagebox()
{
	m_stBarButtons.bOpenMessageBox=(m_stBarButtons.bOpenMessageBox?false:true);
	if (m_stBarButtons.bOpenMessageBox)m_wndToolBar.GetToolBarCtrl().SetState(ID_BUTTON_MESSAGE_LIST,TBSTATE_ENABLED);
	else
	{
		m_wndToolBar.GetToolBarCtrl().HideButton(ID_BUTTON_MESSAGE_LIST);
	}
	UpdateBarView();
	// TODO: Add your command handler code here
}

void CMainFrame::OnUpdateToolbarMessagebox(CCmdUI *pCmdUI)
{
	pCmdUI->Enable();
	pCmdUI->SetCheck(m_stBarButtons.bOpenMessageBox);
	// TODO: Add your command update UI handler code here
}

void CMainFrame::OnToolbarOptionbuttons()
{
	m_stBarButtons.bOpenOptionButtons=(m_stBarButtons.bOpenOptionButtons?false:true);
	if (m_stBarButtons.bOpenOptionButtons)m_wndToolBar.GetToolBarCtrl().SetState(ID_BUTTON_CONTROL_BUTTON,TBSTATE_ENABLED);
	else
	{
		m_wndToolBar.GetToolBarCtrl().HideButton(ID_BUTTON_CONTROL_BUTTON);
	}
	UpdateBarView();
	// TODO: Add your command handler code here
}

void CMainFrame::OnUpdateToolbarOptionbuttons(CCmdUI *pCmdUI)
{
	pCmdUI->Enable();
	pCmdUI->SetCheck(m_stBarButtons.bOpenOptionButtons);
	// TODO: Add your command update UI handler code here
}

void CMainFrame::OnDisplaybarCompass()
{
	m_stBarButtons.bOpenCompass=(m_stBarButtons.bOpenCompass?false:true);
	if (m_stBarButtons.bOpenCompass)m_wndDisplayToolBar.GetToolBarCtrl().SetState(ID_BUTTON_COMPASS,TBSTATE_ENABLED);
	else
	{
		m_wndDisplayToolBar.GetToolBarCtrl().HideButton(ID_BUTTON_COMPASS);
	}
	UpdateBarView();
	// TODO: Add your command handler code here
}

void CMainFrame::OnUpdateDisplaybarCompass(CCmdUI *pCmdUI)
{
	if (m_bIMUApply)
	{
		pCmdUI->Enable();
		pCmdUI->SetCheck(m_stBarButtons.bOpenCompass);
	}
	else
	{
		pCmdUI->Enable(FALSE);
	}

	// TODO: Add your command update UI handler code here
}


int CMainFrame::UpdateBarView()
{
	if (!m_stBarButtons.bOpenCompass
		&&!m_stBarButtons.bOpenPath
		&&!m_stBarButtons.bOpenPIP
		&&!m_stBarButtons.bOpenStatusLight)
	{
		ShowControlBar(&m_wndDisplayToolBar, FALSE, FALSE);
	}
	else
	{
		ShowControlBar(&m_wndDisplayToolBar, TRUE, FALSE);
	}

	if (!m_stBarButtons.bOpenIMU
		&&!m_stBarButtons.bOpenManualMB
		&&!m_stBarButtons.bOpenSLAMSetting)
	{
		ShowControlBar(&m_wndControlToolBar, FALSE, FALSE);
	}
	else
	{
		ShowControlBar(&m_wndControlToolBar, TRUE, FALSE);
	}

	if (!m_stBarButtons.bOpenMessageBox
		&&!m_stBarButtons.bOpenOptionButtons)
	{
		ShowControlBar(&m_wndToolBar, FALSE, FALSE);
	}
	else
	{
		ShowControlBar(&m_wndToolBar, TRUE, FALSE);
	}
	DockControlBar(&m_wndDisplayToolBar);
	DockControlBarLeftOf(&m_wndToolBar,&m_wndDisplayToolBar);
	DockControlBarLeftOf(&m_wndControlToolBar,&m_wndDisplayToolBar);
	return 0;
}
void CMainFrame::OnDisplaybarTriangulation()
{
	m_stBarButtons.bOpenTriangulation=(m_stBarButtons.bOpenTriangulation?false:true);
	if (m_stBarButtons.bOpenTriangulation)m_wndDisplayToolBar.GetToolBarCtrl().SetState(ID_BUTTON_TRIANGLATION,TBSTATE_ENABLED);
	else
	{
		m_wndDisplayToolBar.GetToolBarCtrl().HideButton(ID_BUTTON_TRIANGLATION);
	}
	UpdateBarView();
	// TODO: Add your command handler code here
}

void CMainFrame::OnUpdateDisplaybarTriangulation(CCmdUI *pCmdUI)
{
	pCmdUI->Enable();
	pCmdUI->SetCheck(m_stBarButtons.bOpenTriangulation);
	// TODO: Add your command update UI handler code here
}

void CMainFrame::OnClose()
{
	//AFX_MANAGE_STATE(AfxGetStaticModuleState()); 
	// Note: only queries the active document
	CDocument* pDocument = GetActiveDocument();
	if (pDocument != NULL && !pDocument->CanCloseFrame(this))
	{
		// document can't close right now -- don't close it
		return;
	}
	CWinApp* pApp = AfxGetApp();
	if (pApp != NULL && pApp->m_pMainWnd == this)
	{
		// attempt to save all documents
		if (pDocument == NULL && !pApp->SaveAllModified())
			return;     // don't close it

		// hide the application's windows before closing all the documents
		pApp->HideApplication();

		// close all documents first
		

		// don't exit if there are outstanding component objects
		if (!AfxOleCanExitApp())
		{
			// take user out of control of the app
			AfxOleSetUserCtrl(FALSE);

			// don't destroy the main window and close down just yet
			//  (there are outstanding component (OLE) objects)
			return;
		}

		// there are cases where destroying the documents may destroy the
		//  main window of the application.
		//if (!afxContextIsDLL && pApp->m_pMainWnd == NULL)
		{
			AfxPostQuitMessage(0);
			return;
		}
	}
}

BOOL CMainFrame::DestroyWindow()
{
	// TODO: Add your specialized code here and/or call the base class

	CWnd* pWnd;
	CHandleMap* pMap;
	HWND hWndOrig;
	BOOL bResult;

	if ((m_hWnd == NULL) && (m_pCtrlSite == NULL))
		return FALSE;

	bResult = FALSE;
	pMap = NULL;
	pWnd = NULL;
	hWndOrig = NULL;
/*	if (m_hWnd != NULL)
	{
		pMap = afxMapHWND();
		ENSURE(pMap != NULL);
		pWnd = (CWnd*)pMap->LookupPermanent(m_hWnd);
#ifdef _DEBUG
		hWndOrig = m_hWnd;
#endif
	}*/

#ifdef _AFX_NO_OCC_SUPPORT
	if (m_hWnd != NULL)
		bResult = ::DestroyWindow(m_hWnd);
#else //_AFX_NO_OCC_SUPPORT
	if ((m_hWnd != NULL) || (m_pCtrlSite != NULL))
	{
		if (m_pCtrlSite == NULL)
			bResult = ::DestroyWindow(m_hWnd);
		else
			bResult = m_pCtrlSite->DestroyControl();
	}
#endif //_AFX_NO_OCC_SUPPORT

	if (hWndOrig != NULL)
	{
		// Note that 'this' may have been deleted at this point,
		//  (but only if pWnd != NULL)
		if (pWnd != NULL)
		{
			// Should have been detached by OnNcDestroy
#ifdef _DEBUG
//			ASSERT(pMap->LookupPermanent(hWndOrig) == NULL);
#endif
		}
		else
		{
#ifdef _DEBUG
			ASSERT(m_hWnd == hWndOrig);
#endif
			// Detach after DestroyWindow called just in case
			Detach();
		}
	}
	//return CFrameWnd::DestroyWindow();
	return true;
}
