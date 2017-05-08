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
	ON_UPDATE_COMMAND_UI(ID_VIEW_DISPLAYBAR, &CMainFrame::OnUpdateViewDisplaybar)
	ON_COMMAND(ID_VIEW_DISPLAYBAR, &CMainFrame::OnViewDisplaybar)
	ON_COMMAND(ID_VIEW_CONTROLBAR, &CMainFrame::OnViewControlbar)
	ON_UPDATE_COMMAND_UI(ID_VIEW_CONTROLBAR, &CMainFrame::OnUpdateViewControlbar)

	ON_COMMAND(ID_BUTTON_MESSAGE_LIST,&CMainFrame::OnMessageBar)
	ON_COMMAND(ID_BUTTON_CONTROL_BUTTON,&CMainFrame::OnControlBar)
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
/*	if (!m_wndToolBar.CreateEx(this, TBSTYLE_FLAT, WS_CHILD | WS_VISIBLE | CBRS_TOP
		| CBRS_GRIPPER | CBRS_TOOLTIPS | CBRS_FLYBY | CBRS_SIZE_DYNAMIC) ||
		!m_wndToolBar.LoadToolBar(IDR_MAINFRAME))
	{
		TRACE0("Failed to create toolbar\n");
		return -1;      // fail to create
	}*/




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
//	m_wndToolBar.EnableDocking(CBRS_ALIGN_ANY);
	m_wndDisplayToolBar.EnableDocking(CBRS_ALIGN_ANY);
	m_wndControlToolBar.EnableDocking(CBRS_ALIGN_ANY);
	EnableDocking(CBRS_ALIGN_ANY);
	DockControlBar(&m_wndDisplayToolBar);
//	DockControlBarLeftOf(&m_wndToolBar,&m_wndDisplayToolBar);
	DockControlBarLeftOf(&m_wndControlToolBar,&m_wndDisplayToolBar);
	

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


	m_DlgOptions.Create(IDD_DLGOPTIONS,&m_wndOptionBar);

	m_DlgOptions.UpdateWindow();
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


void CMainFrame::OnMessageBar()
{
	ShowControlBar(&m_wndMessageBar, !m_wndMessageBar.IsVisible(), FALSE);
}

void CMainFrame::OnControlBar()
{
	ShowControlBar(&m_wndOptionBar, !m_wndOptionBar.IsVisible(), FALSE);
}