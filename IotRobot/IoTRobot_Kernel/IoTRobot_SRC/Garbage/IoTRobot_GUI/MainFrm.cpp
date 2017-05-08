// MainFrm.cpp : implementation of the CMainFrame class
//

#include "stdafx.h"
#include "IoTRobot_GUI.h"

#include "MainFrm.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#endif


// CMainFrame

IMPLEMENT_DYNAMIC(CMainFrame, CMDIFrameWnd)

BEGIN_MESSAGE_MAP(CMainFrame, CMDIFrameWnd)
	ON_WM_CREATE()
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


int CMainFrame::OnCreate(LPCREATESTRUCT lpCreateStruct)
{
	if (CMDIFrameWnd::OnCreate(lpCreateStruct) == -1)
		return -1;
	
	if (!m_wndToolBar.CreateEx(this, TBSTYLE_FLAT, WS_CHILD | WS_VISIBLE | CBRS_TOP
		| CBRS_GRIPPER | CBRS_TOOLTIPS | CBRS_FLYBY | CBRS_SIZE_DYNAMIC) ||
		!m_wndToolBar.LoadToolBar(IDR_MAINFRAME))
	{
		TRACE0("Failed to create toolbar\n");
		return -1;      // fail to create
	}

	if (!m_wndStatusBar.Create(this) ||
		!m_wndStatusBar.SetIndicators(indicators,
		  sizeof(indicators)/sizeof(UINT)))
	{
		TRACE0("Failed to create status bar\n");
		return -1;      // fail to create
	}

	// TODO: Delete these three lines if you don't want the toolbar to be dockable
	m_wndToolBar.EnableDocking(CBRS_ALIGN_ANY);
	EnableDocking(CBRS_ALIGN_ANY);
	DockControlBar(&m_wndToolBar);


/*	if (!m_wndOptionBar.Create(_T("OptionBar"), this, CSize(240,100),TRUE,123))
	{
		TRACE0("Failed to create mybar\n");
		return -1;
	}
	if (!m_wndMessageBar.Create(_T("MessageBar"), this,CSize(240,100),TRUE,124))
	{
		TRACE0("Failed to create mybar\n");
		return -1;
	}
	if (!m_wndDisplayBar.Create(_T("DisplayBar"), this,CSize(10000,600),TRUE,125))
	{
		TRACE0("Failed to create mybar\n");
		return -1;
	}

	m_wndOptionBar.SetBarStyle(m_wndOptionBar.GetBarStyle() |
		CBRS_TOOLTIPS | CBRS_FLYBY | CBRS_SIZE_DYNAMIC |CBRS_BORDER_ANY );

	m_wndMessageBar.SetBarStyle(m_wndMessageBar.GetBarStyle() |
		CBRS_TOOLTIPS | CBRS_FLYBY | CBRS_SIZE_DYNAMIC |CBRS_BORDER_ANY);

	m_wndDisplayBar.SetBarStyle(m_wndDisplayBar.GetBarStyle() |
		CBRS_TOOLTIPS | CBRS_FLYBY | CBRS_SIZE_DYNAMIC |CBRS_BORDER_ANY);

	m_wndOptionBar.EnableDocking(CBRS_ALIGN_ANY);
	m_wndMessageBar.EnableDocking(CBRS_ALIGN_ANY);
	m_wndDisplayBar.EnableDocking(CBRS_ALIGN_ANY);




	CRect rc(0, 0, 0, 0);
	CSize sizeMax(0, 0);
	CToolBarCtrl& bar = m_wndToolBar.GetToolBarCtrl();
	for (int nIndex = bar.GetButtonCount() - 1; nIndex >= 0; nIndex--)
	{
		bar.GetItemRect(nIndex, rc);

		rc.NormalizeRect();
		sizeMax.cx = __max(rc.Size().cx, sizeMax.cx);
		sizeMax.cy = __max(rc.Size().cy, sizeMax.cy);
	}
	//sizeMax.cx += 10;
	m_wndToolBar.SetSizes(sizeMax, CSize(16,15));


	m_DlgOptions.Create(IDD_DLG_OPTIONS,&m_wndOptionBar);

	m_DlgOptions.UpdateWindow();
	if(!m_wndEdit.Create(WS_VSCROLL|WS_CHILD|WS_VISIBLE|ES_AUTOVSCROLL|ES_MULTILINE|ES_WANTRETURN,CRect(0,0,0,0),&m_wndMessageBar,101))
		return -1;
	m_wndEdit.ModifyStyleEx(0,WS_EX_CLIENTEDGE);

	DockControlBar(&m_wndMessageBar, AFX_IDW_DOCKBAR_RIGHT);///停靠在右边
	RecalcLayout();
	CRect rect;
	m_wndMessageBar.GetWindowRect(rect);
	rect.OffsetRect(0, 1);//
	DockControlBar(&m_wndOptionBar, AFX_IDW_DOCKBAR_RIGHT,rect);///也停靠在右边
	DockControlBar(&m_wndDisplayBar,AFX_IDW_DOCKBAR_LEFT);
	/////////使浮动与停靠显示相同的头部////
#ifdef _SCB_REPLACE_MINIFRAME
	m_pFloatingFrameClass = RUNTIME_CLASS(CSCBMiniDockFrameWnd);
#endif //_SCB_REPLACE_MINIFRAME*/

	return 0;
}

BOOL CMainFrame::PreCreateWindow(CREATESTRUCT& cs)
{
	if( !CMDIFrameWnd::PreCreateWindow(cs) )
		return FALSE;
	// TODO: Modify the Window class or styles here by modifying
	//  the CREATESTRUCT cs
	cs.x=50;
	cs.y=50;
	cs.cx=800;
	cs.cy=60;
	

	return TRUE;
}


// CMainFrame diagnostics

#ifdef _DEBUG
void CMainFrame::AssertValid() const
{
	CMDIFrameWnd::AssertValid();
}

void CMainFrame::Dump(CDumpContext& dc) const
{
	CMDIFrameWnd::Dump(dc);
}

#endif //_DEBUG


// CMainFrame message handlers



