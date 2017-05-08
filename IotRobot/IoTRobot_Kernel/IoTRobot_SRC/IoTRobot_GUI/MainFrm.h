// MainFrm.h : interface of the CMainFrame class
//


#pragma once
#include "scbarg.h"
#include "CoolTabCtrl.h"
#include "DlgOptions.h"

class CMainFrame : public CFrameWnd
{
	
protected: // create from serialization only
	CMainFrame();
	DECLARE_DYNCREATE(CMainFrame)

// Attributes
public:

// Operations
public:

// Overrides
public:
	virtual BOOL PreCreateWindow(CREATESTRUCT& cs);

// Implementation
public:
	virtual ~CMainFrame();
#ifdef _DEBUG
	virtual void AssertValid() const;
	virtual void Dump(CDumpContext& dc) const;
#endif

public:  // control bar embedded members
	CStatusBar  m_wndStatusBar;
	CToolBar    m_wndToolBar;

	CStatic m_staticImg;

	CCoolBar  m_wndOptionBar;
	CCoolBar  m_wndMessageBar;
	CCoolBar  m_wndDisplayBar;

	CCoolTabCtrl m_wndTabCtrl;
	CDlgOptions m_DlgOptions;
	CTabCtrl m_wndTab;
	CTreeCtrl m_wndTree;
	CEdit	  m_wndEdit;


	CToolBar    m_wndDisplayToolBar;
	CToolBar    m_wndControlToolBar;

	CButton m_ButtonIDrive;
	CButton m_ButtonAutoDrive;


	IoTRobot_BarButtons m_stBarButtons;

	int UpdateBarView();

	//DlgBar    m_DlgBar;

// Generated message map functions
protected:
	afx_msg int OnCreate(LPCREATESTRUCT lpCreateStruct);
	DECLARE_MESSAGE_MAP()
public:
	afx_msg void OnLButtonDown(UINT nFlags, CPoint point);

	void   CMainFrame::DockControlBarLeftOf(CControlBar*   Bar,   CControlBar*   LeftOf) ;
/*	afx_msg void OnUpdateViewDisplaybar(CCmdUI *pCmdUI);
	afx_msg void OnViewDisplaybar();
	afx_msg void OnViewControlbar();
	afx_msg void OnUpdateViewControlbar(CCmdUI *pCmdUI);

	afx_msg void OnMessageBar();
	afx_msg void OnControlBar();
	afx_msg void OnUpdateDisplaybarImu(CCmdUI *pCmdUI);
	afx_msg void OnDisplaybarImu();
	afx_msg void OnDisplaybarManualmap();
	afx_msg void OnUpdateDisplaybarManualmap(CCmdUI *pCmdUI);*/
	afx_msg void OnControlbarImu();
	afx_msg void OnUpdateControlbarImu(CCmdUI *pCmdUI);
	afx_msg void OnControlbarManualmapbuilder();
	afx_msg void OnUpdateControlbarManualmapbuilder(CCmdUI *pCmdUI);
	afx_msg void OnControlbarSlamsettings();
	afx_msg void OnUpdateControlbarSlamsettings(CCmdUI *pCmdUI);
	afx_msg void OnDisplaybarPath();
	afx_msg void OnUpdateDisplaybarPath(CCmdUI *pCmdUI);
	afx_msg void OnDisplaybarPipButton();
	afx_msg void OnUpdateDisplaybarPipButton(CCmdUI *pCmdUI);
	afx_msg void OnDisplaybarStatuslight();
	afx_msg void OnUpdateDisplaybarStatuslight(CCmdUI *pCmdUI);
	afx_msg void OnToolbarMessagebox();
	afx_msg void OnUpdateToolbarMessagebox(CCmdUI *pCmdUI);
	afx_msg void OnToolbarOptionbuttons();
	afx_msg void OnUpdateToolbarOptionbuttons(CCmdUI *pCmdUI);
	afx_msg void OnDisplaybarCompass();
	afx_msg void OnUpdateDisplaybarCompass(CCmdUI *pCmdUI);
	afx_msg void OnDisplaybarTriangulation();
	afx_msg void OnUpdateDisplaybarTriangulation(CCmdUI *pCmdUI);
	afx_msg void OnClose();


	int m_bIMUApply;
	virtual BOOL DestroyWindow();
};


