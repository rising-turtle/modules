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

protected:  // control bar embedded members
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

	//DlgBar    m_DlgBar;

// Generated message map functions
protected:
	afx_msg int OnCreate(LPCREATESTRUCT lpCreateStruct);
	DECLARE_MESSAGE_MAP()
public:
	afx_msg void OnLButtonDown(UINT nFlags, CPoint point);

	void   CMainFrame::DockControlBarLeftOf(CControlBar*   Bar,   CControlBar*   LeftOf) ;
	afx_msg void OnUpdateViewDisplaybar(CCmdUI *pCmdUI);
	afx_msg void OnViewDisplaybar();
	afx_msg void OnViewControlbar();
	afx_msg void OnUpdateViewControlbar(CCmdUI *pCmdUI);

	afx_msg void OnMessageBar();
	afx_msg void OnControlBar();
};


