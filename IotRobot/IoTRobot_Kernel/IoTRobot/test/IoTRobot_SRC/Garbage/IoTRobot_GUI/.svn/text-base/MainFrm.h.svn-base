// MainFrm.h : interface of the CMainFrame class
//


#pragma once

#include "scbarg.h"
#include "CoolTabCtrl.h"
#include "DlgOptions.h"
#include "DlgBar.h"
class CMainFrame : public CMDIFrameWnd
{
	DECLARE_DYNAMIC(CMainFrame)
public:
	CMainFrame();

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

// Generated message map functions
protected:
	afx_msg int OnCreate(LPCREATESTRUCT lpCreateStruct);
	DECLARE_MESSAGE_MAP()
public:
	CCoolBar  m_wndOptionBar;
	CCoolBar  m_wndMessageBar;
	CCoolBar  m_wndDisplayBar;

	CCoolTabCtrl m_wndTabCtrl;
	CDlgOptions m_DlgOptions;
	CTabCtrl m_wndTab;
	CTreeCtrl m_wndTree;
	CEdit	  m_wndEdit;
	DlgBar    m_DlgBar;

};


