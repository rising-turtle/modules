// IoTRobot_GUIView.h : interface of the CIoTRobot_GUIView class
//


#pragma once
#include "OpenGL.h"
#include "DlgRobotView.h"
#include "DlgEditRouteConfirm.h"
class CIoTRobot_GUIView : public CView
{
protected: // create from serialization only
	CIoTRobot_GUIView();
	DECLARE_DYNCREATE(CIoTRobot_GUIView)
// Attributes
public:
	CIoTRobot_GUIDoc* GetDocument() const;
// Operations
public:
// Overrides
public:
	virtual void OnDraw(CDC* pDC);  // overridden to draw this view
	virtual BOOL PreCreateWindow(CREATESTRUCT& cs);

	bool m_bStopThreadRender;
	HANDLE m_hThreadRenderRun;
	static UINT ThreadRender(LPVOID lpParam);
protected:
	virtual BOOL OnPreparePrinting(CPrintInfo* pInfo);
	virtual void OnBeginPrinting(CDC* pDC, CPrintInfo* pInfo);
	virtual void OnEndPrinting(CDC* pDC, CPrintInfo* pInfo);

// Implementation
public:
	virtual ~CIoTRobot_GUIView();
#ifdef _DEBUG
	virtual void AssertValid() const;
	virtual void Dump(CDumpContext& dc) const;
#endif

protected:
// Generated message map functions
protected:
	DECLARE_MESSAGE_MAP()
public:
	afx_msg int OnCreate(LPCREATESTRUCT lpCreateStruct);
	afx_msg void OnSize(UINT nType, int cx, int cy);
	afx_msg BOOL OnEraseBkgnd(CDC* pDC);
	afx_msg void OnDestroy();

public:
	afx_msg void OnPaint();
	CStatic m_staticImg;
	OpenGL m_cOpenGL;
	afx_msg void OnKeyDown(UINT nChar, UINT nRepCnt, UINT nFlags);
	afx_msg BOOL OnMouseWheel(UINT nFlags, short zDelta, CPoint pt);

	
	afx_msg void OnDisplayPC();
	afx_msg void OnDisplayPC_View();
	afx_msg void OnDisplayStateLight();
	afx_msg void OnDisplayPath();
	afx_msg void OnDisplayTwinklingPath();
	afx_msg void OnTriangulation();
	afx_msg void OnManualPath();

	afx_msg void OnCompass();




	//show robot view
	static UINT ThreadRobotView(LPVOID lpParam);
	static UINT ThreadWaitForMileStone(LPVOID lpParam);
	void *m_pApp;
	bool m_bOpenRobotView;
	static CDlgRobotView m_CRobotView;
	
	static void CallBack_RobotView(unsigned char *pucQVGAImg,void *pContext);

	CDlgEditRouteConfirm m_CDlgEditRouteConfirm;
	
};

#ifndef _DEBUG  // debug version in IoTRobot_GUIView.cpp
inline CIoTRobot_GUIDoc* CIoTRobot_GUIView::GetDocument() const
   { return reinterpret_cast<CIoTRobot_GUIDoc*>(m_pDocument); }
#endif

