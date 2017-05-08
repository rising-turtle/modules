// IoTRobot_GUIView.h : interface of the CIoTRobot_GUIView class
//


#pragma once


class CIoTRobot_GUIView : public CView
{
protected: // create from serialization only
	CIoTRobot_GUIView();
	DECLARE_DYNCREATE(CIoTRobot_GUIView)

// Attributes
public:
	CIoTRobot_GUIDoc* GetDocument() const;

	static GLubyte *m_pucColor;
	static GLfloat *m_pfPC;


	static GLubyte *m_pucPIPColor;
	static GLfloat *m_pfPIPVertex;

	GLdouble m_dRotationX ;
	GLdouble m_dRotationY;
	GLdouble m_dTranslationX ;
	GLdouble m_dTranslationY ;
	GLdouble m_dTranslationZ ;

	bool m_bOpenPIP;
	CPoint m_MouseDownPoint;

	CStatic m_staticImg;

	MouseDragMode m_enumMouseDrag ;

	static void CallBack_PIPQVGA(unsigned char *pucQVGAImg,void *pContext);
	static void CallBack_PointCloud(unsigned char *pucImg,float *pfVertex,void *pContext);


	HGLRC m_hRC;    //Rendering Context
	CDC* m_pDC;        //Device Context
	BOOL InitializeOpenGL();    //Initialize OpenGL
	BOOL SetupPixelFormat();    //Set up the Pixel Format
	void RenderScene();            //Render the Scene

// Operations
public:

// Overrides
public:
	virtual void OnDraw(CDC* pDC);  // overridden to draw this view
	virtual BOOL PreCreateWindow(CREATESTRUCT& cs);
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
	afx_msg void OnKeyDown(UINT nChar, UINT nRepCnt, UINT nFlags);
	afx_msg void OnLButtonDown(UINT nFlags, CPoint point);
	afx_msg void OnLButtonUp(UINT nFlags, CPoint point);
	afx_msg void OnRButtonDown(UINT nFlags, CPoint point);
	afx_msg void OnRButtonUp(UINT nFlags, CPoint point);
	afx_msg	void OnMouseMove(UINT nFlags, CPoint point);
	afx_msg BOOL OnMouseWheel(UINT nFlags,short zDelta,CPoint pt);
	afx_msg BOOL OnEraseBkgnd(CDC* pDC);
	afx_msg void OnDestroy();

};

#ifndef _DEBUG  // debug version in IoTRobot_GUIView.cpp
inline CIoTRobot_GUIDoc* CIoTRobot_GUIView::GetDocument() const
   { return reinterpret_cast<CIoTRobot_GUIDoc*>(m_pDocument); }
#endif

