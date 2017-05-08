// IoTRobot_GUIView.cpp : implementation of the CIoTRobot_GUIView class
//

#include "stdafx.h"
#include "IoTRobot_GUI.h"

#include "IoTRobot_GUIDoc.h"
#include "IoTRobot_GUIView.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#endif


// CIoTRobot_GUIView

IMPLEMENT_DYNCREATE(CIoTRobot_GUIView, CView)

BEGIN_MESSAGE_MAP(CIoTRobot_GUIView, CView)
	// Standard printing commands
	ON_COMMAND(ID_FILE_PRINT, &CView::OnFilePrint)
	ON_COMMAND(ID_FILE_PRINT_DIRECT, &CView::OnFilePrint)
	ON_COMMAND(ID_FILE_PRINT_PREVIEW, &CView::OnFilePrintPreview)

	ON_COMMAND(ID_BUTTON_PC,&CIoTRobot_GUIView::OnDisplayPC)
	ON_COMMAND(ID_BUTTON_PC_VIEW,&CIoTRobot_GUIView::OnDisplayPC_View)
	ON_COMMAND(ID_BUTTON_STATE_LIGHT,&CIoTRobot_GUIView::OnDisplayStateLight)
	ON_COMMAND(ID_BUTTON_PATH,&CIoTRobot_GUIView::OnDisplayPath)
	ON_COMMAND(ID_BUTTON_WINKLEPATH,&CIoTRobot_GUIView::OnDisplayTwinklingPath)


	ON_WM_CREATE()
	ON_WM_SIZE()
	ON_WM_ERASEBKGND()
	ON_WM_DESTROY()
	ON_WM_PAINT()
	ON_WM_KEYDOWN()
	ON_WM_MOUSEWHEEL()
END_MESSAGE_MAP()


CIoTRobot_GUIView::CIoTRobot_GUIView()
{
	// TODO: add construction code here

}

CIoTRobot_GUIView::~CIoTRobot_GUIView()
{
}

BOOL CIoTRobot_GUIView::PreCreateWindow(CREATESTRUCT& cs)
{
	// TODO: Modify the Window class or styles here by modifying
	//  the CREATESTRUCT cs

	return CView::PreCreateWindow(cs);
}



/*
void CIoTRobot_GUIView::CallBack_PIPQVGA(unsigned char *pucQVGAImg,void *pContext)
{
	//memcpy(m_pucPIPColor,pucQVGAImg,IOTGUI_PIP_TRIBLESIZE);
}
void CIoTRobot_GUIView::CallBack_PointCloud(unsigned char *pucImg,float *pfVertex,void *pContext)
{
	//memcpy(m_pucColor,pucImg,IOTGUI_PC_TRIBLESIZE);
	//memcpy(m_pfPC,pfVertex,(IOTGUI_PC_TRIBLESIZE<<1));
}
*/


bool g_start;
void CIoTRobot_GUIView::OnDraw(CDC* /*pDC*/)
{
	CIoTRobot_GUIDoc* pDoc = GetDocument();
	ASSERT_VALID(pDoc);
	if (!pDoc)
		return;
	g_start=true;
}


// CIoTRobot_GUIView printing

BOOL CIoTRobot_GUIView::OnPreparePrinting(CPrintInfo* pInfo)
{
	// default preparation
	return DoPreparePrinting(pInfo);
}

void CIoTRobot_GUIView::OnBeginPrinting(CDC* /*pDC*/, CPrintInfo* /*pInfo*/)
{
	// TODO: add extra initialization before printing
}

void CIoTRobot_GUIView::OnEndPrinting(CDC* /*pDC*/, CPrintInfo* /*pInfo*/)
{
	// TODO: add cleanup after printing
}




#ifdef _DEBUG
void CIoTRobot_GUIView::AssertValid() const
{
	CView::AssertValid();
}

void CIoTRobot_GUIView::Dump(CDumpContext& dc) const
{
	CView::Dump(dc);
}

CIoTRobot_GUIDoc* CIoTRobot_GUIView::GetDocument() const // non-debug version is inline
{
	ASSERT(m_pDocument->IsKindOf(RUNTIME_CLASS(CIoTRobot_GUIDoc)));
	return (CIoTRobot_GUIDoc*)m_pDocument;
}
#endif //_DEBUG




int CIoTRobot_GUIView::OnCreate(LPCREATESTRUCT lpCreateStruct)
{
	if (CView::OnCreate(lpCreateStruct) == -1)
		return -1;

	g_start=false;
	RECT rect;
	rect.left=0;
	rect.top=0;
	rect.bottom=480;
	rect.right=640;
	m_staticImg.Create("",WS_VISIBLE|WS_CHILD|SS_BLACKFRAME|WS_CLIPCHILDREN,rect,this,IDC_STATIC_IMG1);

	LPDWORD ThreadRenderID=0;
	CreateThread(NULL,0,(LPTHREAD_START_ROUTINE)ThreadRender,this,0,ThreadRenderID);
	return 0;
}



void CIoTRobot_GUIView::OnSize(UINT nType, int cx, int cy)
{
	CView::OnSize(nType, cx, cy);
//	GLdouble aspect_ratio; // width/height ratio

	RECT rect;
	this->GetClientRect(&rect);
	m_staticImg.MoveWindow(&rect);
	m_cOpenGL.ReSizeGLScene(cx,cy);
	if ( 0 >= cx || 0 >= cy )
	{
		return;
	}
	// TODO: Add your message handler code here
}


BOOL CIoTRobot_GUIView::OnEraseBkgnd(CDC* pDC) 
{
	//Tell Windows not to erase the background
	return TRUE;
}

void CIoTRobot_GUIView::OnDestroy() 
{
	CView::OnDestroy();
	//Make the RC non-current
}



void CIoTRobot_GUIView::OnPaint()
{
	CPaintDC dc(this); // device context for painting
	OnDraw(&dc);
}


UINT CIoTRobot_GUIView::ThreadRender(LPVOID lpParam)
{
	CIoTRobot_GUIView *pView=(CIoTRobot_GUIView *)lpParam;

	while (!g_start)
	{
		Sleep(10);
	}
	pView->m_cOpenGL.OpenlInterface(pView->m_staticImg.m_hWnd,GetModuleHandle(0));
	while (1)
	{
		Sleep(100);
	}
}
void CIoTRobot_GUIView::OnKeyDown(UINT nChar, UINT nRepCnt, UINT nFlags)
{
	// TODO: Add your message handler code here and/or call default
	char tmp=(char)nChar;
	switch (tmp)
	{
	case'W':
		m_cOpenGL.m_dRotationX -= 3;
		//glutPostRedisplay();
		break;
	case 'S':
		m_cOpenGL.m_dRotationX += 3;
		//glutPostRedisplay();
		break;
	case 'A':
		m_cOpenGL.m_dRotationY -= 3;
		//glutPostRedisplay();
		break;
	case 'D':
		m_cOpenGL.m_dRotationY += 3;
		//glutPostRedisplay();
		break;

	case 'Q':
		m_cOpenGL.m_dRotationX -= 3;
		m_cOpenGL.m_dRotationY -= 3;
		//glutPostRedisplay();
		break;

	case 'E':
		m_cOpenGL.m_dRotationX -= 3;
		m_cOpenGL.m_dRotationY += 3;
		//glutPostRedisplay();
		break;


	case 'Z':
		m_cOpenGL.m_dRotationY -= 3;
		m_cOpenGL.m_dRotationX += 3;
		//glutPostRedisplay();
		break;

	case 'C':
		m_cOpenGL.m_dRotationX += 3;
		m_cOpenGL.m_dRotationY += 3;
		//glutPostRedisplay();
		break;
	case '1':
		m_cOpenGL.m_dRotationX=0;
		m_cOpenGL.m_dRotationY=0;
		m_cOpenGL.m_dTranslationZ=0;
		m_cOpenGL.m_dTranslationX=0;
		m_cOpenGL.m_dTranslationY=0;
		//glutPostRedisplay();
		break;
	case '2':
		m_cOpenGL.m_dRotationX=90;
		m_cOpenGL.m_dTranslationY=2;
		//glutPostRedisplay();
		break;

	case 'P':
		m_cOpenGL.m_bOpenPIP=(m_cOpenGL.m_bOpenPIP?false:true);
		//glutPostRedisplay();
		break;

	case 'L':
		m_cOpenGL.m_bOpenStateLight=(m_cOpenGL.m_bOpenStateLight?false:true);
		//glutPostRedisplay();
		break;
	case'O':
		m_cOpenGL.m_bPathTwinkle=(m_cOpenGL.m_bPathTwinkle?false:true);
		//m_stGLPath.pstPathCur=m_stGLPath.pstPathHead;
		//memcpy(m_stPathTwinkle.fLastPos,m_stGLPath.pstPathCur->pstSonNode,12);
		//memcpy(m_stPathTwinkle.fCurPos,m_stGLPath.pstPathCur->pstSonNode,12);
		m_cOpenGL.m_stPathTwinkle.ulStartTime=0;
		break;
	case 'I':
		m_cOpenGL.m_bShowPath=(m_cOpenGL.m_bShowPath?false:true);
		break;

	default:
		break;
	}        
	CView::OnKeyDown(nChar, nRepCnt, nFlags);
}

BOOL CIoTRobot_GUIView::OnMouseWheel(UINT nFlags, short zDelta, CPoint pt)
{
	// TODO: Add your message handler code here and/or call default
	if (zDelta>0)
	{
		m_cOpenGL.m_dTranslationZ += 1;
	}
	else if(zDelta<0)
	{
		m_cOpenGL.m_dTranslationZ-=1;
	}
	return TRUE;
	return CView::OnMouseWheel(nFlags, zDelta, pt);
}


void CIoTRobot_GUIView::OnDisplayPC()
{
	m_cOpenGL.m_bOpenPIP=false;
}

void CIoTRobot_GUIView::OnDisplayPC_View()
{
	m_cOpenGL.m_bOpenPIP=(m_cOpenGL.m_bOpenPIP?false:true);
}

void CIoTRobot_GUIView::OnDisplayStateLight()
{
	m_cOpenGL.m_bOpenStateLight=(m_cOpenGL.m_bOpenStateLight?false:true);
}

void CIoTRobot_GUIView::OnDisplayPath()
{
	m_cOpenGL.m_bShowPath=(m_cOpenGL.m_bShowPath?false:true);
}

void CIoTRobot_GUIView::OnDisplayTwinklingPath()
{
	m_cOpenGL.m_bPathTwinkle=(m_cOpenGL.m_bPathTwinkle?false:true);
	//m_stGLPath.pstPathCur=m_stGLPath.pstPathHead;
	//memcpy(m_stPathTwinkle.fLastPos,m_stGLPath.pstPathCur->pstSonNode,12);
	//memcpy(m_stPathTwinkle.fCurPos,m_stGLPath.pstPathCur->pstSonNode,12);
	m_cOpenGL.m_stPathTwinkle.ulStartTime=0;
}