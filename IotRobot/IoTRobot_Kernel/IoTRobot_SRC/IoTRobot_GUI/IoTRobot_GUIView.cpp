// IoTRobot_GUIView.cpp : implementation of the CIoTRobot_GUIView class
//

#include "stdafx.h"
#include "IoTRobot_GUI.h"

#include "IoTRobot_GUIDoc.h"
#include "IoTRobot_GUIView.h"
#include "MainFrm.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#endif

CDlgRobotView CIoTRobot_GUIView::m_CRobotView;
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
	ON_COMMAND(ID_BUTTON_COMPASS,&CIoTRobot_GUIView::OnCompass)
	ON_COMMAND(ID_BUTTON_TRIANGLATION,&CIoTRobot_GUIView::OnTriangulation)
	ON_COMMAND(ID_BUTTON_MANUAL_PATH,&CIoTRobot_GUIView::OnManualPath)


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
	m_bStopThreadRender=true;
	m_bOpenRobotView=false;

	m_pApp=(void *)AfxGetApp();

}	

CIoTRobot_GUIView::~CIoTRobot_GUIView()
{
	m_cOpenGL.m_bStopOpenGL=true;
	m_bStopThreadRender=true;
	if (m_hThreadRenderRun!=0)
	{
		WaitForSingleObject(m_hThreadRenderRun,INFINITE);
	}
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

	m_bStopThreadRender=false;
	LPDWORD ThreadRenderID=0;
	m_hThreadRenderRun=CreateThread(NULL,0,(LPTHREAD_START_ROUTINE)ThreadRender,this,0,ThreadRenderID);
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
	while (!pView->m_bStopThreadRender)
	{
		Sleep(100);
	}
	return 0;
}
void CIoTRobot_GUIView::OnKeyDown(UINT nChar, UINT nRepCnt, UINT nFlags)
{
	CIoTRobot_GUIApp *pApp=(CIoTRobot_GUIApp *)::AfxGetApp();
	// TODO: Add your message handler code here and/or call default
	char tmp=(char)nChar;
	switch (tmp)
	{
	case'W':
		if (pApp->m_nCurrentRunMode==IDRIVE_MODE)
		{
			m_cOpenGL.m_dKinectPose[2]+=0.1;
			Sleep(10);
		}
		m_cOpenGL.m_dRotationX -= 3;
		//glutPostRedisplay();
		break;
	case 'S':
		if (pApp->m_nCurrentRunMode==IDRIVE_MODE)
		{
			m_cOpenGL.m_dKinectPose[2]-=0.1;
			Sleep(10);
		}
		m_cOpenGL.m_dRotationX += 3;
		//glutPostRedisplay();
		break;
	case 'A':
		if (pApp->m_nCurrentRunMode==IDRIVE_MODE)
		{
			m_cOpenGL.m_dKinectPose[0]-=0.1;
			Sleep(10);
		}
		m_cOpenGL.m_dRotationY -= 3;
		//glutPostRedisplay();
		break;
	case 'D':
		if (pApp->m_nCurrentRunMode==IDRIVE_MODE)
		{
			m_cOpenGL.m_dKinectPose[0]+=0.1;
			Sleep(10);
		}
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

	//case 'P':
		//m_cOpenGL.m_bOpenPIP=(m_cOpenGL.m_bOpenPIP?false:true);
		//glutPostRedisplay();
		//break;

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
	//m_cOpenGL.m_bOpenPIP=false;
}

void CIoTRobot_GUIView::OnDisplayPC_View()
{
	IoTRobot_Message MSG;
	m_bOpenRobotView=(m_bOpenRobotView?false:true);
	LPDWORD ID=0;
	if (m_bOpenRobotView)
	{
		CreateThread(NULL,0,(LPTHREAD_START_ROUTINE)ThreadRobotView,this,0,ID);

		HWND  hwnd;
		while ((hwnd=m_CRobotView.GetSafeHwnd())==0)
		{
			Sleep(10);
		}

		MSG.cCommand=2;
		MSG.cDestModule=IDRIVE_MSG;
		MSG.cFromModule=GUI_MODULE;
		MSG.nParam1=1;
		MSG.nParam2=(int)hwnd;
		MSG.pcParam3=NULL;

	   ((CIoTRobot_GUIApp *)m_pApp)->m_pCSendCMD2Kernel(&MSG);
	}
	else
	{
		MSG.cCommand=2;
		MSG.cDestModule=IDRIVE_MSG;
		MSG.cFromModule=GUI_MODULE;
		MSG.nParam1=0;
		MSG.nParam2=0;
		MSG.pcParam3=NULL;

		((CIoTRobot_GUIApp *)m_pApp)->m_pCSendCMD2Kernel(&MSG);
		m_CRobotView.EndDialog(NULL);
	}
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

void CIoTRobot_GUIView::OnCompass()
{
	m_cOpenGL.m_bCompass=(m_cOpenGL.m_bCompass?false:true);
}


void CIoTRobot_GUIView::OnTriangulation()
{
	m_cOpenGL.m_bTriangulation=(m_cOpenGL.m_bTriangulation?false:true);
}


void CIoTRobot_GUIView::OnManualPath()
{
	m_cOpenGL.m_nManualPath=(m_cOpenGL.m_nManualPath?false:true);
	CMainFrame *pFrame=(CMainFrame *) ::AfxGetMainWnd();
	if (m_cOpenGL.m_nManualPath)
	{
		m_cOpenGL.m_dRotationX=90;
		m_cOpenGL.m_dRotationY=0;
		m_cOpenGL.m_dTranslationY=0;
		m_cOpenGL.m_dTranslationX=0;
		m_cOpenGL.m_dTranslationZ=4;


		m_cOpenGL.m_nManualPathArrayLen=0;
		m_cOpenGL.m_nInflectionPointsNum=0;
		m_cOpenGL.m_nMileStoneLen=0;
		m_cOpenGL.m_nRobotPathArrayLen=0;
		m_cOpenGL.m_nMileStoneHasBeenBuild=0;

		//m_cOpenGL.DrawScaleGridLines();

		

	//	pFrame->m_wndControlToolBar.GetToolBarCtrl().SetState(ID_BUTTON_MANUAL_PATH,TBSTATE_PRESSED);

		LPDWORD ID=0;
		CreateThread(NULL,0,(LPTHREAD_START_ROUTINE)ThreadWaitForMileStone,this,0,ID);
	}
/*	else
	{
		pFrame->m_wndControlToolBar.GetToolBarCtrl().SetState(ID_BUTTON_MANUAL_PATH,TBSTYLE_FLAT);
	}*/
}

UINT CIoTRobot_GUIView::ThreadWaitForMileStone(LPVOID lpParam)
{
	CIoTRobot_GUIView *pView=(CIoTRobot_GUIView *)lpParam;
	CIoTRobot_GUIApp *pApp=(CIoTRobot_GUIApp *)pView->m_pApp;
	IoTRobot_Message MSG;
	AFX_MANAGE_STATE(AfxGetStaticModuleState()); 
	while (!pView->m_cOpenGL.m_nMileStoneHasBeenBuild)
	{
		Sleep(5);
	}
	pView->m_CDlgEditRouteConfirm.m_nConfirmRslt=0;
	pView->m_CDlgEditRouteConfirm.DoModal();
	pView->m_cOpenGL.m_nManualPath=0;
	//Cancel
	if (pView->m_CDlgEditRouteConfirm.m_nConfirmRslt==0)
	{
		pView->m_cOpenGL.m_nManualPathArrayLen=0;
		pView->m_cOpenGL.m_nMileStoneLen=0;
		memset(pView->m_cOpenGL.m_fMileStonePos,0,1200);
	}
	//Retry
	else if (pView->m_CDlgEditRouteConfirm.m_nConfirmRslt==2)
	{
		pView->OnManualPath();
	}
	
	
	return 0;
}

UINT CIoTRobot_GUIView::ThreadRobotView(LPVOID lpParam)
{

	CIoTRobot_GUIView *pView=(CIoTRobot_GUIView *)lpParam;
	CIoTRobot_GUIApp *pApp=(CIoTRobot_GUIApp *)pView->m_pApp;
	AFX_MANAGE_STATE(AfxGetStaticModuleState()); 

	pView->m_CRobotView.DoModal();
	pView->m_bOpenRobotView=false;
	return 0;
}

 void CIoTRobot_GUIView::CallBack_RobotView(unsigned char *pucQVGAImg,void *pContext)
 {
	 CRect rect;
#ifdef QVGA
	 memcpy(m_CRobotView.m_pucRobotView,pucQVGAImg,320*240*3);
#endif
	 
#ifdef CIF
	 memcpy(m_CRobotView.m_pucRobotView,pucQVGAImg,IOTGUI_PIP_TRIBLESIZE_CIF);
#endif
	 

 }