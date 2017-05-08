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
	ON_WM_MOUSEWHEEL()
	ON_WM_CREATE()
	ON_WM_SIZE()
	ON_WM_KEYDOWN()
	ON_WM_LBUTTONDOWN()
	ON_WM_LBUTTONUP()
	ON_WM_RBUTTONDOWN()
	ON_WM_RBUTTONUP()
	ON_WM_MOUSEMOVE()
	//ON_WM_ERASEBKGND()
	ON_WM_DESTROY()
END_MESSAGE_MAP()

// CIoTRobot_GUIView construction/destruction


GLubyte *CIoTRobot_GUIView::m_pucColor;
GLfloat *CIoTRobot_GUIView::m_pfPC;


GLubyte *CIoTRobot_GUIView::m_pucPIPColor;
GLfloat *CIoTRobot_GUIView::m_pfPIPVertex;

CIoTRobot_GUIView::CIoTRobot_GUIView()
{
	// TODO: add construction code here

}

CIoTRobot_GUIView::~CIoTRobot_GUIView()
{
}


void CIoTRobot_GUIView::CallBack_PIPQVGA(unsigned char *pucQVGAImg,void *pContext)
{
	memcpy(m_pucPIPColor,pucQVGAImg,IOTGUI_PIP_TRIBLESIZE);
}
void CIoTRobot_GUIView::CallBack_PointCloud(unsigned char *pucImg,float *pfVertex,void *pContext)
{
	memcpy(m_pucColor,pucImg,IOTGUI_PC_TRIBLESIZE);
	memcpy(m_pfPC,pfVertex,(IOTGUI_PC_TRIBLESIZE<<1));
}



BOOL CIoTRobot_GUIView::PreCreateWindow(CREATESTRUCT& cs)
{
	// TODO: Modify the Window class or styles here by modifying
	//  the CREATESTRUCT cs
	cs.style |= WS_CLIPSIBLINGS | WS_CLIPCHILDREN;
	return CView::PreCreateWindow(cs);
}

// CIoTRobot_GUIView drawing

void CIoTRobot_GUIView::OnDraw(CDC* /*pDC*/)
{
	CIoTRobot_GUIDoc* pDoc = GetDocument();
	ASSERT_VALID(pDoc);
	if (!pDoc)
		return;

	/*glViewport (0, 0, (GLsizei) 640, (GLsizei) 480);
	glMatrixMode (GL_PROJECTION);
	glLoadIdentity ();
	//glFrustum(-0.6,0.6,-0.45,0.45,1,100);
	gluPerspective(45.0,(GLfloat)640/(GLfloat)480,1.0,100.0);
	glMatrixMode (GL_MODELVIEW);
	glLoadIdentity ();
	//gluLookAt(0.0,0.0,-2,
	//	0.0,-0.2,0.0,
	//	0.0,-1.0,0.0);

	gluLookAt(0.0,0.0,-2,
		0.0,-0,0.0,
		0.0,-1.0,0.0);


	::glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
	RenderScene();
	// Tell OpenGL to flush its pipeline
	::glFinish();
	// Now Swap the buffers
	::SwapBuffers( m_pDC->GetSafeHdc() );*/

	// TODO: add draw code for native data here
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


// CIoTRobot_GUIView diagnostics

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


// CIoTRobot_GUIView message handlers


void CIoTRobot_GUIView::RenderScene()
{
	/*int i;
	glClear(GL_COLOR_BUFFER_BIT);
	glColor3f(1.0,1.0,1.0);
	glEnableClientState(GL_COLOR_ARRAY);
	glEnableClientState(GL_VERTEX_ARRAY);
	glColorPointer(3,GL_UNSIGNED_BYTE,0,m_pucColor);
	glVertexPointer(3,GL_FLOAT,0,m_pfPC);


	glPushMatrix();

	glTranslated(m_dTranslationX, m_dTranslationY, m_dTranslationZ);
	glRotated(m_dRotationX, 1, 0, 0);
	glRotated(m_dRotationY, 0, 1, 0);

	glBegin(GL_POINTS);
	for (i=0;i<IOTGUI_PC_SIZE;i++)
	{
		glArrayElement(i);
	}
	glEnd();

	glBegin(GL_LINES);
	glLineWidth(5);
	glColor3f(1.0,0.0,0.0);
	glVertex3f(0.0,0,0);
	glVertex3f(0.4,0,0);

	glColor3f(0.0,1.0,0.0);
	glVertex3f(0.0,0,0);
	glVertex3f(0.0,0.4,0);

	glColor3f(0.0,0.0,1.0);
	glVertex3f(0.0,0,0);
	glVertex3f(0.0,0,0.4);
	glEnd();


	glPopMatrix();


	if (m_bOpenPIP)
	{
		glColorPointer(3,GL_UNSIGNED_BYTE,0,m_pucPIPColor);
		glVertexPointer(3,GL_FLOAT,0,m_pfPIPVertex);
		glBegin(GL_POINTS);


		for (i=0;i<320*240;i++)
		{
			glArrayElement(i);
		}

		glEnd();
	}

	//glutSwapBuffers();
	glFlush();

	Invalidate(FALSE);*/



	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); // Clear Screen And Depth Buffer
	glLoadIdentity();


	static GLfloat xrot; 
	static GLfloat yrot; 
	static GLfloat zrot; 

	glPushMatrix(); 
	glLoadIdentity(); 
	glTranslatef(0.0f,0.0f,-6.0f); 
	glRotatef(xrot,1.0f,0.0f,0.0f);
	glRotatef(yrot,0.0f,1.0f,0.0f);
	glRotatef(zrot,0.0f,0.0f,1.0f);
	glBegin(GL_QUADS);

	// Front Face

	if (!m_bOpenPIP)
	{



	glColor3f(1.0f,0.0f,0.0f);glVertex3f(-1.0f, -1.0f, 1.0f);

	glColor3f(0.0f,1.0f,0.0f);glVertex3f( 1.0f, -1.0f, 1.0f);
	glColor3f(0.0f,1.0f,0.0f);glVertex3f( 1.0f, 1.0f, 1.0f);
	glColor3f(0.0f,0.0f,1.0f);glVertex3f(-1.0f, 1.0f, 1.0f);

	// Back Face

	glColor3f(1.0f,0.0f,0.0f);glVertex3f(-1.0f, -1.0f, -1.0f);
	glColor3f(0.0f,1.0f,0.0f);glVertex3f(-1.0f, 1.0f, -1.0f);
	glColor3f(0.0f,1.0f,0.0f);glVertex3f( 1.0f, 1.0f, -1.0f);
	glColor3f(0.0f,0.0f,1.0f);glVertex3f( 1.0f, -1.0f, -1.0f);

	// Top Face

	glColor3f(0.0f,1.0f,0.0f);glVertex3f(-1.0f, 1.0f, -1.0f);
	glColor3f(0.0f,0.0f,1.0f);glVertex3f(-1.0f, 1.0f, 1.0f);
	glColor3f(0.0f,1.0f,0.0f);glVertex3f( 1.0f, 1.0f, 1.0f);
	glColor3f(0.0f,1.0f,0.0f);glVertex3f( 1.0f, 1.0f, -1.0f);

	// Bottom Face

	glColor3f(1.0f,0.0f,0.0f);glVertex3f(-1.0f, -1.0f, -1.0f);
	glColor3f(0.0f,0.0f,1.0f);glVertex3f( 1.0f, -1.0f, -1.0f);
	glColor3f(0.0f,1.0f,0.0f);glVertex3f( 1.0f, -1.0f, 1.0f);
	glColor3f(1.0f,0.0f,0.0f);glVertex3f(-1.0f, -1.0f, 1.0f);

	// Right face

	glColor3f(0.0f,0.0f,1.0f);glVertex3f( 1.0f, -1.0f, -1.0f);
	glColor3f(0.0f,1.0f,0.0f);glVertex3f( 1.0f, 1.0f, -1.0f);
	glColor3f(0.0f,1.0f,0.0f);glVertex3f( 1.0f, 1.0f, 1.0f);
	glColor3f(0.0f,1.0f,0.0f);glVertex3f( 1.0f, -1.0f, 1.0f);
	}

	// Left Face

	glColor3f(1.0f,0.0f,0.0f);glVertex3f(-1.0f, -1.0f, -1.0f);
	glColor3f(1.0f,0.0f,0.0f);glVertex3f(-1.0f, -1.0f, 1.0f);
	glColor3f(0.0f,0.0f,1.0f);glVertex3f(-1.0f, 1.0f, 1.0f);
	glColor3f(0.0f,1.0f,0.0f);glVertex3f(-1.0f, 1.0f, -1.0f);
	glEnd();
	glPopMatrix(); 

	xrot+=1.3f;
	yrot+=1.2f;
	zrot+=1.4f; 
	
	{
		Invalidate(FALSE);
	}
}


int CIoTRobot_GUIView::OnCreate(LPCREATESTRUCT lpCreateStruct)
{
	if (CView::OnCreate(lpCreateStruct) == -1)
		return -1;

	int i,j,idx;
	m_enumMouseDrag=None;
	m_pucColor=new GLubyte[IOTGUI_PC_TRIBLESIZE];
	m_pfPC=new GLfloat[IOTGUI_PC_TRIBLESIZE];

	m_pucPIPColor=new GLubyte[IOTGUI_PIP_TRIBLESIZE];
	m_pfPIPVertex=new GLfloat[IOTGUI_PIP_TRIBLESIZE];


	memset(m_pucColor,0,IOTGUI_PC_TRIBLESIZE);
	memset(m_pfPC,0,IOTGUI_PC_TRIBLESIZE*2);

	memset(m_pucPIPColor,0,IOTGUI_PIP_TRIBLESIZE);
	memset(m_pfPIPVertex,0,IOTGUI_PIP_TRIBLESIZE*2);

	idx=0;
	for (j=0;j<240;j++)
	{
		for (i=0;i<320;i++)
		{
			m_pfPIPVertex[idx]=X_RATE*i+0.5;
			idx++;
			m_pfPIPVertex[idx]=-Y_RATE*j-0.35;
			idx++;
			m_pfPIPVertex[idx]=0;
			idx++;
		}
	}
	glClearColor(0,0,0,0);
	glShadeModel(GL_FLAT);


	m_dRotationX=0;
	m_dRotationY=0;
	m_dTranslationZ=0;
	m_dTranslationX=0;
	m_dTranslationY=0;

	m_bOpenPIP=false;
	m_MouseDownPoint.x=0;
	m_MouseDownPoint.y=0;


	// TODO:  Add your specialized creation code here
	InitializeOpenGL();
	return 0;
}

BOOL CIoTRobot_GUIView::InitializeOpenGL()
{
	m_pDC = new CClientDC(this);

	//Failure to Get DC
	if(m_pDC == NULL)
	{
		//		MessageBox("Error Obtaining DC");
		return FALSE;
	}
	//Failure to set the pixel format
	if(!SetupPixelFormat())
	{
		return FALSE;
	}
	//Create Rendering Context
	m_hRC = ::wglCreateContext (m_pDC->GetSafeHdc ());
	//Failure to Create Rendering Context
	if(m_hRC == 0)
	{
		//		MessageBox("Error Creating RC");
		return FALSE;
	}
	//Make the RC Current
	if(::wglMakeCurrent (m_pDC->GetSafeHdc (), m_hRC)==FALSE)
	{
		//		MessageBox("Error making RC Current");
		return FALSE;
	}
	//Specify Black as the clear color
	::glClearColor(0.0f,0.0f,0.0f,0.0f);
	//Specify the back of the buffer as clear depth
	::glClearDepth(1.0f);
	//Enable Depth Testing
	::glEnable(GL_DEPTH_TEST);
	return TRUE;
}
//Setup Pixel Format
/////////////////////////////////////////////////////////////////////////////
BOOL CIoTRobot_GUIView::SetupPixelFormat()
{
	static PIXELFORMATDESCRIPTOR pfd = 
	{
		sizeof(PIXELFORMATDESCRIPTOR),  // size of this pfd
		1,                              // version number
		PFD_DRAW_TO_WINDOW |            // support window
		PFD_SUPPORT_OPENGL |            // support OpenGL
		PFD_DOUBLEBUFFER,                // double buffered
		PFD_TYPE_RGBA,                  // RGBA type
		24,                             // 24-bit color depth
		0, 0, 0, 0, 0, 0,               // color bits ignored
		0,                              // no alpha buffer
		0,                              // shift bit ignored
		0,                              // no accumulation buffer
		0, 0, 0, 0,                     // accum bits ignored
		16,                             // 16-bit z-buffer
		0,                              // no stencil buffer
		0,                              // no auxiliary buffer
		PFD_MAIN_PLANE,                 // main layer
		0,                              // reserved
		0, 0, 0                         // layer masks ignored
	};
	int m_nPixelFormat = ::ChoosePixelFormat(m_pDC->GetSafeHdc(), &pfd);
	if ( m_nPixelFormat == 0 )
	{
		return FALSE;
	}
	if ( ::SetPixelFormat(m_pDC->GetSafeHdc(), m_nPixelFormat, &pfd) == FALSE)
	{
		return FALSE;
	}
	return TRUE;
}

void CIoTRobot_GUIView::OnSize(UINT nType, int cx, int cy)
{
	CView::OnSize(nType, cx, cy);
	GLdouble aspect_ratio; // width/height ratio

	if ( 0 >= cx || 0 >= cy )
	{
		return;
	}
	// select the full client area
	::glViewport(0, 0, cx, cy);
	// compute the aspect ratio
	// this will keep all dimension scales equal
	aspect_ratio = (GLdouble)cx/(GLdouble)cy;
	// select the projection matrix and clear it
	::glMatrixMode(GL_PROJECTION);
	::glLoadIdentity();
	// select the viewing volume
	::gluPerspective(45.0f, aspect_ratio, .01f, 200.0f);

	// switch back to the modelview matrix and clear it
	::glMatrixMode(GL_MODELVIEW);
	::glLoadIdentity();

	// TODO: Add your message handler code here
}
void CIoTRobot_GUIView::OnKeyDown(UINT nChar, UINT nRepCnt, UINT nFlags) 
{
	// TODO: Add your message handler code here and/or call default

	char tmp=(char)nChar;
	if (tmp=='O')
	{
		m_bOpenPIP=(m_bOpenPIP?false:true);
	}
	switch (tmp)
	{
	case'w':
		m_dRotationX -= 3;
		//glutPostRedisplay();
		break;
	case 's':
		m_dRotationX += 3;
		//glutPostRedisplay();
		break;
	case 'a':
		m_dRotationY -= 3;
		//glutPostRedisplay();
		break;
	case 'd':
		m_dRotationY += 3;
		//glutPostRedisplay();
		break;

	case 'q':
		m_dRotationX -= 3;
		m_dRotationY -= 3;
		//glutPostRedisplay();
		break;

	case 'e':
		m_dRotationX -= 3;
		m_dRotationY += 3;
		//glutPostRedisplay();
		break;


	case 'z':
		m_dRotationY -= 3;
		m_dRotationX += 3;
		//glutPostRedisplay();
		break;

	case 'c':
		m_dRotationX += 3;
		m_dRotationY += 3;
		//glutPostRedisplay();
		break;
	case '1':
		m_dRotationX=0;
		m_dRotationY=0;
		m_dTranslationZ=0;
		m_dTranslationX=0;
		m_dTranslationY=0;
		//glutPostRedisplay();
		break;
	case '2':
		m_dRotationX=90;
		m_dTranslationY=2;
		//glutPostRedisplay();
		break;

	case 'o':
		m_bOpenPIP=(m_bOpenPIP?false:true);
		//glutPostRedisplay();
		break;

	default:
		break;
	}        

	InvalidateRect(NULL,FALSE);

	CView::OnKeyDown(nChar, nRepCnt, nFlags);
}

void CIoTRobot_GUIView::OnLButtonDown(UINT nFlags, CPoint point) 
{
	// TODO: Add your message handler code here and/or call default
	m_MouseDownPoint=point;
	m_enumMouseDrag=Rotate;
	SetCapture();

	CView::OnLButtonDown(nFlags, point);
}

void CIoTRobot_GUIView::OnLButtonUp(UINT nFlags, CPoint point) 
{
	// TODO: Add your message handler code here and/or call default
	m_MouseDownPoint=CPoint(0,0);
	ReleaseCapture();

	CView::OnLButtonUp(nFlags, point);
}


void CIoTRobot_GUIView::OnRButtonDown(UINT nFlags, CPoint point) 
{
	// TODO: Add your message handler code here and/or call default
	m_MouseDownPoint=point;
	m_enumMouseDrag=Translate;
	SetCapture();

	CView::OnRButtonDown(nFlags, point);
}

void CIoTRobot_GUIView::OnRButtonUp(UINT nFlags, CPoint point) 
{
	// TODO: Add your message handler code here and/or call default
	m_MouseDownPoint=CPoint(0,0);
	ReleaseCapture();

	CView::OnRButtonUp(nFlags, point);
}

void CIoTRobot_GUIView::OnMouseMove(UINT nFlags, CPoint point) 
{
	// TODO: Add your message handler code here and/or call default
	// Check if we have captured the mouse
	if (GetCapture()==this)
	{
		GLdouble dx = point.x- m_MouseDownPoint.x;
		GLdouble dy = point.y - m_MouseDownPoint.y;

		// Update the camera transformation.
		switch (m_enumMouseDrag) {
		case Rotate:
			m_dRotationY += 360 * dx / IOTGUI_PC_WIDTH;
			m_dRotationX += 360 * dy / IOTGUI_PC_HEIGHT;
			break;
		case Zoom:
			m_dTranslationZ += 10 * dy / IOTGUI_PC_HEIGHT;
			break;
		case Translate:
			m_dTranslationX += 10 * dx / IOTGUI_PC_WIDTH;
			m_dTranslationY += - 10 * dy / IOTGUI_PC_HEIGHT;
			break;
		case None:
			break;
		}
		m_MouseDownPoint=point;
	}

	CView::OnMouseMove(nFlags, point);
}

BOOL  CIoTRobot_GUIView:: OnMouseWheel(   UINT   nFlags,   short   zDelta,   CPoint   pt   )
{
	if (zDelta>0)
	{
		m_dTranslationZ += 1;
	}
	else if(zDelta<0)
	{
		m_dTranslationZ-=1;
	}
	
	CView::OnMouseWheel(nFlags,zDelta, pt);
	return TRUE;
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
	if(::wglMakeCurrent (0,0) == FALSE)
	{
		//MessageBox("Could not make RC non-current");
	}

	//Delete the rendering context
	if(::wglDeleteContext (m_hRC)==FALSE)
	{
		//MessageBox("Could not delete RC");
	}
	//Delete the DC
	if(m_pDC)
	{
		delete m_pDC;
	}
	//Set it to NULL
	m_pDC = NULL;

	if (m_pucColor)
	{
		delete [] m_pucColor;
		m_pucColor=NULL;
	}


	if (m_pfPC)
	{
		delete [] m_pfPC;
		m_pfPC=NULL;
	}

	if (m_pucPIPColor)
	{
		delete [] m_pucPIPColor;
		m_pucPIPColor=NULL;
	}

	if (m_pfPIPVertex)
	{
		delete [] m_pfPIPVertex;
		m_pfPIPVertex=NULL;
	}
}
