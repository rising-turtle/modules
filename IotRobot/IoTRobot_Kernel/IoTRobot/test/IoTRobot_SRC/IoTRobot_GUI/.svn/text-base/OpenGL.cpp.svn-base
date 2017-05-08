//
// This code was created by Jeff Molofee '99
//
// If you've found this code useful, please let me know.
//
// Visit me at www.demonews.com/hosted/nehe
//


//#include <gl\glaux.h>	// Header File For The GLaux Library
#include "stdafx.h"
#include "OpenGL.h"
#include "afxmt.h"

#define glRED glColor3f(1.0,0,0)
#define glGREEN glColor3f(0,1.0,0)
#define glBLUE glColor3f(0,0,1.0)
#define glWHITE glColor3f(1.0,1.0,1.0)
#define glPINK glColor3f(1.0,0,1.0)


float * OpenGL::m_pfPIPVertex;
unsigned char * OpenGL::m_pucPIPColor;
float * OpenGL::m_pfPC;
unsigned char * OpenGL::m_pucColor;

GLdouble OpenGL::m_dRotationX;
GLdouble OpenGL::m_dRotationY;
GLdouble OpenGL::m_dTranslationX ;
GLdouble OpenGL::m_dTranslationY ;
GLdouble OpenGL::m_dTranslationZ ;

bool  OpenGL::m_bOpenPIP;
bool  OpenGL::m_bStopRender;
LPDWORD  OpenGL::ThreadRenderID;
CPoint  OpenGL::m_MouseDownPoint;
MouseDragMode  OpenGL::m_enumMouseDrag ;
CPoint  OpenGL::m_nWndSize;

GLubyte *OpenGL::m_pucSLAMColor;
GLfloat *OpenGL::m_pfSLAMPC;
int OpenGL::m_nCurrentCount;

bool OpenGL::m_bOpenStateLight;
GLbyte OpenGL::m_cRunState;
IoTRobot_GLPath OpenGL::m_stGLPath;

bool OpenGL::m_bStart2AddNode;
bool OpenGL::m_bPathTwinkle;
IoTRobot_PathTwinkleParam OpenGL::m_stPathTwinkle;
bool OpenGL::m_bShowPath;;

CCriticalSection critical_section;

bool OpenGL::m_bNoIMUInfo;
bool OpenGL::m_bUseIMU;

OpenGL::OpenGL()
{
	int i,j,idx;
	m_enumMouseDrag=None;
	m_pucColor=new GLubyte[IOTGUI_PC_TRIBLESIZE];
	m_pfPC=new GLfloat[IOTGUI_PC_TRIBLESIZE];

	m_pucPIPColor=new GLubyte[IOTGUI_PIP_TRIBLESIZE];
	m_pfPIPVertex=new GLfloat[IOTGUI_PIP_TRIBLESIZE];


	m_pucSLAMColor=new GLubyte[IOTGUI_SLAM_TRIBLESIZE];
	m_pfSLAMPC=new GLfloat[IOTGUI_SLAM_TRIBLESIZE];



	memset(m_pucColor,0,IOTGUI_PC_TRIBLESIZE);
	memset(m_pfPC,0,IOTGUI_PC_TRIBLESIZE*4);

	memset(m_pucPIPColor,0,IOTGUI_PIP_TRIBLESIZE);
	memset(m_pfPIPVertex,0,IOTGUI_PIP_TRIBLESIZE*4);


	memset(m_pucSLAMColor,0,IOTGUI_SLAM_TRIBLESIZE);
	memset(m_pfSLAMPC,0,IOTGUI_SLAM_TRIBLESIZE*4);




	ThreadRenderID=0;

	idx=0;
	for (j=0;j<240;j++)
	{
		for (i=0;i<320;i++)
		{
			m_pfPIPVertex[idx]=X_RATE*i+0.37;
			idx++;
			m_pfPIPVertex[idx]=-Y_RATE*j-0.28;
			idx++;
			m_pfPIPVertex[idx]=0;
			idx++;
		}
	}


	for (i=0;i<CIRCLE_POINTS;i++)
	{

	}
	glClearColor(0,0,0,0);
	glShadeModel(GL_FLAT);


	m_dRotationX=0;
	m_dRotationY=0;
	m_dTranslationZ=0;
	m_dTranslationX=0;
	m_dTranslationY=0;

	m_bOpenPIP=true;
	m_MouseDownPoint.x=0;
	m_MouseDownPoint.y=0;
	m_nCurrentCount=0;

	m_bOpenStateLight=true;
	m_cRunState=0;
	m_bStart2AddNode=false;
	m_stGLPath.nCurNodeLen=0;
	m_bPathTwinkle=0;
	m_bShowPath=true;

	m_stGLPath.nCurNodeLen=0;
	m_stGLPath.pstPathHead=NULL;
	m_stGLPath.pstPathTail=NULL;
	m_stGLPath.pstPathCur=NULL;

	m_bNoIMUInfo=false;
	m_bUseIMU=false;
}

OpenGL::~OpenGL()
{

}
static HWND hWnd;
static	HGLRC hRC;		// Permanent Rendering Context
static	HDC hDC;		// Private GDI Device Context
BOOL	keys[256];		// Array Used For The Keyboard Routine
GLvoid OpenGL::InitGL(GLsizei Width, GLsizei Height)	// This Will Be Called Right After The GL Window Is Created
{
/*	glClearColor(0.0f, 0.0f, 0.0f, 0.0f);		// This Will Clear The Background Color To Black
	glClearDepth(1.0);							// Enables Clearing Of The Depth Buffer
	glDepthFunc(GL_LESS);						// The Type Of Depth Test To Do
	glEnable(GL_DEPTH_TEST);					// Enables Depth Testing
	glShadeModel(GL_SMOOTH);					// Enables Smooth Color Shading

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();							// Reset The Projection Matrix

	gluPerspective(45.0f,(GLfloat)Width/(GLfloat)Height,0.1f,100.0f);	// Calculate The Aspect Ratio Of The Window

	glMatrixMode(GL_MODELVIEW);*/



	glViewport (0, 0, (GLsizei) Width, (GLsizei) Height);
	glMatrixMode (GL_PROJECTION);
	glLoadIdentity ();
	//glFrustum(-0.6,0.6,-0.45,0.45,1,100);
	gluPerspective(40.0,(GLfloat)Width/(GLfloat)Height,1.0,100.0);
	glMatrixMode (GL_MODELVIEW);
	glLoadIdentity ();
	gluLookAt(0.0,0.0,-2,
		0.0,-0,0.0,
		0.0,-1.0,0.0);
}

GLvoid OpenGL::ReSizeGLScene(GLsizei Width, GLsizei Height)
{
	if (Height==0)								// Prevent A Divide By Zero If The Window Is Too Small
		Height=1;
	glViewport (0, 0, (GLsizei) Width, (GLsizei) Height);
	glMatrixMode (GL_PROJECTION);
	glLoadIdentity ();
	//glFrustum(-0.6,0.6,-0.45,0.45,1,100);
	gluPerspective(40.0,(GLfloat)Width/(GLfloat)Height,1.0,100.0);
	glMatrixMode (GL_MODELVIEW);
	glLoadIdentity ();
	//gluLookAt(0.0,0.0,-2,
	//	0.0,-0.2,0.0,
	//	0.0,-1.0,0.0);

	gluLookAt(0.0,0.0,-2,
		0.0,-0,0.0,
		0.0,-1.0,0.0);

	MoveWindow(hWnd,0,0,Width,Height,false);
}

GLvoid OpenGL::DrawGLScene(GLvoid)
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);		// Clear The Screen And The Depth Buffer
	glLoadIdentity();// Reset The View
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

	//	if (!m_bOpenPIP)
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

}

LRESULT CALLBACK OpenGL::WndProc(	HWND	hWnd,
						 UINT	message,
						 WPARAM	wParam,
						 LPARAM	lParam)
{
	RECT	Screen;							// Used Later On To Get The Size Of The Window
	GLuint	PixelFormat;
	CPoint MousePos;
	int kkk=10;
	short   zDelta;
	static	PIXELFORMATDESCRIPTOR pfd=
	{
		sizeof(PIXELFORMATDESCRIPTOR),		// Size Of This Pixel Format Descriptor
		1,									// Version Number (?)
		PFD_DRAW_TO_WINDOW |				// Format Must Support Window
		PFD_SUPPORT_OPENGL |				// Format Must Support OpenGL
		PFD_DOUBLEBUFFER,					// Must Support Double Buffering
		PFD_TYPE_RGBA,						// Request An RGBA Format
		16,									// Select A 16Bit Color Depth
		0, 0, 0, 0, 0, 0,					// Color Bits Ignored (?)
		0,									// No Alpha Buffer
		0,									// Shift Bit Ignored (?)
		0,									// No Accumulation Buffer
		0, 0, 0, 0,							// Accumulation Bits Ignored (?)
		16,									// 16Bit Z-Buffer (Depth Buffer)  
		0,									// No Stencil Buffer
		0,									// No Auxiliary Buffer (?)
		PFD_MAIN_PLANE,						// Main Drawing Layer
		0,									// Reserved (?)
		0, 0, 0								// Layer Masks Ignored (?)
	};

	switch (message)						// Tells Windows We Want To Check The Message
	{
	case WM_CREATE:
		hDC = GetDC(hWnd);				// Gets A Device Context For The Window
		PixelFormat = ChoosePixelFormat(hDC, &pfd);		// Finds The Closest Match To The Pixel Format We Set Above

		if (!PixelFormat)
		{
		//	MessageBox(0,"Can't Find A Suitable PixelFormat.","Error",MB_OK|MB_ICONERROR);
			PostQuitMessage(0);			// This Sends A 'Message' Telling The Program To Quit
			break;						// Prevents The Rest Of The Code From Running
		}

		if(!SetPixelFormat(hDC,PixelFormat,&pfd))
		{
		//	MessageBox(0,"Can't Set The PixelFormat.","Error",MB_OK|MB_ICONERROR);
			PostQuitMessage(0);
			break;
		}

		hRC = wglCreateContext(hDC);
		if(!hRC)
		{
			//MessageBox(0,"Can't Create A GL Rendering Context.","Error",MB_OK|MB_ICONERROR);
			PostQuitMessage(0);
			break;
		}

		if(!wglMakeCurrent(hDC, hRC))
		{
			//MessageBox(0,"Can't activate GLRC.","Error",MB_OK|MB_ICONERROR);
			PostQuitMessage(0);
			break;
		}

		GetClientRect(hWnd, &Screen);
		InitGL(Screen.right, Screen.bottom);
		break;

	case WM_DESTROY:
	case WM_CLOSE:
		ChangeDisplaySettings(NULL, 0);

		wglMakeCurrent(hDC,NULL);
		wglDeleteContext(hRC);
		ReleaseDC(hWnd,hDC);

		PostQuitMessage(0);
		break;

	case WM_KEYDOWN:
		keys[wParam] = TRUE;
		OnKeyDown(wParam,0,0);
		break;
	case WM_CHAR:
		keys[wParam] = TRUE;
		break;

	case WM_KEYUP:
		keys[wParam] = FALSE;
		break;

	case WM_SIZE:
		ReSizeGLScene(LOWORD(lParam),HIWORD(lParam));
		break;

	case WM_LBUTTONDOWN:
		MousePos.x= LOWORD(lParam);
		MousePos.y= HIWORD(lParam);
		OnLButtonDown(0,MousePos);
		break;
	case WM_MOUSEMOVE:
		MousePos.x= LOWORD(lParam);
		MousePos.y= HIWORD(lParam);
		OnMouseMove(0,MousePos);
		break;

	case WM_LBUTTONUP:
		MousePos.x= LOWORD(lParam);
		MousePos.y= HIWORD(lParam);
		OnLButtonUp(0,MousePos);
		break;

	case WM_RBUTTONDOWN:
		MousePos.x= LOWORD(lParam);
		MousePos.y= HIWORD(lParam);
		OnRButtonDown(0,MousePos);
		break;

	case WM_RBUTTONUP:
		MousePos.x= LOWORD(lParam);
		MousePos.y= HIWORD(lParam);
		OnRButtonUp(0,MousePos);
		break;

	case WM_MOUSEWHEEL:
		zDelta   =   (short)   HIWORD(wParam);  
		MousePos.x= LOWORD(lParam);
		MousePos.y= HIWORD(lParam);
		OnMouseWheel(0,zDelta,MousePos);
		break;



	default:
		return (DefWindowProc(hWnd, message, wParam, lParam));
	}
	return (0);
}

//int WINAPI WinMain(	HINSTANCE	hInstance, 
//				   HINSTANCE	hPrevInstance, 
//				   LPSTR		lpCmdLine, 
//				   int			nCmdShow)
int OpenGL::OpenlInterface(HWND hParent,HINSTANCE	hInstance)
{
	MSG			msg;		// Windows Message Structure
	WNDCLASS	wc;			// Windows Class Structure Used To Set Up The Type Of Window
//	HWND		hWnd;		// Storage For Window Handle

	wc.style			= CS_HREDRAW | CS_VREDRAW | CS_OWNDC;
	wc.lpfnWndProc		= (WNDPROC) WndProc;
	wc.cbClsExtra		= 0;
	wc.cbWndExtra		= 0;
	wc.hInstance		= hInstance;
	wc.hIcon			= NULL;
	wc.hCursor			= LoadCursor(NULL, IDC_ARROW);
	wc.hbrBackground	= NULL;
	wc.lpszMenuName		= NULL;
	wc.lpszClassName	= "OpenGL WinClass";

	if(!RegisterClass(&wc))
	{
		//MessageBox(0,"Failed To Register The Window Class.","Error",MB_OK|MB_ICONERROR);
		return FALSE;
	}

	hWnd = CreateWindow(
		"OpenGL WinClass",
		"Jeff Molofee's GL Code Tutorial ... NeHe '99",		// Title Appearing At The Top Of The Window

		WS_CHILD |
		WS_CLIPCHILDREN |
		WS_CLIPSIBLINGS,

		0, 0,												// The Position Of The Window On The Screen
		640, 480,											// The Width And Height Of The WIndow

		hParent,
		NULL,
		hInstance,
		NULL);

	if(!hWnd)
	{
		///MessageBox(0,"Window Creation Error.","Error",MB_OK|MB_ICONERROR);
		return FALSE;
	}
	ShowWindow(hWnd, SW_SHOW);
	UpdateWindow(hWnd);
	SetFocus(hWnd);
	wglMakeCurrent(hDC,hRC);



	while (1)
	{
		// Process All Messages
		while (PeekMessage(&msg, NULL, 0, 0, PM_NOREMOVE))
		{
			if (GetMessage(&msg, NULL, 0, 0))
			{
				TranslateMessage(&msg);
				DispatchMessage(&msg);
			}
			else
			{
				return TRUE;
			}
		}
		//DrawGLScene();
		//RenderScene();
		RenderSLAM();
		//DrawPath();
		SwapBuffers(hDC);

		if (keys[VK_ESCAPE]) SendMessage(hWnd,WM_CLOSE,0,0);
	}
}

/*
void OpenGL::DrawPath()
{
	int i;
	glLoadIdentity ();
	gluLookAt(0.0,0.0,-2,
		0.0,-0.0,0.0,
		0.0,-1.0,0.0);
	glClear(GL_COLOR_BUFFER_BIT);

	glPushMatrix();
	glColor3f(1.0,0.0,0.0);
	glutSolidSphere(0.01,10,10);
	glTranslated(0.1,0,0);
	glColor3f(0.0,1.0,0.0);
	glutSolidSphere(0.01,10,10);


	glBegin(GL_LINES);
	glColor3f(1.0,1.0,1.0);
	glVertex3f(0.0,0,0);
	glVertex3f(0.1,0,0);
	glEnd();

	//glScalef(2,1,1);
	//glutSolidCube(0.05);

	glPopMatrix();
	glFlush();

}*/

void OpenGL::RenderScene()
{
	int i;
	glLoadIdentity ();
	gluLookAt(0.0,0.0,-2,
		0.0,-0.0,0.0,
		0.0,-1.0,0.0);


	glClear(GL_COLOR_BUFFER_BIT);
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



	if (m_bShowPath)
	{
		DrawPath();

		//if (m_bPathTwinkle)
		{
			PathTwinkle();
		}
	}


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
	glFlush();
	Sleep(10);

}

void OpenGL::OnKeyDown(UINT nChar, UINT nRepCnt, UINT nFlags) 
{
	// TODO: Add your message handler code here and/or call default
	char tmp=(char)nChar;
	switch (tmp)
	{
	case'W':
		m_dRotationX -= 3;
		//glutPostRedisplay();
		break;
	case 'S':
		m_dRotationX += 3;
		//glutPostRedisplay();
		break;
	case 'A':
		m_dRotationY -= 3;
		//glutPostRedisplay();
		break;
	case 'D':
		m_dRotationY += 3;
		//glutPostRedisplay();
		break;

	case 'Q':
		m_dRotationX -= 3;
		m_dRotationY -= 3;
		//glutPostRedisplay();
		break;

	case 'E':
		m_dRotationX -= 3;
		m_dRotationY += 3;
		//glutPostRedisplay();
		break;


	case 'Z':
		m_dRotationY -= 3;
		m_dRotationX += 3;
		//glutPostRedisplay();
		break;

	case 'C':
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

	case 'P':
		m_bOpenPIP=(m_bOpenPIP?false:true);
		//glutPostRedisplay();
		break;

	case 'L':
		m_bOpenStateLight=(m_bOpenStateLight?false:true);
		//glutPostRedisplay();
		break;
	case'O':
		m_bPathTwinkle=(m_bPathTwinkle?false:true);
		//m_stGLPath.pstPathCur=m_stGLPath.pstPathHead;
		//memcpy(m_stPathTwinkle.fLastPos,m_stGLPath.pstPathCur->pstSonNode,12);
		//memcpy(m_stPathTwinkle.fCurPos,m_stGLPath.pstPathCur->pstSonNode,12);
		m_stPathTwinkle.ulStartTime=0;
		break;

	case 'I':
		m_bShowPath=(m_bShowPath?false:true);
		break;

	default:
		break;
	}        
}

HWND g_hCapture;
int g_nLBD=0;
void OpenGL::OnLButtonDown(UINT nFlags, CPoint point) 
{
	// TODO: Add your message handler code here and/or call default
	m_MouseDownPoint=point;
	m_enumMouseDrag=Rotate;
	if(g_nLBD==0)g_nLBD=1;
	//g_hCapture=SetCapture(hWnd);
}




void OpenGL::RenderSLAM()
{
	//printf("#$%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%\n");
	int i;
	glLoadIdentity ();
	gluLookAt(0.0,0.0,-2,
		0.0,-0.0,0.0,
		0.0,-1.0,0.0);


	glClear(GL_COLOR_BUFFER_BIT);
	glEnableClientState(GL_COLOR_ARRAY);
	glEnableClientState(GL_VERTEX_ARRAY);
	glColorPointer(3,GL_UNSIGNED_BYTE,0,m_pucSLAMColor);
	glVertexPointer(3,GL_FLOAT,0,m_pfSLAMPC);


	glPushMatrix();

	glTranslated(m_dTranslationX, m_dTranslationY, m_dTranslationZ);
	glRotated(m_dRotationX, 1, 0, 0);
	glRotated(m_dRotationY, 0, 1, 0);

	glBegin(GL_POINTS);
	for (i=0;i<m_nCurrentCount;i++)
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



	
	if (m_bShowPath)
	{
		DrawPath();

	//	if (m_bPathTwinkle)
		{
			PathTwinkle();
		}
	}


	glPopMatrix();


	if (m_bOpenPIP)
	{
		glColorPointer(3,GL_UNSIGNED_BYTE,0,m_pucPIPColor);
		glVertexPointer(3,GL_FLOAT,0,m_pfPIPVertex);
		glBegin(GL_POINTS);


		for (i=0;i<IOTGUI_PIP_SIZE;i++)
		{
			glArrayElement(i);
		}

		glEnd();
	}

	if (m_bOpenStateLight)
	{
		if(m_cRunState==0) glColor3f (0.0, 1.0, 0.0);
		else if(m_cRunState==-1) glColor3f (0.0, 1.0, 0.0);
		else if(m_cRunState==-2) glColor3f (0.0, 0.0, 1.0);
		else if(m_cRunState==-3) glColor3f (1.0, 0.0, 0.0);

			
		glBegin(GL_POLYGON);

		glVertex3f(-0.95,-0.7,0);
		glVertex3f(-0.95,-0.6,0);
		glVertex3f(-0.85,-0.6,0);
		glVertex3f(-0.85,-0.7,0);
		glEnd();
	}


//	printf("m_bNoIMUInfo$$$$$$$$$$$$$$$  :%d  \n",m_bNoIMUInfo);
	if(m_bNoIMUInfo)
	{

		glColor3f (1.0, 1.0, 1.0);
		glBegin(GL_LINE_STRIP );
		glVertex3f(-0.94,-0.6,0);
		glVertex3f(-0.92,-0.7,0);
		glVertex3f(-0.9,-0.63,0);
		glVertex3f(-0.87,-0.7,0);
		glVertex3f(-0.86,-0.6,0);
		glEnd();

		glColor3f (1.0, 1.0, 1.0);
		glBegin(GL_LINES);
		glVertex3f(-0.95,-0.64,0);
		glVertex3f(-0.85,-0.66,0);
		glEnd();
	}
	else
	{
		if (m_bUseIMU)
		{
			glColor3f (1.0, 1.0, 1.0);
			glBegin(GL_LINE_STRIP );
			glVertex3f(-0.94,-0.6,0);
			glVertex3f(-0.92,-0.7,0);
			glVertex3f(-0.9,-0.63,0);
			glVertex3f(-0.87,-0.7,0);
			glVertex3f(-0.86,-0.6,0);
			glEnd();
		}
	}
	



	glFlush();
	Sleep(10);

}



void OpenGL::OnLButtonUp(UINT nFlags, CPoint point) 
{
	// TODO: Add your message handler code here and/or call default
	m_MouseDownPoint=CPoint(0,0);
	ReleaseCapture();
	g_hCapture=0;
	g_nLBD=0;
}


void OpenGL::OnRButtonDown(UINT nFlags, CPoint point) 
{
	// TODO: Add your message handler code here and/or call default
	m_MouseDownPoint=point;
	m_enumMouseDrag=Translate;
	if(g_nLBD==0)g_nLBD=2;
}

void OpenGL::OnRButtonUp(UINT nFlags, CPoint point) 
{
	// TODO: Add your message handler code here and/or call default
	m_MouseDownPoint=CPoint(0,0);
	g_nLBD=0;
}

void OpenGL::OnMouseMove(UINT nFlags, CPoint point) 
{
	//if (GetCapture()==g_hCapture)
	GLdouble dx = point.x- m_MouseDownPoint.x;
	GLdouble dy = point.y - m_MouseDownPoint.y;
	if(g_nLBD==1)
	{
		m_dRotationY += 360 * dx / IOTGUI_PC_WIDTH;
		m_dRotationX += 360 * dy / IOTGUI_PC_HEIGHT;
	}
	else if (g_nLBD==2)
	{
		m_dTranslationX += 10 * dx / IOTGUI_PC_WIDTH;
		m_dTranslationY += 10 * dy / IOTGUI_PC_HEIGHT;
	}
	m_MouseDownPoint=point;

}

BOOL  OpenGL:: OnMouseWheel(   UINT   nFlags,   short   zDelta,   CPoint   pt   )
{
	if (zDelta>0)
	{
		m_dTranslationZ += 1;
	}
	else if(zDelta<0)
	{
		m_dTranslationZ-=1;
	}
	return TRUE;
}



void OpenGL::CallBack_PIPQVGA(unsigned char *pucQVGAImg,void *pContext)
{

	memcpy(m_pucPIPColor,pucQVGAImg,IOTGUI_PIP_TRIBLESIZE);
}
void OpenGL::CallBack_PointCloud(unsigned char *pucImg,float *pfVertex,void *pContext)
{

	memcpy(m_pucColor,pucImg,IOTGUI_PC_TRIBLESIZE);
	memcpy(m_pfPC,pfVertex,(IOTGUI_PC_TRIBLESIZE*4));
}


void OpenGL::CallBack_RunState(char cState)
{

	//printf("CallBack_RunState :     %d  \n",cState);
	if (cState==1)
	{
		//printf("opengl  hhahahahahhahah  \n");
		m_bNoIMUInfo=true;
	}
	else if (cState==2)
	{
		//printf("opengl  hhahahahahhahah  \n");
		m_bNoIMUInfo=0;
	}
	else m_cRunState=cState;
}

void OpenGL::CallBack_SLAM_PC(unsigned char *pucImg,float *pfVertex,void *pContext,int nCount)
{
	//printf("hahahahhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhh\n");
	if (m_nCurrentCount+nCount<IOTGUI_SLAM_SIZE)
	{
		memcpy(m_pucSLAMColor+m_nCurrentCount,pucImg,nCount);
		memcpy(m_pfSLAMPC+m_nCurrentCount,pfVertex,nCount*4);
		m_nCurrentCount+=nCount;
	}
	else
	{
		printf("Exceed Max PC Storage \n");
	}
}


int OpenGL::CreatePath()
{
	m_stGLPath.nCurNodeLen=0;
	m_stGLPath.pstPathHead=new IoTRobot_GLPathNode;
	m_stGLPath.pstPathHead->pstSonNode=NULL;
	m_stGLPath.pstPathHead->pstParentNode=NULL;


	m_stGLPath.pstPathCur=m_stGLPath.pstPathHead;
	m_stGLPath.pstPathTail=m_stGLPath.pstPathHead;
	return 0;
}

int OpenGL::AddPathNode(float *pfTMat,float *pfRAngle)
{
	critical_section.Lock();
	if (m_stGLPath.nCurNodeLen==0)
	{
		CreatePath();
		memcpy(m_stGLPath.pstPathTail->fRAngle,pfRAngle,12);
		memcpy(m_stGLPath.pstPathTail->fTMat,pfTMat,12);
		m_stGLPath.nCurNodeLen=1;
	}
	else
	{
		IoTRobot_GLPathNode *pstNode=new IoTRobot_GLPathNode;
		memcpy(pstNode->fRAngle,pfRAngle,12);
		memcpy(pstNode->fTMat,pfTMat,12);
		pstNode->nIdx=m_stGLPath.pstPathTail->nIdx+1;
		m_stGLPath.pstPathTail->pstSonNode=pstNode;
		pstNode->pstParentNode=m_stGLPath.pstPathTail;
		pstNode->pstSonNode=NULL;
		m_stGLPath.pstPathTail=pstNode;
		m_stGLPath.nCurNodeLen++;
		if (m_stGLPath.nCurNodeLen>MAX_PATH_NODE_LEN)
		{
			IoTRobot_GLPathNode *pTmp=m_stGLPath.pstPathHead;
			m_stGLPath.pstPathHead=m_stGLPath.pstPathHead->pstSonNode;
			m_stGLPath.pstPathHead->pstParentNode=NULL;
			delete pTmp;
			m_stGLPath.nCurNodeLen=MAX_PATH_NODE_LEN;
		}
	}
	critical_section.Unlock();
	return 0;
}


void OpenGL::CallBack_Path(float *pfTMat,float *pfRAngle,void *pContext)
{
	AddPathNode(pfTMat,pfRAngle);
}


int OpenGL::PathTwinkle()
{
	if (m_stGLPath.pstPathHead!=NULL)
	{
		DWORD ulCurTime=::GetTickCount();
		if (ulCurTime-m_stPathTwinkle.ulStartTime<MAX_TWINKLE_TIME)
		{
			if (m_stPathTwinkle.nColorCount<MAX_TWINKLE_INTERVAL)
			{
				glPushMatrix();
				glGREEN;
				glTranslated(m_stPathTwinkle.fLastPos[0], m_stPathTwinkle.fLastPos[1], m_stPathTwinkle.fLastPos[2]);
				glutSolidSphere(0.007,10,10);
				glPopMatrix();


				glPushMatrix();
				glGREEN;
				glTranslated(m_stPathTwinkle.fCurPos[0], m_stPathTwinkle.fCurPos[1], m_stPathTwinkle.fCurPos[2]);
				glutSolidSphere(0.007,10,10);
				glPopMatrix();

				glBegin(GL_LINES);
				glPINK;
				glVertex3f(m_stPathTwinkle.fCurPos[0],m_stPathTwinkle.fCurPos[1],m_stPathTwinkle.fCurPos[2]);
				glVertex3f(m_stPathTwinkle.fLastPos[0],m_stPathTwinkle.fLastPos[1],m_stPathTwinkle.fLastPos[2]);
				glEnd();
				m_stPathTwinkle.nColorCount++;
			}
			else
			{
				m_stPathTwinkle.nColorCount++;
				if (m_stPathTwinkle.nColorCount==MAX_TWINKLE_INTERVAL*2)m_stPathTwinkle.nColorCount=0;
			}
		}
		else
		{
			m_stPathTwinkle.ulStartTime=ulCurTime;
			if (m_stGLPath.pstPathCur!=NULL)
			{
				memcpy(m_stPathTwinkle.fLastPos,m_stPathTwinkle.fCurPos,12);
				memcpy(m_stPathTwinkle.fCurPos,m_stGLPath.pstPathCur->fTMat,12);
				m_stPathTwinkle.nColorCount=0;
			}
			else
			{
				m_stGLPath.pstPathCur=m_stGLPath.pstPathHead;
				memcpy(m_stPathTwinkle.fLastPos,m_stGLPath.pstPathCur->fTMat,12);
				memcpy(m_stPathTwinkle.fCurPos,m_stGLPath.pstPathCur->fTMat,12);
				m_stPathTwinkle.nColorCount=0;
			}
			m_stGLPath.pstPathCur=m_stGLPath.pstPathCur->pstSonNode;

		}
	}
	
		
	
	return 0;
}




int OpenGL::DrawPath()
{
	int i;
	
	IoTRobot_GLPathNode *pstPathNode=m_stGLPath.pstPathHead;
	float fLastNodePos[3];
	critical_section.Lock();


	if (m_stGLPath.pstPathHead!=NULL)
	{
		memcpy(fLastNodePos,m_stGLPath.pstPathHead->fTMat,12);
		while (pstPathNode!=NULL)
		{
			glPushMatrix();
			glTranslated(pstPathNode->fTMat[0], pstPathNode->fTMat[1], pstPathNode->fTMat[2]);
			glColor3f(1.0,0.0,0.0);
			glutSolidSphere(0.007,10,10);





			//coordinate
			glBegin(GL_LINES);
			glLineWidth(5);
			glColor3f(1.0,0.0,0.0);
			glVertex3f(0.0,0,0);
			glVertex3f(0.05,0,0);

			glColor3f(0.0,1.0,0.0);
			glVertex3f(0.0,0,0);
			glVertex3f(0.0,0.05,0);

			glColor3f(0.0,0.0,1.0);
			glVertex3f(0.0,0,0);
			glVertex3f(0.0,0,0.05);
			glEnd();


			glPopMatrix();

			glPushMatrix();
			//path node edge
			glBegin(GL_LINES);
			glColor3f(1.0,1.0,1.0);
			glVertex3f(pstPathNode->fTMat[0],pstPathNode->fTMat[1],pstPathNode->fTMat[2]);
			glVertex3f(fLastNodePos[0],fLastNodePos[1],fLastNodePos[2]);
			glEnd();
			glPopMatrix();

			memcpy(fLastNodePos,pstPathNode->fTMat,12);
			pstPathNode=pstPathNode->pstSonNode;
		}
	}
	
	critical_section.Unlock();
	return true;
	
}
