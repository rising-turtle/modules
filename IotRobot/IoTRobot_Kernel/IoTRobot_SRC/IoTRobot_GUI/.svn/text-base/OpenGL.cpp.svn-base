//
// This code was created by Jeff Molofee '99
//
// If you've found this code useful, please let me know.
//
// Visit me at www.demonews.com/hosted/nehe
//

// decide whether to use Triangles!!!
//#define TRIANGLE

//#include <gl\glaux.h>	// Header File For The GLaux Library
#include "stdafx.h"
#include "OpenGL.h"
#include "afxmt.h"
#include <math.h>

#define glRED glColor3f(1.0,0,0)
#define glGREEN glColor3f(0,1.0,0)
#define glBLUE glColor3f(0,0,1.0)
#define glWHITE glColor3f(1.0,1.0,1.0)
#define glPINK glColor3f(1.0,0,1.0)

//float * OpenGL::m_pfPIP_CIFVertex;
//unsigned char * OpenGL::m_pucPIP_CIFColor;
//float * OpenGL::m_pfPIPVertex;
//unsigned char * OpenGL::m_pucPIPColor;
float * OpenGL::m_pfPC;
unsigned char * OpenGL::m_pucColor;

GLdouble OpenGL::m_dRotationX;
GLdouble OpenGL::m_dRotationY;
GLdouble OpenGL::m_dTranslationX ;
GLdouble OpenGL::m_dTranslationY ;
GLdouble OpenGL::m_dTranslationZ ;

//bool  OpenGL::m_bOpenPIP;
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
bool OpenGL::m_bCompass;
bool OpenGL::m_bTriangulation;
bool OpenGL::m_bStopOpenGL;

float OpenGL::m_fScreenTopX;
float OpenGL::m_fScreenTopY;
float OpenGL::m_fCircle[60];
char OpenGL::m_cNodeNum[200];
double OpenGL::m_dChassisPose[3];
int OpenGL::m_nFirstPoseRcd;
double OpenGL::m_dFirstPose[3];
double OpenGL::m_dKinectPose[6];

int OpenGL::m_nManualPath;
int OpenGL::m_nManualPathArray[10000];
int OpenGL::m_nManualPathArrayLen;
int OpenGL::m_nSceenWidth;
int OpenGL::m_nSceenHeight;

int OpenGL::m_nRobotPathArray[300];
int OpenGL::m_nRobotPathArrayLen;
float OpenGL::m_fMileStonePos[300];
int OpenGL::m_nMileStoneLen;

int OpenGL::m_nInflectionPointsIdxArray[300];
int OpenGL::m_nInflectionPointsNum;


float OpenGL::m_fPathArray[10000];
float OpenGL::m_fThres;
float OpenGL::m_cMileStoneFlag[100];
int OpenGL::m_nMileStoneHasBeenBuild;
int OpenGL::m_nReachMileStoneIdx;
int OpenGL::m_nHorizonCutPC;
int OpenGL::m_nDrawVitualLine;

int OpenGL::m_nRealPathPointNum;
float OpenGL::m_fRealPathPos[200];
float OpenGL::m_fRealPathFlag[100];

char OpenGL::m_cScale[200];

void *OpenGL::m_pOpenGL;



std::vector<GLuint> OpenGL::m_call_list;
int OpenGL::m_index_of_call_list=1;
GLubyte* OpenGL::m_PlaneColor;
GLfloat* OpenGL::m_PlaneVertex;
int OpenGL::m_nPlanes;

float OpenGL::m_fGridLinesPos[18][6];
float OpenGL::m_fScaleGridPos[4][3];
OpenGL::OpenGL()
{

	m_pOpenGL=(void*)this;
	int i,j,idx;
	m_enumMouseDrag=None;
	m_pucColor=new GLubyte[IOTGUI_PC_TRIBLESIZE];
	m_pfPC=new GLfloat[IOTGUI_PC_TRIBLESIZE];

	//m_pucPIPColor=new GLubyte[IOTGUI_PIP_TRIBLESIZE];
	//m_pfPIPVertex=new GLfloat[IOTGUI_PIP_TRIBLESIZE];

	//m_pucPIP_CIFColor=new GLubyte[IOTGUI_PIP_TRIBLESIZE_CIF];
	//m_pfPIP_CIFVertex=new GLfloat[IOTGUI_PIP_TRIBLESIZE_CIF];

	m_pucSLAMColor=new GLubyte[IOTGUI_SLAM_TRIBLESIZE];
	m_pfSLAMPC=new GLfloat[IOTGUI_SLAM_TRIBLESIZE];



	memset(m_pucColor,0,IOTGUI_PC_TRIBLESIZE);
	memset(m_pfPC,0,IOTGUI_PC_TRIBLESIZE*4);

	//	memset(m_pucPIPColor,0,IOTGUI_PIP_TRIBLESIZE);
	//	memset(m_pfPIPVertex,0,IOTGUI_PIP_TRIBLESIZE*4);


	memset(m_pucSLAMColor,0,IOTGUI_SLAM_TRIBLESIZE);
	memset(m_pfSLAMPC,0,IOTGUI_SLAM_TRIBLESIZE*4);

	memset(m_dChassisPose,0,48);


	ThreadRenderID=0;

	//glVertex3f(R*cos(2*Pi/n*i),0, R*sin(2*Pi/n*i));
	for (i=0;i<20;i++)
	{
		m_fCircle[i*3]=0.2*cos(2*3.1415926/20*i);
		m_fCircle[i*3+1]=0.3;
		m_fCircle[i*3+2]=0.2*sin(2*3.1415926/20*i);
	}
	

	glClearColor(0,0,0,0);
	glShadeModel(GL_FLAT);


	m_dRotationX=0;
	m_dRotationY=0;
	m_dTranslationZ=0;
	m_dTranslationX=0;
	m_dTranslationY=0;

	//	m_bOpenPIP=true;
	m_MouseDownPoint.x=0;
	m_MouseDownPoint.y=0;
	m_nCurrentCount=0;

	m_bOpenStateLight=true;
	m_cRunState=0;
	m_bStart2AddNode=false;
	m_stGLPath.nCurNodeLen=0;
	m_bPathTwinkle=0;
	m_bShowPath=false;

	m_stGLPath.nCurNodeLen=0;
	m_stGLPath.pstPathHead=NULL;
	m_stGLPath.pstPathTail=NULL;
	m_stGLPath.pstPathCur=NULL;

	m_bNoIMUInfo=false;
	m_bUseIMU=false;
	m_bCompass=false;
	m_bTriangulation=true;
	m_bStopOpenGL=true;

	memset(m_cNodeNum,0,200);
	m_nFirstPoseRcd=0;
	m_nHorizonCutPC=0;
	m_nDrawVitualLine=0;



	// Generate display list from index=1
	m_PlaneColor=new GLubyte[1024*1024*3];
	m_PlaneVertex=new GLfloat[1024*1024*3];

	memset(m_PlaneColor,0,1024*1024*3);
	memset(m_PlaneVertex,0,1024*1024*3*4);
	m_nPlanes=0;
	m_index_of_call_list=129;
}

OpenGL::~OpenGL()
{
	if (m_pucColor!=NULL)
	{
		delete []m_pucColor;
	}

	if (m_pfPC!=NULL)
	{
		delete [] m_pfPC;
	}
	

	if (m_pucSLAMColor!=NULL)
	{
		delete [] m_pucSLAMColor;
	}

	if (m_pfSLAMPC!=NULL)
	{
		delete [] m_pfSLAMPC;
	}

	if (m_PlaneColor!=0)
	{
		delete [] m_PlaneColor;
	}

	if (m_PlaneVertex!=0)
	{
		delete []m_PlaneVertex;
	}
}
static HWND hWnd;
static	HGLRC hRC;		// Permanent Rendering Context
static	HDC hDC;		// Private GDI Device Context
BOOL	keys[256];		// Array Used For The Keyboard Routine
GLvoid OpenGL::InitGL(GLsizei Width, GLsizei Height)	// This Will Be Called Right After The GL Window Is Created
{
	

	m_nSceenWidth=Width;
	m_nSceenHeight=Height;

	glutInitDisplayMode(GLUT_DEPTH);
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
	m_fScreenTopY=-0.60;
	m_fScreenTopX=m_fScreenTopY*Width/Height;
}

GLvoid OpenGL::ReSizeGLScene(GLsizei Width, GLsizei Height)
{
	if (Height==0)								// Prevent A Divide By Zero If The Window Is Too Small
		Height=1;

	m_nSceenWidth=Width;
	m_nSceenHeight=Height;
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

	m_fScreenTopY=-0.60;
	m_fScreenTopX=m_fScreenTopY*Width/Height;
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
	case WM_LBUTTONDBLCLK:
		MousePos.x= LOWORD(lParam);
		MousePos.y= HIWORD(lParam);
		OnLButtonDoubleClieck(0,MousePos);
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

	m_bStopOpenGL=false;
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



	while (!m_bStopOpenGL)
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
		//	RenderCharactors();
		//DrawGLScene();
		//RenderScene();
		RenderSLAM();
		//DrawPath();
		//DrawRect();
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
	glLineWidth(5);
	glBegin(GL_LINES);
	
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

	//	if (m_bOpenPIP)
	//{
	/*	glColorPointer(3,GL_UNSIGNED_BYTE,0,m_pucPIPColor);
	glVertexPointer(3,GL_FLOAT,0,m_pfPIPVertex);
	glBegin(GL_POINTS);


	for (i=0;i<320*240;i++)
	{
	glArrayElement(i);
	}

	glEnd();*/

	//	glColorPointer(3,GL_UNSIGNED_BYTE,0,m_pucPIP_CIFColor);
	//	glVertexPointer(3,GL_FLOAT,0,m_pfPIP_CIFVertex);
	//	glBegin(GL_POINTS);


	//	for (i=0;i<352*288;i++)
	//	{
	//		glArrayElement(i);
	//	}

	//	glEnd();
	//}
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

		//	case 'P':
		//	m_bOpenPIP=(m_bOpenPIP?false:true);
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
	if (m_nManualPath)
	{
		m_nManualPathArray[m_nManualPathArrayLen]=point.x;
		m_nManualPathArrayLen++;
		m_nManualPathArray[m_nManualPathArrayLen]=point.y;
		m_nManualPathArrayLen++;

		memset(m_fRealPathFlag,0,100*4);
		m_nDrawVitualLine=0;
		m_nRealPathPointNum=0;
		//DrawRobotPath();
	}

	//g_hCapture=SetCapture(hWnd);
}



void OpenGL::RenderCharactors()
{
	selectFont(48, ANSI_CHARSET, "Comic Sans MS");
	glClear(GL_COLOR_BUFFER_BIT);
	glColor3f(1.0f, 0.0f, 0.0f);
	glRasterPos2f(-1.0f, 0.0f);
	drawString("Hello, World!");
}

int g_nMileStoneFlagNum=0;
void OpenGL::RenderSLAM()
{
	//printf("#$%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%\n");


	int i;
	
	glLoadIdentity ();
	gluLookAt(0.0,0.0,-2,
		0.0,-0.0,0.0,
		0.0,-1.0,0.0);
	glClear(GL_COLOR_BUFFER_BIT);


	if (m_bOpenStateLight)
	{
		if (m_cRunState==0||m_cRunState==-1)
		{
			glColor3f(0.0f, 1.0f, 0.0f);
		}
		else
		{
			glColor3f(1.0f, 0.0f, 0.0f);
		}
		selectFont(50, ANSI_CHARSET, "Arial Black");
		//glClear(GL_COLOR_BUFFER_BIT);
		glRasterPos2f(m_fScreenTopX, m_fScreenTopY);
		drawString(m_cNodeNum);
	}





	glEnableClientState(GL_COLOR_ARRAY);
	glEnableClientState(GL_VERTEX_ARRAY);
	glColorPointer(3,GL_UNSIGNED_BYTE,0,m_pucSLAMColor);
	glVertexPointer(3,GL_FLOAT,0,m_pfSLAMPC);

	if (m_bCompass)
	{
		glPushMatrix();
		glRotated(m_dRotationX, 1, 0, 0);
		glRotated(m_dRotationY, 0, 1, 0);
		glTranslated(m_dTranslationX, m_dTranslationY, 0);

		DrawArrow();
		glPopMatrix();
	}


	glPushMatrix();



	glTranslated(m_dTranslationX, m_dTranslationY, m_dTranslationZ);
	glRotated(m_dRotationX, 1, 0, 0);
	glRotated(m_dRotationY, 0, 1, 0);



	//draw fllor
/*	glColor3f(0.1,0.1,0.1);
	glBegin(GL_POLYGON);
	glNormal3f(0,-1,0);
	glVertex3f(-10,KINECT_HEIGHT,-10);
	glVertex3f(-10,KINECT_HEIGHT,10);
	glVertex3f(10,KINECT_HEIGHT,10);
	glVertex3f(10,KINECT_HEIGHT,-10);
	glEnd();*/

	if (m_bTriangulation)
	{
		glBegin(GL_TRIANGLES);
	}
	else
	{
		glBegin(GL_POINTS);
	}
	for (i=0;i<m_nCurrentCount;i++)
	{
		glArrayElement(i);
	}
	glEnd();




	glLineWidth( 3 );//设置线的宽度为6
	glBegin(GL_LINES);
	
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

	DrawCurrentPose();
	DrawMileStone();


	if (m_nDrawVitualLine)
	{
		float fCurPos[3];
		if (m_nRealPathPointNum>0)
		{
			DrawReallPath();
			fCurPos[0]=m_fRealPathPos[(m_nRealPathPointNum-1)*2];
			fCurPos[1]=KINECT_HEIGHT;
			fCurPos[2]=m_fRealPathPos[(m_nRealPathPointNum-1)*2+1];
		}
		else
		{
			fCurPos[0]=m_dKinectPose[0];
			fCurPos[1]=m_dKinectPose[1];
			fCurPos[2]=m_dKinectPose[2];
		}
		DrawVitualPath(m_fMileStonePos,fCurPos);
	}



	// Render Planes!!
//	if (g_nLBD==0)
	{
	
		if(m_nPlanes>0)
		{
			// create ShowList
			GLuint cloud_list_index = glGenLists(1);
			if(cloud_list_index>0) 
			{
				glNewList(cloud_list_index, GL_COMPILE);
				//m_index_of_call_list++;
				m_call_list.push_back(cloud_list_index);

				GLubyte* pColor=m_PlaneColor;
				GLfloat* pfVertex=m_PlaneVertex;
				//// display these nCount planes
				//for(int i=0;i<m_nPlanes;i++)
				//{
				//	glBegin(GL_POLYGON);
				//	glColor3ub(*pColor,*(pColor+1),*(pColor+2));
				//	pColor+=3;
				//	float nVertex=0;
				//	memcpy(&nVertex,pfVertex,sizeof(float));
				//	int nNumV=(int)nVertex;
				//	pfVertex++;
				//	for(int j=0;j<nNumV;j++)
				//	{
				//		glVertex3f(*pfVertex,*(pfVertex+1),*(pfVertex+2));
				//		pfVertex+=3;
				//	}
				//	glEnd();
				//}
				glEnableClientState(GL_COLOR_ARRAY);
				glEnableClientState(GL_VERTEX_ARRAY);
				glColorPointer(3,GL_UNSIGNED_BYTE,0,m_PlaneColor);
				glVertexPointer(3,GL_FLOAT,0,m_PlaneVertex);

				glBegin(GL_POINTS);
				for(int i=0;i<m_nPlanes;i++)
				{
					glArrayElement(i);
				}
				glEnd();
				m_nPlanes=0;
				glEndList();
			}
		}
		if(m_call_list.size()>0)
		{
			for(size_t i=0;i<m_call_list.size();i++)
				glCallList(m_call_list[i]);
		}
	}


	

	glPopMatrix();


	if (m_nManualPath==1)
	{
		float fYMin=m_fScreenTopY,fYMax=-m_fScreenTopY;
		//float fXMin=m_fScreenTopX,fXMax=-m_fScreenTopX;
		float fStep=-m_fScreenTopY*2/6;
		float fXMin=-fStep*5,fXMax=fStep*5;
		for (i=0;i<11;i++)
		{
			m_fGridLinesPos[i][0]=fXMin+i*fStep;
			m_fGridLinesPos[i][1]=fYMin;
			m_fGridLinesPos[i][2]=0;

			m_fGridLinesPos[i][3]=fXMin+i*fStep;
			m_fGridLinesPos[i][4]=fYMax;
			m_fGridLinesPos[i][5]=0;
		}

		for (i=0;i<7;i++)
		{
			m_fGridLinesPos[i+11][0]=fXMin;
			m_fGridLinesPos[i+11][1]=fYMin+i*fStep;
			m_fGridLinesPos[i+11][2]=0;

			m_fGridLinesPos[i+11][3]=fXMax;
			m_fGridLinesPos[i+11][4]=fYMin+i*fStep;
			m_fGridLinesPos[i+11][5]=0;
		}

		m_fScaleGridPos[0][0]=fXMax-fStep;
		m_fScaleGridPos[0][1]=fYMin;
		m_fScaleGridPos[0][2]=0;

		m_fScaleGridPos[1][0]=fXMax-fStep;
		m_fScaleGridPos[1][1]=fYMin+fStep;
		m_fScaleGridPos[1][2]=0;

		m_fScaleGridPos[2][0]=fXMax;
		m_fScaleGridPos[2][1]=fYMin+fStep;
		m_fScaleGridPos[2][2]=0;

		m_fScaleGridPos[3][0]=fXMax;
		m_fScaleGridPos[3][1]=fYMin;
		m_fScaleGridPos[3][2]=0;

		float fScale=(m_dTranslationZ+2+KINECT_HEIGHT)*0.1;

		int nInteger=(int)fScale;
		float fDec=fScale-nInteger;
		int nLen;

		itoa(nInteger,m_cScale,10);
		nLen=strlen(m_cScale);
		m_cScale[nLen]=46;
		nLen++;

		nInteger=fDec*100;
		itoa(nInteger,&m_cScale[nLen],10);
		nLen=strlen(m_cScale);
		m_cScale[nLen]=109;


		glColor3f(0.0f, 1.0f, 0.0f);
		glRasterPos2f(fXMax-fStep+0.02, fYMin+fStep-0.02);
		DrawString(m_cScale,2);

		glRasterPos2f(fXMax-fStep+0.1, fYMin+fStep-0.1);
		DrawString(m_cScale,2);
		

		DrawScaleGridLines();

	}

/*	glPushMatrix();
	glRotated(m_dRotationX, 1, 0, 0);
	glRotated(m_dRotationY, 0, 1, 0);
	if (m_nManualPath==1)
	{
		


		float fXMin=m_dKinectPose[0]-2.5,fXMax=m_dKinectPose[0]+2.5;
		float fZMin=m_dKinectPose[2]-1.5,fZMax=m_dKinectPose[2]+1.5;

		for (i=0;i<11;i++)
		{
			m_fGridLinesPos[i][0]=fXMin+i*0.5;
			m_fGridLinesPos[i][1]=3;
			m_fGridLinesPos[i][2]=fZMin;

			m_fGridLinesPos[i][3]=fXMin+i*0.5;
			m_fGridLinesPos[i][4]=3;
			m_fGridLinesPos[i][5]=fZMax;
		}

		for (i=0;i<7;i++)
		{
			m_fGridLinesPos[i+11][0]=fXMin;
			m_fGridLinesPos[i+11][1]=3;
			m_fGridLinesPos[i+11][2]=fZMin+i*0.5;

			m_fGridLinesPos[i+11][3]=fXMax;
			m_fGridLinesPos[i+11][4]=3;
			m_fGridLinesPos[i+11][5]=fZMin+i*0.5;
		}

		m_fScaleGridPos[0][0]=fXMax-0.5;
		m_fScaleGridPos[0][1]=3;
		m_fScaleGridPos[0][2]=fZMax;

		m_fScaleGridPos[1][0]=fXMax-0.5;
		m_fScaleGridPos[1][1]=3;
		m_fScaleGridPos[1][2]=fZMax-0.5;

		m_fScaleGridPos[2][0]=fXMax;
		m_fScaleGridPos[2][1]=3;
		m_fScaleGridPos[2][2]=fZMax-0.5;

		m_fScaleGridPos[3][0]=fXMax;
		m_fScaleGridPos[3][1]=3;
		m_fScaleGridPos[3][2]=fZMax;

	

		DrawScaleGridLines();

	}

	glPopMatrix();*/



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
		
	DrawMotionPath();
		


}

UINT OpenGL::ThreadTest(LPDWORD param)
{
	int nTmp=0;
	while (nTmp<m_nMileStoneLen)
	{
		m_cMileStoneFlag[nTmp]=0;
		nTmp++;
		Sleep(1000);
	}

	return 0;
}

void OpenGL::OnLButtonUp(UINT nFlags, CPoint point) 
{
	// TODO: Add your message handler code here and/or call default
	m_MouseDownPoint=CPoint(0,0);
	ReleaseCapture();
	g_hCapture=0;
	g_nLBD=0;

	if(m_nManualPath)
	{
		//m_nManualPath=0;
		printf("current point num:  %d",m_nManualPathArrayLen);

		if (m_nManualPathArrayLen>0)
		{
			DrawRobotPath();

			if (m_nManualPathArrayLen==2)
			{
				m_nDrawVitualLine=1;

			
			}
		//	m_dKinectPose[0],m_dKinectPose[1],m_dKinectPose[2]
		//	m_nManualPathArray[0]=m_dKinectPose[0];
		//	m_nManualPathArray[1]=m_dKinectPose[2];
		}
		
		m_nManualPathArrayLen=0;
	}



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
	if (m_nManualPath)
	{
		if(g_nLBD==1)
		{
		//	printf("On Mouse Move iN!!! \n");
			if (m_nManualPathArrayLen<10000)
			{
			//	if (abs(point.x-m_nManualPathArray[m_nManualPathArrayLen-2])>5
			//		&&abs(point.y-m_nManualPathArray[m_nManualPathArrayLen-1])>5)
				{
					m_nManualPathArray[m_nManualPathArrayLen]=point.x;
					m_nManualPathArrayLen++;
					m_nManualPathArray[m_nManualPathArrayLen]=point.y;
					m_nManualPathArrayLen++;
				}

			}
			//printf("x:  %d   y:  %d",point.x,point.y);
			//Sleep(2);
		}
	}
	else
	{
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


}


/*void OpenGL::OnMouseMove(UINT nFlags, CPoint point) 
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

}*/

BOOL  OpenGL:: OnMouseWheel(   UINT   nFlags,   short   zDelta,   CPoint   pt   )
{
	if (zDelta>0)
	{
		m_dTranslationZ += 0.2;
	}
	else if(zDelta<0)
	{
		m_dTranslationZ-=0.2;
	}
	return TRUE;
}


/*
void OpenGL::CallBack_PIPQVGA(unsigned char *pucQVGAImg,void *pContext)
{
int i;
//memcpy(m_pucPIPColor,pucQVGAImg,IOTGUI_PIP_TRIBLESIZE);
DWORD start=GetTickCount();
memcpy(m_pucPIP_CIFColor,pucQVGAImg,IOTGUI_PIP_TRIBLESIZE_CIF);

glLoadIdentity ();
gluLookAt(0.0,0.0,-2,
0.0,-0.0,0.0,
0.0,-1.0,0.0);

glEnableClientState(GL_COLOR_ARRAY);
glEnableClientState(GL_VERTEX_ARRAY);


glColorPointer(3,GL_UNSIGNED_BYTE,0,m_pucPIP_CIFColor);
glVertexPointer(3,GL_FLOAT,0,m_pfPIP_CIFVertex);
glBegin(GL_POINTS);


for (i=0;i<352*288;i++)
{
glArrayElement(i);
}

glEnd();
glFlush();
printf("time consume:  %d \n",GetTickCount()-start);
}*/
void OpenGL::CallBack_PointCloud(unsigned char *pucImg,float *pfVertex,void *pContext)
{

	memcpy(m_pucColor,pucImg,IOTGUI_PC_TRIBLESIZE);
	memcpy(m_pfPC,pfVertex,(IOTGUI_PC_TRIBLESIZE*4));
}


void OpenGL::CallBack_RunState(char *cState)
{
	//1 运行状态  4
	//2 当前 Session 数 4
	//3 当前node数  4
	//4 如果有IMU信息，加入IMU信息  48(6*8)

	int nNodeNum,nStt,nSessionID,i,j;

	char a1[100],a2[100];
	float fWrongFlag;
	double	dVal;
	memcpy(&nStt,cState,4);
	m_cRunState=nStt;
	memcpy(&nSessionID,cState+4,4);
	memcpy(&nNodeNum,cState+8,4);
	memset(a1,0,100);
	memset(a2,0,100);

	memset(m_cNodeNum,300,0);

//	printf("nSessionID :%d  ",nSessionID);
//	printf("nNodeNum :%d  ",nNodeNum);
	itoa(nSessionID,a1,10);
	itoa(nNodeNum,a2,10);
	
	//printf("a1:  %s  ",a1);
	//printf("a2:  %s  ",a2);
	i=0;
	j=0;
	while (a1[i]!='\0')
	{
		m_cNodeNum[j]=a1[i];
		j++;
		i++;
	}
	m_cNodeNum[j++]=32;
	m_cNodeNum[j++]=58;
	m_cNodeNum[j++]=32;

	i=0;
	while (a2[i]!='\0')
	{
		m_cNodeNum[j]=a2[i];
		j++;
		i++;
	}
//	printf("Run State:  %s \n",m_cNodeNum);

	memcpy(m_dChassisPose,cState+36,24);

	memcpy(&fWrongFlag,cState+60,4);

	if (fWrongFlag!=-99999)
	{
		memcpy(&dVal,cState+60,8);
		m_dKinectPose[0]=dVal;
		printf("x:  %f  ",dVal);
		memcpy(&dVal,cState+68,8);
		m_dKinectPose[1]=dVal;
		printf("y:  %f  ",dVal);
		memcpy(&dVal,cState+76,8);
		m_dKinectPose[2]=dVal;
		printf("z:  %f  ",dVal);
		memcpy(&dVal,cState+84,8);
		m_dKinectPose[3]=dVal*180/3.1415926;
		memcpy(&dVal,cState+92,8);
		m_dKinectPose[4]=dVal*180/3.1415926;
		memcpy(&dVal,cState+100,8);
		m_dKinectPose[5]=dVal*180/3.1415926;

	}
	//m_dChassisPose[0]+=180;
	//m_dChassisPose[1]+=180;
	//m_dChassisPose[1]+=180;
	if (m_dChassisPose[0]==4&&m_dChassisPose[1]==5&&m_dChassisPose[2]==6)
	{
	}
	else
	{
		if (m_nFirstPoseRcd<5)
		{
			m_nFirstPoseRcd++;
			memcpy(m_dFirstPose,m_dChassisPose,24);
		}
	}


}

void OpenGL::CallBack_SLAM_PC(unsigned char *pucImg,float *pfVertex,void *pContext,int nCount)
{
	int *pnFlag=(int*)pContext,i,nNum,nRealNum;
	float *fpTmp;
	unsigned char *pucImg11;

	DWORD start=::GetTickCount();
	if (*pnFlag==-1)
	{
		m_nCurrentCount=0;
	}
	if (m_nHorizonCutPC)
	{
		nNum=nCount/3;
		nRealNum=0;
		if (m_nCurrentCount+nCount<IOTGUI_SLAM_SIZE)
		{
			fpTmp=m_pfSLAMPC+m_nCurrentCount;
			pucImg11=m_pucSLAMColor+m_nCurrentCount;
			for (i=0;i<nNum;i++)
			{
				if (pfVertex[i*3+1]>0.3)
				{
					memcpy(fpTmp,&pfVertex[i*3],3*4);
					memcpy(pucImg11,&pucImg[i*3],3);

					fpTmp+=3;
					pucImg11+=3;
					nRealNum+=1;
				}
				else
				{
					int ddddddd=10;
				}
			}
		}
		m_nCurrentCount+=nRealNum;
	}
	else
	{
		printf("Timr comsume  :  %d  \n",::GetTickCount()-start);
		if (m_nCurrentCount+nCount<IOTGUI_SLAM_SIZE)
		{
			memcpy(m_pucSLAMColor+m_nCurrentCount,pucImg,nCount);
			memcpy(m_pfSLAMPC+m_nCurrentCount,pfVertex,nCount*4);
			m_nCurrentCount+=nCount/3;
		}
		else
		{
			printf("Exceed Max PC Storage \n");
		}
	}
	/*nNum=nCount/3;
	nRealNum=0;
	if (m_nCurrentCount+nCount<IOTGUI_SLAM_SIZE)
	{
		fpTmp=m_pfSLAMPC+m_nCurrentCount;
		pucImg11=m_pucSLAMColor+m_nCurrentCount;
		for (i=0;i<nNum;i++)
		{
			if (pfVertex[i*3+1]>0.3)
			{
				memcpy(fpTmp,&pfVertex[i*3],3*4);
				memcpy(pucImg11,&pucImg[i*3],3);

				fpTmp+=3;
				pucImg11+=3;
				nRealNum+=1;
			}
			else
			{
				int ddddddd=10;
			}
		}
	}
	m_nCurrentCount+=nRealNum;*/



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
	int i;
	int nPathNodeNum=*((int *)pContext);
	float *pfMatTmp=pfTMat,*pfRAngleTmp=pfRAngle;
	for (i=0;i<nPathNodeNum;i++)
	{
		AddPathNode(pfMatTmp,pfRAngleTmp);
		pfMatTmp+=3;
		pfRAngleTmp+=3;
	}

	/*
	m_dKinectPose[0]=pfTMat[nPathNodeNum*3-3];
		m_dKinectPose[1]=pfTMat[nPathNodeNum*3-2];
		m_dKinectPose[2]=pfTMat[nPathNodeNum*3-1];
	
		m_dKinectPose[3]=pfRAngle[nPathNodeNum*3-3]*180/3.1415926;
		m_dKinectPose[4]=pfRAngle[nPathNodeNum*3-2]*180/3.1415926;
		m_dKinectPose[5]=pfRAngle[nPathNodeNum*3-1]*180/3.1415926;*/
	

}


int OpenGL::PathTwinkle()
{
	/*if (m_stGLPath.pstPathHead!=NULL)
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
	*/


	return 0;
}




int OpenGL::DrawPath()
{

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
			glRotated(pstPathNode->fRAngle[0],1,0,0);
			glRotated(pstPathNode->fRAngle[1],0,1,0);
			glRotated(pstPathNode->fRAngle[2],0,0,1);
			glColor3f(1.0,0.0,0.0);
			glutSolidSphere(0.02,10,10);


			//coordinate
			glLineWidth(5);
			glBegin(GL_LINES);
			
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


int OpenGL::DrawChassisPose()
{
	glPushMatrix();
	glTranslated(0.5, 0, 0);
	//glRotated(m_dChassisPose[0], m_dChassisPose[1], m_dChassisPose[2]);
	glRotated(m_dFirstPose[0]-m_dChassisPose[0],1,0,0);
	glRotated(-(m_dFirstPose[2]-m_dChassisPose[2]),0,1,0);
	glRotated(m_dFirstPose[1]-m_dChassisPose[1],0,0,1);

	//coordinate
	glLineWidth(5);
	glBegin(GL_LINES);
	
	glColor3f(1.0,0.0,0.0);
	glVertex3f(0,0,0);
	glVertex3f(0.5,0,0);

	glColor3f(0.0,1.0,0.0);
	glVertex3f(0,0,0);
	glVertex3f(0,0.5,0);

	glColor3f(0.0,0.0,1.0);
	glVertex3f(0,0,0);
	glVertex3f(0,0,0.5);
	glEnd();
	glPopMatrix();
	return 0;
}

int OpenGL::DrawArrow()
{

	float Pi=3.1415926,R=0.3;
	int i=0,n=20;
	float fAngle;
	glLineWidth( 1 );//设置线的宽度为6
	fAngle=m_dFirstPose[2]+179;
	glPushMatrix();
	glTranslated(0, 0.3, 0);
	glRotated(-m_dFirstPose[0],1,0,0);
	printf("Z:  %f  \n",fAngle);
	glRotated(-fAngle,0,1,0);
	glRotated(-m_dFirstPose[1],0,0,1);


	glColor3f(0.3,0.3,0.3);
	glBegin(GL_LINE_LOOP);
	for (i=0;i<20;i++)
	{
		glVertex3f(m_fCircle[i*3],0,m_fCircle[i*3+2]);
	}
	glEnd();


	glColor3f(0.18,0.74,0.5);
	glBegin(GL_POLYGON);
	glVertex3f(m_fCircle[15],0,m_fCircle[17]);
	glVertex3f(m_fCircle[39],0,m_fCircle[41]);
	glVertex3f(0,0,0);
	glEnd();

	glColor3f(0.57,0.54,0.49);
	glBegin(GL_LINE_STRIP);
	glVertex3f(m_fCircle[15],0,m_fCircle[17]);
	glVertex3f(m_fCircle[51],0,m_fCircle[53]);
	glVertex3f(0,0,0);
	glEnd();
	glPopMatrix();
	glLineWidth( 3 );//设置线的宽度为6

/*	glClear(GL_DEPTH_BUFFER_BIT);
	glEnable(GL_DEPTH_TEST);
	glColor3f(0.5,0.5,0.5);
	glBegin(GL_POLYGON);


	glEdgeFlag(GL_TRUE);
	glVertex3f(0,-0.4,0);
	glEdgeFlag(GL_TRUE);
	glVertex3f(0,-0.5,0);
	glEdgeFlag(GL_TRUE);
	glVertex3f(0.3,-0.5,0);
	glEdgeFlag(GL_TRUE);
	glVertex3f(0.3,-0.4,0);

	glEdgeFlag(GL_TRUE);
	glVertex3f(0,-0.4,0.05);
	glEdgeFlag(GL_TRUE);
	glVertex3f(0,-0.5,0.05);
	glEdgeFlag(GL_TRUE);
	glVertex3f(0.3,-0.5,0.05);
	glEdgeFlag(GL_TRUE);
	glVertex3f(0.3,-0.4,0.05);
	glEnd();
	glBegin(GL_POLYGON);
	glEdgeFlag(GL_TRUE);
	glVertex3f(0.3,-0.35,0);
	glEdgeFlag(GL_TRUE);
	glVertex3f(0.4,-0.45,0);
	glEdgeFlag(GL_TRUE);
	glVertex3f(0.3,-0.55,0);

	glEdgeFlag(GL_TRUE);
	glVertex3f(0.3,-0.35,0.05);
	glEdgeFlag(GL_TRUE);
	glVertex3f(0.4,-0.45,0.05);
	glEdgeFlag(GL_TRUE);
	glVertex3f(0.3,-0.55,0.05);
	glEnd();

	glBegin(GL_QUADS);
	glEdgeFlag(GL_TRUE);
	glVertex3f(0,-0.4,0);
	glEdgeFlag(GL_TRUE);
	glVertex3f(0,-0.5,0);
	glEdgeFlag(GL_TRUE);
	glVertex3f(0,-0.5,0.05);
	glEdgeFlag(GL_TRUE);
	glVertex3f(0,-0.4,0.05);

	glEdgeFlag(GL_TRUE);
	glVertex3f(0,-0.5,0);
	glEdgeFlag(GL_TRUE);
	glVertex3f(0.3,-0.5,0);
	glEdgeFlag(GL_TRUE);
	glVertex3f(0.3,-0.5,0.05);
	glEdgeFlag(GL_TRUE);
	glVertex3f(0,-0.5,0.05);

	glEdgeFlag(GL_TRUE);
	glVertex3f(0,-0.4,0);
	glEdgeFlag(GL_TRUE);
	glVertex3f(0.3,-0.4,0);
	glEdgeFlag(GL_TRUE);
	glVertex3f(0.3,-0.4,0.05);
	glEdgeFlag(GL_TRUE);
	glVertex3f(0,-0.4,0.05);

	glEdgeFlag(GL_TRUE);
	glVertex3f(0.3,-0.5,0);
	glEdgeFlag(GL_TRUE);
	glVertex3f(0.3,-0.55,0);
	glEdgeFlag(GL_TRUE);
	glVertex3f(0.3,-0.55,0.05);
	glEdgeFlag(GL_TRUE);
	glVertex3f(0.3,-0.5,0.05);

	glEdgeFlag(GL_TRUE);
	glVertex3f(0.3,-0.55,0);
	glEdgeFlag(GL_TRUE);
	glVertex3f(0.4,-0.45,0);
	glEdgeFlag(GL_TRUE);
	glVertex3f(0.4,-0.45,0.05);
	glEdgeFlag(GL_TRUE);
	glVertex3f(0.3,-0.55,0.05);

	glVertex3f(0.4,-0.45,0);
	glVertex3f(0.3,-0.35,0);
	glVertex3f(0.3,-0.35,0.05);
	glVertex3f(0.4,-0.45,0.05);

	glVertex3f(0.3,-0.35,0);
	glVertex3f(0.3,-0.4,0);
	glVertex3f(0.3,-0.4,0.05);
	glVertex3f(0.3,-0.35,0.05);

	glEnd();

	glColor3f(1,1,1);
	glBegin(GL_LINES);
	glVertex3f(0,-0.4,0);
	glVertex3f(0,-0.5,0);

	glVertex3f(0,-0.5,0);
	glVertex3f(0.3,-0.5,0);

	glVertex3f(0.3,-0.5,0);
	glVertex3f(0.3,-0.55,0);

	glVertex3f(0.3,-0.55,0);
	glVertex3f(0.4,-0.45,0);

	glVertex3f(0.4,-0.45,0);
	glVertex3f(0.3,-0.35,0);

	glVertex3f(0.3,-0.35,0);
	glVertex3f(0.3,-0.4,0);

	glVertex3f(0.3,-0.4,0);
	glVertex3f(0,-0.4,0);


	glVertex3f(0,-0.4,0.05);
	glVertex3f(0,-0.5,0.05);

	glVertex3f(0,-0.5,0.05);
	glVertex3f(0.3,-0.5,0.05);

	glVertex3f(0.3,-0.5,0.05);
	glVertex3f(0.3,-0.55,0.05);

	glVertex3f(0.3,-0.55,0.05);
	glVertex3f(0.4,-0.45,0.05);

	glVertex3f(0.4,-0.45,0.05);
	glVertex3f(0.3,-0.35,0.05);

	glVertex3f(0.3,-0.35,0.05);
	glVertex3f(0.3,-0.4,0.05);

	glVertex3f(0.3,-0.4,0.05);
	glVertex3f(0,-0.4,0.05);




	glVertex3f(0,-0.4,0);
	glVertex3f(0,-0.4,0.05);
	glVertex3f(0,-0.5,0);
	glVertex3f(0,-0.5,0.05);
	glVertex3f(0.3,-0.5,0);
	glVertex3f(0.3,-0.5,0.05);
	glVertex3f(0.3,-0.4,0);
	glVertex3f(0.3,-0.4,0.05);

	glVertex3f(0.3,-0.35,0);
	glVertex3f(0.3,-0.35,0.05);
	glVertex3f(0.4,-0.45,0);
	glVertex3f(0.4,-0.45,0.05);
	glVertex3f(0.3,-0.55,0);
	glVertex3f(0.3,-0.55,0.05);
	glEnd();
	glDisable(GL_DEPTH_TEST);*/
	return 0;
}


int OpenGL::DrawMotionPath()
{
	int i=0,nHalfWidth=m_nSceenWidth/2,nHalfHeight=m_nSceenHeight/2;
	float fYRatio=0.7279;  //tan20
	float fXRatio=0.7279*m_nSceenWidth/m_nSceenHeight;
	float fX,fY,fX2,fY2;


	glColor3f(1,0,0);
	glBegin(GL_LINE_STRIP);
	for (i=0;i<m_nManualPathArrayLen;i=i+2)
	{
		fX=((float)m_nManualPathArray[i]-nHalfWidth)/nHalfWidth*fXRatio;
		fY=((float)m_nManualPathArray[i+1]-nHalfHeight)/nHalfHeight*fYRatio;
		//m_fPathArray[i]=fX;
		//m_fPathArray[i+1]=fY;
		glVertex3f(fX,fY,0);
	}
	glEnd();
	glFlush();
	return 0;
}

int OpenGL::DrawRobotPath()
{
	int i=0,nHalfWidth=m_nSceenWidth/2,nHalfHeight=m_nSceenHeight/2;
	float fYRatio=0.7279;  //tan20
	float fXRatio=0.7279*m_nSceenWidth/m_nSceenHeight;
	float fRatio=(m_dTranslationZ+2+KINECT_HEIGHT)/2;
	float fX,fY,fX2,fY2;

	float x1,x2,y1,y2;
	float angle1,angle2;

	int m_nTmp[300];
	int m_nTmpLen=0;

	float fMaxStepLen;

	if (m_nManualPathArrayLen==2)
	{
		m_nMileStoneLen=1;
		fX=((float)m_nManualPathArray[i]-nHalfWidth)/nHalfWidth*fXRatio;
		fY=((float)m_nManualPathArray[i+1]-nHalfHeight)/nHalfHeight*fYRatio;

		m_fMileStonePos[0]=fRatio*fX;
		m_fMileStonePos[1]=KINECT_HEIGHT;
		m_fMileStonePos[2]=-fRatio*fY;
	}
	else
	{
		float fRatio=(m_dTranslationZ+2+KINECT_HEIGHT)/2;
		for (i=0;i<m_nManualPathArrayLen;i=i+2)
		{
			fX=((float)m_nManualPathArray[i]-nHalfWidth)/nHalfWidth*fXRatio;
			fY=((float)m_nManualPathArray[i+1]-nHalfHeight)/nHalfHeight*fYRatio;
			m_fPathArray[i]=fX;
			m_fPathArray[i+1]=fY;
			glVertex3f(fX,fY,0);
		}
		m_fThres=MIN_INFLEXION_LEN/fRatio;
		fMaxStepLen=MAX_ONE_STEP_LEN/fRatio;
		m_nInflectionPointsNum=0;
		m_nInflectionPointsIdxArray[0]=0;
		m_nInflectionPointsNum++;
		GenerateInflectionPoint(m_fPathArray,0,m_nManualPathArrayLen/2-1,m_fThres,fMaxStepLen);
		m_nInflectionPointsIdxArray[m_nInflectionPointsNum]=m_nManualPathArrayLen/2-1;
		m_nInflectionPointsNum++;


		/*if (m_nManualPathArrayLen!=0)
		{
		m_nTmp[0]=m_nManualPathArray[0];
		m_nTmpLen++;
		m_nTmp[1]=m_nManualPathArray[1];
		m_nTmpLen++;

		x1=m_nManualPathArray[0];
		y1=m_nManualPathArray[1];
		for (i=2;i<m_nManualPathArrayLen-2;i=i+2)
		{
		//x1=m_nManualPathArray[i]-m_nManualPathArray[i-2];
		x2=m_nManualPathArray[i+2]-m_nManualPathArray[i];

		//y1=m_nManualPathArray[i+1]-m_nManualPathArray[i-1];
		y2=m_nManualPathArray[i+3]-m_nManualPathArray[i+1];

		angle1=atan2(y1,x1)*180/3.1415926;
		angle2=atan2(y2,x2)*180/3.1415926;

		//if (fabs(angle1-angle2)>15&&fabs(angle1-angle2)<165)
		if (fabs(angle1-angle2)>15)
		{
		m_nTmp[m_nTmpLen]=m_nManualPathArray[i];
		m_nTmpLen++;
		m_nTmp[m_nTmpLen]=m_nManualPathArray[i+1];
		m_nTmpLen++;
		x1=x2;
		y1=y2;
		}

		}

		m_nTmp[m_nTmpLen]=m_nManualPathArray[m_nManualPathArrayLen-2];
		m_nTmpLen++;
		m_nTmp[m_nTmpLen]=m_nManualPathArray[m_nManualPathArrayLen-1];
		m_nTmpLen++;


		m_nRobotPathArray[0]=m_nTmp[0];
		m_nRobotPathArray[1]=m_nTmp[1];

		m_nRobotPathArrayLen=2;
		for (i=2;i<m_nTmpLen;i=i+2)
		{
		if (abs(m_nTmp[i]-m_nRobotPathArray[m_nRobotPathArrayLen-2])>=10
		&&abs(m_nTmp[i+1]-m_nRobotPathArray[m_nRobotPathArrayLen-1])>=10)
		{
		m_nRobotPathArray[m_nRobotPathArrayLen]=m_nTmp[i];
		m_nRobotPathArrayLen++;
		m_nRobotPathArray[m_nRobotPathArrayLen]=m_nTmp[i+1];
		m_nRobotPathArrayLen++;
		}
		}

		m_nRobotPathArray[m_nRobotPathArrayLen]=m_nTmp[m_nTmpLen-2];
		m_nRobotPathArrayLen++;
		m_nRobotPathArray[m_nRobotPathArrayLen]=m_nTmp[m_nTmpLen-1];
		m_nRobotPathArrayLen++;
		printf("Robot Path Num: %d",m_nRobotPathArrayLen);
		}*/

		BuildMileStone();
	}
	
	return 0;
}


void OpenGL::DrawString(const char* str,int nIdx)
{
	static int isFirstCall = 1;

	static GLuint lists;
	static GLuint lists2;


	if( isFirstCall ) { // 如果是第一次调用，执行初始化

		// 为每一个ASCII字符产生一个显示列表

		isFirstCall = 0;



		// 申请MAX_CHAR个连续的显示列表编号

		lists = glGenLists(MAX_CHAR);



		// 把每个字符的绘制命令都装到对应的显示列表中

		wglUseFontBitmaps(wglGetCurrentDC(), 0, MAX_CHAR, lists);


		lists2=glGenLists(MAX_CHAR);

		selectFont(20, ANSI_CHARSET, "Arial Black");

		wglUseFontBitmaps(wglGetCurrentDC(), 0, MAX_CHAR, lists2);

	}

	// 调用每个字符对应的显示列表，绘制每个字符

	if (nIdx==1)
	{
		for(; *str!='\0'; ++str)

			glCallList(lists + *str);
	}
	else if (nIdx==2)
	{
		for(; *str!='\0'; ++str)

			glCallList(lists2 + *str);
	}

}
void OpenGL::drawString(const char* str) {

	static int isFirstCall = 1;

	static GLuint lists;


	if( isFirstCall ) { // 如果是第一次调用，执行初始化

		// 为每一个ASCII字符产生一个显示列表

		isFirstCall = 0;



		// 申请MAX_CHAR个连续的显示列表编号

		lists = glGenLists(MAX_CHAR);



		// 把每个字符的绘制命令都装到对应的显示列表中

		wglUseFontBitmaps(wglGetCurrentDC(), 0, MAX_CHAR, lists);

	}

	// 调用每个字符对应的显示列表，绘制每个字符

	for(; *str!='\0'; ++str)

		glCallList(lists + *str);

}


void OpenGL::drawStringForScaleGrid(const char* str) {

	static int isFirstCallForScaleGrid = 1;

	static GLuint listsForScaleGrid;


	if( isFirstCallForScaleGrid ) { // 如果是第一次调用，执行初始化

		// 为每一个ASCII字符产生一个显示列表

		isFirstCallForScaleGrid = 0;



		// 申请MAX_CHAR个连续的显示列表编号

		listsForScaleGrid = glGenLists(MAX_CHAR);



		// 把每个字符的绘制命令都装到对应的显示列表中

		wglUseFontBitmaps(wglGetCurrentDC(), 0, MAX_CHAR, listsForScaleGrid);

	}

	// 调用每个字符对应的显示列表，绘制每个字符

	for(; *str!='\0'; ++str)

		glCallList(listsForScaleGrid + *str);

}
void OpenGL::selectFontForScaleGrid(int size, int charset, const char* face) {

	HFONT hFontForScaleGrid = CreateFontA(size, 0, 0, 0, FW_MEDIUM, 0, 0, 0,

		charset, OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS,

		DEFAULT_QUALITY, DEFAULT_PITCH | FF_SWISS, face);

	HFONT hOldFontForScaleGrid = (HFONT)SelectObject(wglGetCurrentDC(), hFontForScaleGrid);

	DeleteObject(hOldFontForScaleGrid);

}



void OpenGL::selectFont(int size, int charset, const char* face) {

	HFONT hFont = CreateFontA(size, 0, 0, 0, FW_MEDIUM, 0, 0, 0,

		charset, OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS,

		DEFAULT_QUALITY, DEFAULT_PITCH | FF_SWISS, face);

	HFONT hOldFont = (HFONT)SelectObject(wglGetCurrentDC(), hFont);

	DeleteObject(hOldFont);

}

int OpenGL::DrawCurrentPose()
{
	
	float fX,fY,fZ,fDis;
	//I have draw the draft on my note
	float fL1=0.288675/2,fL2=0.1443375/2,fL3=0.25/2,fL4=0.05/2,fL5=0.43301/2,fL6=0.5/2,fL7=0.08333/2,fL9=tan(3.1415926/6)*fL7;

	glPushMatrix();
	glTranslated(m_dKinectPose[0],m_dKinectPose[1],m_dKinectPose[2]);


	glRotated(m_dKinectPose[3],1,0,0);
	glRotated(m_dKinectPose[4],0,1,0);
	glRotated(m_dKinectPose[5],0,0,1);

	glClear(GL_DEPTH_BUFFER_BIT);
	glEnable(GL_DEPTH_TEST);



	glBegin(GL_TRIANGLES);

	//
	//     *
	//     **
	//     ***
	glColor3f(0.5,0.5,0.5);
	glVertex3f(0,-fL4,fL1);
	glVertex3f(fL3,-fL4,-fL2);
	glVertex3f(0.0,-fL4,0);

	glVertex3f(0,fL4,fL1);
	glVertex3f(fL3,fL4,-fL2);
	glVertex3f(0.0,fL4,0);

	//     *
	//    **
	//   ***
	glColor3f(0.1,0.1,0.7);
	glVertex3f(0,-fL4,fL1);
	glVertex3f(-fL3,-fL4,-fL2);
	glVertex3f(0.0,-fL4,0);

	glVertex3f(0,fL4,fL1);
	glVertex3f(-fL3,fL4,-fL2);
	glVertex3f(0.0,fL4,0);
	glEnd();

	//profile of the triangle
	glBegin(GL_QUADS);
	glColor3f(0.5,0.5,0.5);

	glVertex3f(0,-fL4,fL1);
	glVertex3f(0,fL4,fL1);
	glVertex3f(fL3,fL4,-fL2);
	glVertex3f(fL3,-fL4,-fL2);

	glVertex3f(fL3,fL4,-fL2);
	glVertex3f(fL3,-fL4,-fL2);
	glVertex3f(0.0,-fL4,0);
	glVertex3f(0.0,fL4,0);

	glColor3f(0.1,0.1,0.7);

	glVertex3f(0,-fL4,fL1);
	glVertex3f(-fL3,-fL4,-fL2);
	glVertex3f(-fL3,fL4,-fL2);
	glVertex3f(0,fL4,fL1);
	
	glVertex3f(-fL3,-fL4,-fL2);
	glVertex3f(-fL3,fL4,-fL2);
	glVertex3f(0.0,fL4,0);
	glVertex3f(0.0,-fL4,0);
	glEnd();


	//   **
	//   **
	//   **
	glBegin(GL_QUADS);
	glColor3f(0.1,0.1,0.7);
	glVertex3f(0,fL4,0);
	glVertex3f(-fL7,fL4,0);
	glVertex3f(-fL7,fL4,-fL1);
	glVertex3f(0.0,fL4,-fL1);


	glVertex3f(0,-fL4,0);
	glVertex3f(-fL7,-fL4,0);
	glVertex3f(-fL7,-fL4,-fL1);
	glVertex3f(0.0,-fL4,-fL1);

	glVertex3f(-fL7,-fL4,0);
	glVertex3f(-fL7,-fL4,-fL1);
	glVertex3f(-fL7,fL4,-fL1);
	glVertex3f(-fL7,fL4,0);

	glVertex3f(-fL7,-fL4,-fL1);
	glVertex3f(0.0,-fL4,-fL1);
	glVertex3f(0.0,fL4,-fL1);
	glVertex3f(-fL7,fL4,-fL1);
	glEnd();

	//   **
	//   **
	//   **
	glBegin(GL_QUADS);
	glColor3f(0.5,0.5,0.5);
	glVertex3f(0,fL4,0);
	glVertex3f(fL7,fL4,0);
	glVertex3f(fL7,fL4,-fL1);
	glVertex3f(0.0,fL4,-fL1);


	glVertex3f(0,-fL4,0);
	glVertex3f(fL7,-fL4,0);
	glVertex3f(fL7,-fL4,-fL1);
	glVertex3f(0.0,-fL4,-fL1);

	glVertex3f(fL7,-fL4,0);
	glVertex3f(fL7,-fL4,-fL1);
	glVertex3f(fL7,fL4,-fL1);
	glVertex3f(fL7,fL4,0);

	glVertex3f(fL7,-fL4,-fL1);
	glVertex3f(0.0,-fL4,-fL1);
	glVertex3f(0.0,fL4,-fL1);
	glVertex3f(fL7,fL4,-fL1);
	glEnd();

	glLineWidth(2);
	glBegin(GL_LINES);
	glColor3f(1,1,1);
	////////////////////////////////
	glVertex3f(0,-fL4,fL1);
	glVertex3f(fL3,-fL4,-fL2);

	glVertex3f(fL3,-fL4,-fL2);
	glVertex3f(fL7,-fL4,-fL9);

	glVertex3f(fL7,-fL4,-fL9);
	glVertex3f(fL7,-fL4,-fL1);

	glVertex3f(fL7,-fL4,-fL1);
	glVertex3f(0,-fL4,-fL1);

	glVertex3f(0,-fL4,-fL1);
	glVertex3f(0,-fL4,fL1);

	//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
	glVertex3f(0,fL4,fL1);
	glVertex3f(fL3,fL4,-fL2);

	glVertex3f(fL3,fL4,-fL2);
	glVertex3f(fL7,fL4,-fL9);

	glVertex3f(fL7,fL4,-fL9);
	glVertex3f(fL7,fL4,-fL1);

	glVertex3f(fL7,fL4,-fL1);
	glVertex3f(0,fL4,-fL1);

	glVertex3f(0,fL4,-fL1);
	glVertex3f(0,fL4,fL1);



	////////////////////////////////
	glVertex3f(0,-fL4,fL1);
	glVertex3f(-fL3,-fL4,-fL2);

	glVertex3f(-fL3,-fL4,-fL2);
	glVertex3f(-fL7,-fL4,-fL9);

	glVertex3f(-fL7,-fL4,-fL9);
	glVertex3f(-fL7,-fL4,-fL1);

	glVertex3f(-fL7,-fL4,-fL1);
	glVertex3f(0,-fL4,-fL1);

	glVertex3f(0,-fL4,-fL1);
	glVertex3f(0,-fL4,fL1);

	//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
	glVertex3f(0,fL4,fL1);
	glVertex3f(-fL3,fL4,-fL2);

	glVertex3f(-fL3,fL4,-fL2);
	glVertex3f(-fL7,fL4,-fL9);

	glVertex3f(-fL7,fL4,-fL9);
	glVertex3f(-fL7,fL4,-fL1);

	glVertex3f(-fL7,fL4,-fL1);
	glVertex3f(0,fL4,-fL1);

	glVertex3f(0,fL4,-fL1);
	glVertex3f(0,fL4,fL1);



	/////////////////////////////////

	//////////////////////////////

	glVertex3f(0,-fL4,fL1);
	glVertex3f(0,fL4,fL1);

	glVertex3f(-fL3,fL4,-fL2);
	glVertex3f(-fL3,-fL4,-fL2);

	glVertex3f(fL3,fL4,-fL2);
	glVertex3f(fL3,-fL4,-fL2);

	glVertex3f(-fL7,fL4,-fL9);
	glVertex3f(-fL7,-fL4,-fL9);

	glVertex3f(-fL7,fL4,-fL1);
	glVertex3f(-fL7,-fL4,-fL1);

	glVertex3f(0,fL4,-fL1);
	glVertex3f(0,-fL4,-fL1);

	glVertex3f(fL7,fL4,-fL9);
	glVertex3f(fL7,-fL4,-fL9);

	glVertex3f(fL7,fL4,-fL1);
	glVertex3f(fL7,-fL4,-fL1);

	glEnd();


	glLineWidth(3);

	glDisable(GL_DEPTH_TEST);

	glPopMatrix();
	/*glPopMatrix();
	glPushMatrix();
	glTranslated(m_dKinectPose[0],m_dKinectPose[1]+0.2,m_dKinectPose[2]);

	glRotated(m_dFirstPose[0]-m_dChassisPose[0],1,0,0);
	glRotated(-(m_dFirstPose[2]-m_dChassisPose[2]),0,1,0);
	glRotated(m_dFirstPose[1]-m_dChassisPose[1],0,0,1);

	glPopMatrix();*/

	return 0;
}


int OpenGL::BuildMileStone()
{

	int i,j,nTmp;

	float fRatio=(m_dTranslationZ+2+KINECT_HEIGHT)/2;
	m_nMileStoneLen=0;

	//冒泡排序
	bubble_sort(m_nInflectionPointsIdxArray,m_nInflectionPointsNum);

	memset(m_cMileStoneFlag,0,100);
	for (i=0;i<m_nInflectionPointsNum;i++)
	{
		m_fMileStonePos[m_nMileStoneLen*3]=fRatio*m_fPathArray[m_nInflectionPointsIdxArray[i]*2];
		m_fMileStonePos[m_nMileStoneLen*3+1]=KINECT_HEIGHT;
		m_fMileStonePos[m_nMileStoneLen*3+2]=-fRatio*m_fPathArray[m_nInflectionPointsIdxArray[i]*2+1];
		m_nMileStoneLen++;
		m_cMileStoneFlag[i]=1;
	}
	//LPDWORD ID=0;
	//CreateThread(NULL,0,(LPTHREAD_START_ROUTINE)ThreadTest,NULL,0,ID);
	/*	int nHalfWidth=m_nSceenWidth/2,nHalfHeight=m_nSceenHeight/2;
	float fYRatio=0.7279;  //tan20
	float fXRatio=0.7279*m_nSceenWidth/m_nSceenHeight;
	float fX,fY,fX2,fY2;
	float fRatio=(m_dTranslationZ+2+KINECT_HEIGHT)/2;

	int i;
	m_nMileStoneLen=0;
	for (i=0;i<m_nRobotPathArrayLen;i=i+2)
	{
	fX=((float)m_nRobotPathArray[i]-nHalfWidth)/nHalfWidth*fXRatio;
	fY=((float)m_nRobotPathArray[i+1]-nHalfHeight)/nHalfHeight*fYRatio;
	//m_fMileStonePos[i*3]=fRatio*fX;
	//m_fMileStonePos[i*3+1]=fRatio*fY;
	//m_fMileStonePos[i*3+2]=m_dTranslationZ+KINECT_HEIGHT;

	m_fMileStonePos[m_nMileStoneLen*3]=fRatio*fX;
	m_fMileStonePos[m_nMileStoneLen*3+1]=KINECT_HEIGHT;
	m_fMileStonePos[m_nMileStoneLen*3+2]=-fRatio*fY;
	m_nMileStoneLen++;
	}*/

	//m_nInflectionPointsNum=0;

	m_nReachMileStoneIdx=0;
	m_nMileStoneHasBeenBuild=1;
	return 0;
}

int OpenGL::DrawMileStone()
{
	int i;
	//
	for (i=0;i<m_nMileStoneLen;i++)
	{
		glPushMatrix();
		glTranslated(m_fMileStonePos[i*3],m_fMileStonePos[i*3+1],m_fMileStonePos[i*3+2]);
		if (m_cMileStoneFlag[i]==1)
		{
			glColor3f(0,1.0,1.0);
		}
		else
		{
			glColor3f(0.3,0.3,0);
		}
		glutSolidSphere(0.05,10,10);
		glPopMatrix();
		glVertex3f(m_fMileStonePos[i*3],m_fMileStonePos[i*3+1],m_fMileStonePos[i*3+2]);
	}

	glLineWidth(4);
	for (i=1;i<m_nMileStoneLen;i++)
	{
		//未到达
		if (m_cMileStoneFlag[i]==1)
		{
			glEnable(GL_LINE_STIPPLE);//可以虚线，先声明可以使用 
			//glLineWidth (2.0);
			glLineStipple (1, 0xF00F);  
			glColor3f(1.0,1.0,1.0);
			glBegin(GL_LINES);  
			glVertex3f (m_fMileStonePos[i*3],m_fMileStonePos[i*3+1],m_fMileStonePos[i*3+2]); 
			glVertex3f (m_fMileStonePos[(i-1)*3],m_fMileStonePos[(i-1)*3+1],m_fMileStonePos[(i-1)*3+2]); 
			glEnd();

			glDisable(GL_LINE_STIPPLE);
		}
		else
		{
			
			glBegin(GL_LINES); 
			glColor3f(1.0,1.0,1.0);
			glVertex3f (m_fMileStonePos[i*3],m_fMileStonePos[i*3+1],m_fMileStonePos[i*3+2]); 
			glVertex3f (m_fMileStonePos[(i-1)*3],m_fMileStonePos[(i-1)*3+1],m_fMileStonePos[(i-1)*3+2]); 
			glEnd();
		}

	}

	glLineWidth(3);

/*	glEnable(GL_LINE_STIPPLE);//可以虚线，先声明可以使用 
	//glLineWidth (2.0);
	glLineStipple (1, 0x0F0F);  
	glColor3f(1.0,1.0,1.0);
	glBegin(GL_LINES);  
	glVertex3f (pfSrcPos[0],KINECT_HEIGHT,pfSrcPos[2]); 
	glVertex3f (pfDstPos[0],KINECT_HEIGHT,pfDstPos[2]); 
	glEnd();

	glDisable(GL_LINE_STIPPLE);*/
/*	glBegin(GL_LINE_STRIP);
	for (i=0;i<m_nMileStoneLen;i++)
	{
		if (m_cMileStoneFlag[i]==1)
		{
			glColor3f(1,0,0);
			glVertex3f(m_fMileStonePos[i*3],m_fMileStonePos[i*3+1],m_fMileStonePos[i*3+2]);
		}
		else
		{
			glColor3f(0.3,0.3,0.3);
			glVertex3f(m_fMileStonePos[i*3],m_fMileStonePos[i*3+1],m_fMileStonePos[i*3+2]);
		}
		
	}
	glEnd();*/
	glFlush();
	return 0;
}
float OpenGL::TwoPointsDis(float x1,float y1,float x2,float y2)
{
	float x,y,fTmp;
	x=x2-x1;
	y=y2-y1;
	fTmp=sqrt((double)x*x+y*y);
	return fTmp;
}

int OpenGL::GenerateInflectionPoint(float *fPointArray,int nStartIdx,int nEndIdx,float fInflexionThres,float fMaxOneStep)
{
	float fMaxVal;
	int nMaxValIdx,i;
	float x1,y1,x2,y2,x,y;
	float fLen,fNum,fTmp,fVal1,fVal2;;

	/*	if (LookForInflectionPoint(fPointArray,nStartIdx,nEndIdx,&nMaxValIdx,&fMaxVal)>fInflexionThres)
	{
	m_nInflectionPointsIdxArray[m_nInflectionPointsNum]=nMaxValIdx;
	m_nInflectionPointsNum++;
	GenerateInflectionPoint(fPointArray,nStartIdx,nMaxValIdx,fInflexionThres,fMaxOneStep);
	GenerateInflectionPoint(fPointArray,nMaxValIdx,nEndIdx,fInflexionThres,fMaxOneStep);
	}*/
	fVal1=LookForInflectionPoint(fPointArray,nStartIdx,nEndIdx,&nMaxValIdx,&fMaxVal);
	if (nMaxValIdx!=0)
	{
		fVal2=TwoPointsDis(fPointArray[nStartIdx*2],fPointArray[nStartIdx*2+1],fPointArray[nEndIdx*2],fPointArray[nEndIdx*2+1]);
	}
	else
	{
		fVal2=0;
	}

	if(fVal1<fInflexionThres
		&&fVal2<fMaxOneStep)
	{
		return 0;
	}
	else
	{
		m_nInflectionPointsIdxArray[m_nInflectionPointsNum]=nMaxValIdx;
		m_nInflectionPointsNum++;
		GenerateInflectionPoint(fPointArray,nStartIdx,nMaxValIdx,fInflexionThres,fMaxOneStep);
		GenerateInflectionPoint(fPointArray,nMaxValIdx,nEndIdx,fInflexionThres,fMaxOneStep);
	}

	/*	else
	{
	x1=fPointArray[nStartIdx*2];
	y1=fPointArray[nStartIdx*2+1];

	x2=fPointArray[nEndIdx*2];
	y2=fPointArray[nEndIdx*2+1];

	x=x2-x1;
	y=y2-y1;
	fLen=sqrt(x*x+y*y);
	if (fLen>fMaxOneStep)
	{

	if(LookForInflectionPoint(fPointArray,nStartIdx,nEndIdx,&nMaxValIdx,&fMaxVal)>(fInflexionThres/2))
	{
	m_nInflectionPointsIdxArray[m_nInflectionPointsNum]=nMaxValIdx;
	m_nInflectionPointsNum++;
	GenerateInflectionPoint(fPointArray,nStartIdx,nMaxValIdx,fInflexionThres/2,fMaxOneStep);
	GenerateInflectionPoint(fPointArray,nMaxValIdx,nEndIdx,fInflexionThres/2,fMaxOneStep);
	}
	for (i=nStartIdx+1;i<nEndIdx;i++)
	{
	x2=fPointArray[i*2];
	y2=fPointArray[i*2+1];
	x=x2-x1;
	y=y2-y1;
	fTmp=sqrt(x*x+y*y);
	if (fTmp>fMaxOneStep)
	{
	m_nInflectionPointsIdxArray[m_nInflectionPointsNum]=nMaxValIdx;
	m_nInflectionPointsNum++;
	x1=x2;
	y1=y2;
	}
	}
	}
	}*/
	return 0;
}

float OpenGL::LookForInflectionPoint(float *fPointArray,int nStartIdx,int nEndIdx,int *pnMaxValIdx,float *pfMaxVal)
{
	float k,A,B,C,D;
	float fMaxVal,fVal;
	int nMaxValIdx=0,i,nMidPointNum,nCurIdx,j,m,nMidIdx;
	if (nStartIdx>nEndIdx)
	{
		int ddddd=1000;
	}
	if (fabs(fPointArray[nStartIdx*2+1]-fPointArray[nEndIdx*2+1])<0.04)
	{
		fMaxVal=0;


		nMidPointNum=(nEndIdx-nStartIdx);
		if (nMidPointNum==3)
		{
			int gggg=10;
		}
		nMidIdx=nStartIdx+nMidPointNum/2;
		j=0;
		m=-1;
		i=0;
		nCurIdx=nMidIdx;
		nMidPointNum-=2;
		while (i<nMidPointNum)
		{
			if (nCurIdx!=nStartIdx&&nCurIdx!=nEndIdx)
			{
				fVal=fabs(fPointArray[nCurIdx*2+1]-fPointArray[nStartIdx*2+1]);
				if(fVal>fMaxVal)
				{
					fMaxVal=fVal;
					nMaxValIdx=nCurIdx;
				}
				m*=-1;
				j=(i/2)*m;
				nCurIdx=j+nMidIdx;

			}
			i++;
		}

		/*for (i=nStartIdx+1;i<nEndIdx-1;i++)
		{
		fVal=fabs(fPointArray[i*2+1]-fPointArray[nStartIdx*2+1]);
		if(fVal>fMaxVal)
		{
		fMaxVal=fVal;
		nMaxValIdx=i;
		}
		}*/
	}
	else
	{
		k=(fPointArray[nStartIdx*2]-fPointArray[nEndIdx*2])/(fPointArray[nStartIdx*2+1]-fPointArray[nEndIdx*2+1]);
		A=1.0;
		B=-k;
		C=k*fPointArray[nStartIdx*2+1]-fPointArray[nStartIdx*2];
		D=sqrt(A*A+B*B);
		fMaxVal=0;

		nMidPointNum=(nEndIdx-nStartIdx);
		if (nMidPointNum==3)
		{
			int gggg=10;
		}
		nMidIdx=nStartIdx+nMidPointNum/2;
		j=0;
		m=-1;
		i=0;
		nCurIdx=nMidIdx;
		nMidPointNum-=2;
		while (i<nMidPointNum)
		{
			if (nCurIdx!=nStartIdx&&nCurIdx!=nEndIdx)
			{
				fVal=fabs(A*fPointArray[nCurIdx*2]+B*fPointArray[nCurIdx*2+1]+C)/D;
				if(fVal>fMaxVal)
				{
					fMaxVal=fVal;
					nMaxValIdx=nCurIdx;
				}
				m*=-1;
				j=(i/2)*m;
				nCurIdx=j+nMidIdx;

			}
			i++;
		}


		/*	for (i=nStartIdx+1;i<nEndIdx-1;i++)
		{
		fVal=fabs(A*fPointArray[i*2]+B*fPointArray[i*2+1]+C)/D;
		if(fVal>fMaxVal)
		{
		fMaxVal=fVal;
		nMaxValIdx=i;
		}
		}*/
	}

	*pnMaxValIdx=nMaxValIdx;
	*pfMaxVal=fMaxVal;


	return fMaxVal;
}


void OpenGL::bubble_sort(int *x, int n)
{
	int j, k=0, h,t;
	for (h=n-1,k=h; h>0; h--)
	{ 
		for (j=0, k=0; j<h; j++)
		{
			if (*(x+j) > *(x+j+1))
			{ 
				t = *(x+j);
				*(x+j) = *(x+j+1);
				*(x+j+1) = t;
				k = j;
			}
		}
	}
} 

void OpenGL::OnLButtonDoubleClieck(UINT nFlags, CPoint point)
{
	if (m_nManualPath)
	{
		m_nManualPathArray[m_nManualPathArrayLen]=point.x;
		m_nManualPathArrayLen++;
		m_nManualPathArray[m_nManualPathArrayLen]=point.y;
		m_nManualPathArrayLen++;

		//DrawRobotPath();
	}
}

int OpenGL::DrawVitualPath(float *pfSrcPos, float *pfDstPos)
{
	//float fDis=CalDis(pfDstPos[0],pfDstPos[1],pfSrcPos[0],pfSrcPos[1]);
	glEnable(GL_LINE_STIPPLE);//可以虚线，先声明可以使用 
	//glLineWidth (2.0);
	glLineStipple (1, 0xF00F); 
	glColor3f(1.0,1.0,1.0);
	glBegin(GL_LINES);  
	glVertex3f (pfSrcPos[0],KINECT_HEIGHT,pfSrcPos[2]); 
	glVertex3f (pfDstPos[0],KINECT_HEIGHT,pfDstPos[2]); 
	glEnd();

	glDisable(GL_LINE_STIPPLE);

	return  0;
}

float OpenGL::CalDis(float fX1,float fY1,float fX2,float fY2)
{
	return sqrt((fX1-fX2)*(fX1-fX2)+(fY1-fY2)*(fY1-fY2));
}

void OpenGL::CallBack_GetDataFromNetServer(unsigned char *pucRTPData, unsigned int uiDataLen, void *pContext)
{

	printf("CallBack_GetDataFromNetServer IN #####################################\n");
	int i;
	float fX,fZ;
	if (uiDataLen==10)
	{
		for (i=0;i<m_nRealPathPointNum;i++)
		{
			m_fRealPathFlag[i]=-1;
		}
	}
	else if (uiDataLen==18)
	{

		int nMileStoneIdx;

		
		memcpy(&nMileStoneIdx,pucRTPData+9,4);
		printf("nMileStoneIdx   :  %d  \n",nMileStoneIdx);
		m_cMileStoneFlag[nMileStoneIdx]=0;
		
	}
	else
	{
		if (m_nRealPathPointNum==0)
		{
			m_nRealPathPointNum=1;
			m_fRealPathPos[0]=m_dKinectPose[0];
			m_fRealPathPos[1]=m_dKinectPose[2];
		}

		for (i=0;i<m_nRealPathPointNum-1;i++)
		{
			m_fRealPathFlag[i]=-1;
		}

		memcpy(&fX,&pucRTPData[0],4);
		memcpy(&fZ,&pucRTPData[4],4);
		m_fRealPathPos[m_nRealPathPointNum*2]=fX;
		m_fRealPathPos[m_nRealPathPointNum*2+1]=fZ;
		m_nRealPathPointNum++;
	}

}

int OpenGL::DrawReallPath()
{
	int i;
	//
	for (i=0;i<m_nRealPathPointNum;i++)
	{
		glPushMatrix();
	//	glTranslated(m_fMileStonePos[i*3],m_fMileStonePos[i*3+1],m_fMileStonePos[i*3+2]);
		glTranslated(m_fRealPathPos[i*2],KINECT_HEIGHT,m_fRealPathPos[i*2+1]);
		if (m_fRealPathFlag[i]==0)
		{
			glColor3f(0,1.0,1.0);
		}
		else
		{
			glColor3f(0.3,0.3,0);
		}
		glutSolidSphere(0.05,10,10);
		glPopMatrix();
	//	glVertex3f(m_fMileStonePos[i*3],m_fMileStonePos[i*3+1],m_fMileStonePos[i*3+2]);
	}


	glBegin(GL_LINE_STRIP);
	for (i=0;i<m_nRealPathPointNum;i++)
	{
		if (m_fRealPathFlag[i]==0)
		{
			glColor3f(1,0,0);
		}
		else
		{
			glColor3f(0.3,0.3,0.3);
			
		}
		glVertex3f(m_fRealPathPos[i*2],KINECT_HEIGHT,m_fRealPathPos[i*2+1]);

	}
	glEnd();
	glFlush();

	return 0;
}

void OpenGL::CallBack_Session_Planes(unsigned char *pucImg, float* pfVertex, int nCount)
{
	/*
	memcpy(m_PlaneColor+m_nPlanes*3,pucImg,nCount*3);
	float* p_fVertex=m_PlaneVertex+m_nPlanes*3;
	for(int i=0;i<nCount;i++)
	{
	float nVertex=0;
	memcpy(&nVertex,pfVertex,sizeof(float));
	int nNumV=(int)nVertex;
	memcpy(p_fVertex,&nVertex,4); // add head
	p_fVertex+=1;
	pfVertex+=1;
	memcpy(p_fVertex,pfVertex,nNumV*3*4);
	pfVertex+=nNumV*3;
	p_fVertex+=nNumV*3;
	}
	m_nPlanes+=nCount;*/

	memcpy(m_PlaneColor+m_nPlanes*3,pucImg,nCount*3);
	memcpy(m_PlaneVertex+m_nPlanes*3,pfVertex,nCount*3*4);
	m_nPlanes+=nCount;
}

int OpenGL::DrawScaleGridLines()
{
	int i;

	

	glLineWidth(2);
	glEnable(GL_LINE_STIPPLE);//可以虚线，先声明可以使用 
	//glLineWidth (2.0);
	glLineStipple (1, 0x0F0F);  
	glColor3f(0.1,0.2,1.0);
	glBegin(GL_LINES); 
	glColor3f(0.1,0.1,1.0);
	for (i=0;i<18;i++)
	{
		glVertex3f(m_fGridLinesPos[i][0],m_fGridLinesPos[i][1],m_fGridLinesPos[i][2]);
		glVertex3f(m_fGridLinesPos[i][3],m_fGridLinesPos[i][4],m_fGridLinesPos[i][5]);
	}

	glEnd();
	glDisable(GL_LINE_STIPPLE);

	glBegin(GL_LINES); 
	for (i=1;i<4;i++)
	{
		glVertex3f(m_fScaleGridPos[i-1][0],m_fScaleGridPos[i-1][1],m_fScaleGridPos[i-1][2]);
		glVertex3f(m_fScaleGridPos[i][0],m_fScaleGridPos[i][1],m_fScaleGridPos[i][2]);
	}

	glVertex3f(m_fScaleGridPos[0][0],m_fScaleGridPos[0][1],m_fScaleGridPos[0][2]);
	glVertex3f(m_fScaleGridPos[3][0],m_fScaleGridPos[3][1],m_fScaleGridPos[3][2]);
	glEnd();

	



	return 0;
}