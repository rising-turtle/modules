#pragma once
#include <windows.h>	// Header File For Windows
#include <gl\gl.h>		// Header File For The OpenGL32 Library
#include <gl\glu.h>		// Header File For The GLu32 Library
#include <vector>
#include "IoTRobot_Internal_Define.h"
using namespace std;
class OpenGL
{
public:
	OpenGL(void);
	~OpenGL(void);

	static GLubyte *m_pucColor;
	static GLfloat *m_pfPC;


	static GLubyte *m_pucPIPColor;
	static GLfloat *m_pfPIPVertex;

	static GLdouble m_dRotationX ;
	static GLdouble m_dRotationY;
	static GLdouble m_dTranslationX ;
	static GLdouble m_dTranslationY ;
	static GLdouble m_dTranslationZ ;


	static GLvoid InitGL(GLsizei Width, GLsizei Height);
	static GLvoid ReSizeGLScene(GLsizei Width, GLsizei Height);
	static GLvoid DrawGLScene(GLvoid);
	static LRESULT CALLBACK WndProc(	HWND	hWnd,
		UINT	message,
		WPARAM	wParam,
		LPARAM	lParam);
	int OpenlInterface(HWND hParent,HINSTANCE	hInstance);

	static void CallBack_PIPQVGA(unsigned char *pucQVGAImg,void *pContext);
	static void CallBack_PointCloud(unsigned char *pucImg,float *pfVertex,void *pContext);
	static void CallBack_Path(float *pfTMat,float *pfRAngle,void *pContext);
	static void CallBack_RunState(char cState);


	HGLRC m_hRC;    //Rendering Context
	CDC* m_pDC;        //Device Context
	BOOL InitializeOpenGL();    //Initialize OpenGL
	BOOL SetupPixelFormat();    //Set up the Pixel Format
	void RenderScene();            //Render the Scene
//	void DrawPath();


	static bool m_bOpenPIP;
	static bool m_bStopRender;
	static LPDWORD ThreadRenderID;
	static CPoint m_MouseDownPoint;



	static MouseDragMode m_enumMouseDrag ;
	static CPoint m_nWndSize;

	static void OnKeyDown(UINT nChar, UINT nRepCnt, UINT nFlags);
	static void OnLButtonDown(UINT nFlags, CPoint point);
	static void OnLButtonUp(UINT nFlags, CPoint point);
	static void OnRButtonDown(UINT nFlags, CPoint point);
	static void OnRButtonUp(UINT nFlags, CPoint point);
	static void OnMouseMove(UINT nFlags, CPoint point);
	static BOOL OnMouseWheel(UINT   nFlags,short zDelta,CPoint pt);

	


////SLAM

	static GLubyte *m_pucSLAMColor;
	static GLfloat *m_pfSLAMPC;
	static int m_nCurrentCount;
	static void CallBack_SLAM_PC(unsigned char *pucImg,float *pfVertex,void *pContext,int nCount);
	void RenderSLAM();            //Render the Scene

//	static GLfloat *m_pfCircle;
	static bool m_bOpenStateLight;
	static GLbyte m_cRunState;


	static IoTRobot_GLPath m_stGLPath;

	static bool m_bStart2AddNode;
	static bool m_bPathTwinkle;
	static bool m_bShowPath;
	static int CreatePath();
	static int AddPathNode(float *pfTMat,float *pfRAngle);
	static int DrawPath();
	static int DeletePath();
	static int PathTwinkle();

	static IoTRobot_PathTwinkleParam m_stPathTwinkle;


	static bool m_bNoIMUInfo;
	static bool m_bUseIMU;

//	static vector <IoTRobot_GLPathNode>  m_vcGLPath;
};
