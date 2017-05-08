#pragma once
#include <windows.h>	// Header File For Windows
#include <gl\gl.h>		// Header File For The OpenGL32 Library
#include <gl\glu.h>		// Header File For The GLu32 Library
#include <vector>
#include "IoTRobot_Internal_Define.h"
using namespace std;

#define KINECT_HEIGHT 0.5
#define MAX_ONE_STEP_LEN 0.75
#define MIN_INFLEXION_LEN 0.2

#define MAX_CHAR       128
#define MIN_PACE_LOCK 0.3
class OpenGL
{
public:
	OpenGL(void);
	~OpenGL(void);

	static GLubyte *m_pucColor;
	static GLfloat *m_pfPC;


	//static GLubyte *m_pucPIPColor;
	//static GLfloat *m_pfPIPVertex;

	static GLubyte *m_pucPIP_CIFColor;
	static GLfloat *m_pfPIP_CIFVertex;

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
	static void CallBack_RunState(char *cState);
	static void CallBack_GetDataFromNetServer(unsigned char*pucRTPData,unsigned int uiDataLen,void *pContext);

	HGLRC m_hRC;    //Rendering Context
	CDC* m_pDC;        //Device Context
	BOOL InitializeOpenGL();    //Initialize OpenGL
	BOOL SetupPixelFormat();    //Set up the Pixel Format
	void RenderScene();            //Render the Scene
	void RenderCharactors();
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
	static void OnLButtonDoubleClieck(UINT nFlags, CPoint point);
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
	static bool m_bCompass;
	static bool m_bTriangulation;
	static int CreatePath();
	static int AddPathNode(float *pfTMat,float *pfRAngle);
	static int DrawPath();
	static int DeletePath();
	static int PathTwinkle();
	static int DrawCurrentPose();

	static int DrawArrow();
	static int DrawChassisPose();

	static IoTRobot_PathTwinkleParam m_stPathTwinkle;


	static bool m_bNoIMUInfo;
	static bool m_bUseIMU;

	static bool m_bStopOpenGL;

	static float m_fScreenTopX;
	static float m_fScreenTopY;

	void selectFont(int size, int charset, const char* face);
	void drawString(const char* str);
	void drawStringForScaleGrid(const char* str);
	void selectFontForScaleGrid(int size, int charset, const char* face);

	void DrawString(const char* str,int nIdx);

	static void *m_pOpenGL;
	static char m_cNodeNum[200];


	static float m_fCircle[60];
	static double m_dChassisPose[3];
	static double m_dKinectPose[6];


	static int m_nFirstPoseRcd;
	static double m_dFirstPose[3];


	//Manual Path
	static int DrawMotionPath();
	static int m_nManualPath;
	static int m_nManualPathArray[10000];
	static float m_fPathArray[10000];
	static int m_nManualPathArrayLen;
	static int m_nSceenWidth;
	static int m_nSceenHeight;

	//Robot Path

	static int DrawRobotPath();
	static int BuildMileStone();
	static int m_nRobotPathArray[300];
	static int m_nRobotPathArrayLen;
	static int GenerateInflectionPoint(float *fPointArray,int nStartIdx,int nEndIdx,float fInflexionThres,float fMaxOneStep);
	static float LookForInflectionPoint(float *fPointArray,int nStartIdx,int nEndIdx,int *pnMaxValIdx,float *pfMaxVal);

	static int m_nInflectionPointsIdxArray[300];
	static int m_nInflectionPointsNum;

	//MileStone 
	static int DrawMileStone();
	static float m_fMileStonePos[300];
	static float m_cMileStoneFlag[100];
	static int m_nMileStoneLen;
	static int m_nReachMileStoneIdx;
	static float m_fThres;
	static int m_nMileStoneHasBeenBuild;


	static float TwoPointsDis(float x1,float y1,float x2,float y2);
	static void bubble_sort(int *x, int n);


	static UINT ThreadTest(LPDWORD param);


	//Path explore
	static int m_nHorizonCutPC;
	static int DrawVitualPath(float *pfSrcPos,float *pfDstPos);
	float CalDis(float fX1,float fY1,float fX2,float fY2);
	static int m_nDrawVitualLine;

	static int DrawReallPath();
	static int m_nRealPathPointNum;
	static float m_fRealPathPos[200];
	static float m_fRealPathFlag[100];


	////Draw Planes
	static int m_nPlanes;
	static GLubyte* m_PlaneColor;
	static GLfloat* m_PlaneVertex;
	static void CallBack_Session_Planes(unsigned char *pucImg, float* pfVertex, int nCount);
	static std::vector<GLuint> m_call_list;
	static int m_index_of_call_list;


	//Draw Scale Grid Lines
	static bool m_bOpenScaleGridLines;
	static float m_fGridLinesPos[18][6];
	static int DrawScaleGridLines();
	static float m_fScaleGridPos[4][3];
	static char m_cScale[200];
	
//	static vector <IoTRobot_GLPathNode>  m_vcGLPath;
};
