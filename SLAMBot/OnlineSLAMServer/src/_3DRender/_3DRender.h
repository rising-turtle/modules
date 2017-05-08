#pragma once
#include <GL/glew.h>
#include <GL/freeglut.h>
#include <vector>
#include <string>
#include "../Basic_Define.h"

#define TRI_NUM 10000000
#define MAX_PLAN_NUM 10
#define MAX_VERTEX_NUM 24
#define MAX_DIS 0.5
#define ROBOT_HEIGHT 0
using namespace std;
class _3DRender
{
public:
	_3DRender();
	~_3DRender();

	int PCDDataIn(bool *bHasSync,PCDData **ppstPCD,ClientInfo *pstClientInfo,int *nCurStatus);
	int MapRenderInit();
	int MapRenderRun();
	int MapRenderStop();
private:

	static GLubyte *m_pucColorBuff;
	static GLfloat *m_pfPosBuff;
	static float m_fTranslationX, m_fTranslationY, m_fTranslationZ;
	static float m_fRotationX, m_fRotationY,m_fRotationZ;
	static int m_nSceenWidth,m_nSceenHeight;
	static float m_fScreenTopX,m_fScreenTopY;
	static int m_nTriangleNum;
	static int DrawPCD();

	static int m_nMousePos[2];
	static int m_nLBD;
	static int m_nRBD;




	int GetPos(char *pcTxt,GLfloat *pfPos);
	int GetColor(char *pcTxt,GLubyte *pusPos);
	static void RenderScene();
	static GLvoid ReSizeGLScene(GLsizei Width, GLsizei Height);
	static void ProcessMouse(int button, int state, int x, int y);
	static void MouseCB(int x, int y);

	GLvoid InitGL(GLsizei Width, GLsizei Height);

	static bool m_bSnapSync[SUPPORT_CLEINT_NUM];
	static PCDData *m_pstPCDData[SUPPORT_CLEINT_NUM];
	static PosOffset m_stPosOffset[SUPPORT_CLEINT_NUM];
	static bool m_bRenderOneFrame;


	static int m_nTriangleIdx[SUPPORT_CLEINT_NUM*2];

	static void ShowText(char *text_toshow, double x, double y,double z, int colorid);


	ClientInfo m_stClientInfo[SUPPORT_CLEINT_NUM];
	static int m_nClientStates[SUPPORT_CLEINT_NUM];

};
