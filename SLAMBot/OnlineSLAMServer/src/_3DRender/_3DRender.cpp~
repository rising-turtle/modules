#include "_3DRender.h"
#include <iostream>
#include <fstream>
#include <string.h>
#include <math.h>

static bool m_bSnapSync[SUPPORT_CLEINT_NUM];
static PCDData *m_pstPCDData[SUPPORT_CLEINT_NUM];

GLubyte *_3DRender::m_pucColorBuff;
GLfloat *_3DRender::m_pfPosBuff;
float _3DRender::m_fTranslationX;
float _3DRender::m_fTranslationY;
float _3DRender::m_fTranslationZ;
float _3DRender::m_fRotationX;
float _3DRender::m_fRotationY;
float _3DRender::m_fRotationZ;


int _3DRender::m_nSceenWidth;
int _3DRender::m_nSceenHeight;
float _3DRender::m_fScreenTopX;
float _3DRender::m_fScreenTopY;
int _3DRender::m_nTriangleNum;
int _3DRender::m_nMousePos[2];
int _3DRender::m_nLBD;
int _3DRender::m_nRBD;

bool _3DRender::m_bRenderOneFrame;
PosOffset _3DRender::m_stPosOffset[SUPPORT_CLEINT_NUM];
bool _3DRender::m_bSnapSync[SUPPORT_CLEINT_NUM];
PCDData *_3DRender::m_pstPCDData[SUPPORT_CLEINT_NUM];
int _3DRender::m_nTriangleIdx[SUPPORT_CLEINT_NUM*2];


int _3DRender::m_nClientStates[SUPPORT_CLEINT_NUM];
//Jpeg MapRender::m_CJpeg;

#define WINDOW_WIDTH  640
#define WINDOW_HEIGHT 480

#define  GLUT_WHEEL_UP 3
#define  GLUT_WHEEL_DOWN 4

int _3DRender::PCDDataIn(bool *bHasSync,PCDData **ppstPCD,ClientInfo *pstClientInfo,int *nCurStatus)
{
	int i,j;
	memcpy(m_bSnapSync,bHasSync,sizeof(bool)*SUPPORT_CLEINT_NUM);
	int nTriangleCount=0,nColorCount=0;
	memset(m_nClientStates,0,SUPPORT_CLEINT_NUM*sizeof(int));
	//printf("PCDDataIn!!!\n");

	memcpy(m_stClientInfo,pstClientInfo,sizeof(ClientInfo)*SUPPORT_CLEINT_NUM);
	memset(m_nTriangleIdx,0,sizeof(int)*SUPPORT_CLEINT_NUM*2);
	for(i=0;i<SUPPORT_CLEINT_NUM;i++)
	{
		if(!m_stClientInfo[i].bHasCnc)
		{
			m_nClientStates[i]=-200;
		}
		else
		{
			//printf("m_bSnapSync:%d, %d\n",i,m_bSnapSync[i]);
			if(m_bSnapSync[i])
			{
				//printf("%d, status:%d\n",i,m_nClientStates[i]);
				m_nClientStates[i]=nCurStatus[i];
				//memcpy(m_pstPCDData[i],ppstPCD[i],640*480*sizeof(PCDData));
				//ClientStates[m_stPosOffset[i].nID]=m_stPosOffset[i].nStatus;
				//printf("Client status:%d\n",ClientStates[m_stPosOffset[i].nID]);


				//m_stPosOffset[i].fOffsetParams[0]=pstClientInfo[i].stRegisterInfo.fOffsetParams[0];
				//m_stPosOffset[i].fOffsetParams[1]=pstClientInfo[i].stRegisterInfo.fOffsetParams[1];
				//m_stPosOffset[i].fOffsetParams[2]=pstClientInfo[i].stRegisterInfo.fOffsetParams[2];
				//m_stPosOffset[i].fOffsetParams[3]=pstClientInfo[i].stRegisterInfo.fOffsetParams[3];


				if(m_nClientStates[i]==1)
				{

					m_nTriangleIdx[i*2]=nTriangleCount/3;

					for(j=0;j<640*480;j++)
					{
						if(ppstPCD[i][j].ucR!=0&&ppstPCD[i][j].ucG!=0&&ppstPCD[i][j].ucB!=0)
						{
							m_pfPosBuff[nTriangleCount++]=ppstPCD[i][j].fX;
							m_pfPosBuff[nTriangleCount++]=ppstPCD[i][j].fY;
							m_pfPosBuff[nTriangleCount++]=ppstPCD[i][j].fZ;

							m_pfPosBuff[nTriangleCount++]=ppstPCD[i][j].fX;
							m_pfPosBuff[nTriangleCount++]=ppstPCD[i][j].fY-0.01;
							m_pfPosBuff[nTriangleCount++]=ppstPCD[i][j].fZ-0.01;

							m_pfPosBuff[nTriangleCount++]=ppstPCD[i][j].fX+0.01;
							m_pfPosBuff[nTriangleCount++]=ppstPCD[i][j].fY+0.01;
							m_pfPosBuff[nTriangleCount++]=ppstPCD[i][j].fZ;


							m_pucColorBuff[nColorCount++]=ppstPCD[i][j].ucR;
							m_pucColorBuff[nColorCount++]=ppstPCD[i][j].ucG;
							m_pucColorBuff[nColorCount++]=ppstPCD[i][j].ucB;

							m_pucColorBuff[nColorCount++]=ppstPCD[i][j].ucR;
							m_pucColorBuff[nColorCount++]=ppstPCD[i][j].ucG;
							m_pucColorBuff[nColorCount++]=ppstPCD[i][j].ucB;


							m_pucColorBuff[nColorCount++]=ppstPCD[i][j].ucR;
							m_pucColorBuff[nColorCount++]=ppstPCD[i][j].ucG;
							m_pucColorBuff[nColorCount++]=ppstPCD[i][j].ucB;
						}
					}
					m_nTriangleIdx[i*2+1]=nTriangleCount/3;
				}
			}
			else
			{

				m_nClientStates[i]=-100;
			}
		}
	}


	//printf("PCDDataEnd!!!\n");
	m_nTriangleNum=nTriangleCount/3;
	//printf("Cur m_nTriangleNum:%d\n",m_nTriangleNum);
	return 0;
}

_3DRender::_3DRender()
{
	m_pucColorBuff=new unsigned char[TRI_NUM*3];
	m_pfPosBuff=new float  [TRI_NUM*3];
}

_3DRender::~_3DRender()
{

}

int _3DRender::MapRenderInit()
{
	int i,j;
	FILE * pColorFile,*pVertexFile;
	size_t result;
	char *pcFileBuff1,*pcFileBuff2;
	int nLen1,nLen2;
	bool bJump;

	char cColorData[100];
	char cVertexData[100];

	memset(cColorData,0,100);
	memset(cVertexData,0,100);

	GLfloat *pfPosPtr=NULL;
	GLubyte *pucColorPtr=NULL;

	pfPosPtr=m_pfPosBuff;
	pucColorPtr=m_pucColorBuff;
	m_nTriangleNum=0;

	m_fTranslationX=0;
	m_fTranslationY=0;
	m_fTranslationZ=0;
	//m_fTranslationZ=31;
	//m_fRotationX=21.5;
	m_fRotationX=45;
	m_fRotationY=0;


	m_nSceenWidth=WINDOW_WIDTH;
	m_nSceenHeight=WINDOW_HEIGHT;

	m_fScreenTopY=-0.60;
	m_fScreenTopX=m_fScreenTopY*m_nSceenWidth/m_nSceenHeight;

	m_nMousePos[0]=0;
	m_nMousePos[1]=0;
	m_nLBD=0;
	m_nRBD=0;

	for(i=0;i<SUPPORT_CLEINT_NUM;i++)
	{
		m_pstPCDData[i]=new PCDData[640*480];
		m_bSnapSync[i]=0;
	}

	m_bRenderOneFrame=true;

	m_stPosOffset[3].fOffsetParams[0]=0;
	m_stPosOffset[3].fOffsetParams[1]=0;
	m_stPosOffset[3].fOffsetParams[2]=0;
	m_stPosOffset[3].fOffsetParams[3]=0;


	m_stPosOffset[0].fOffsetParams[0]=0;
	m_stPosOffset[0].fOffsetParams[1]=-0.61;
	m_stPosOffset[0].fOffsetParams[2]=0;
	m_stPosOffset[0].fOffsetParams[3]=0;
	return 0;
}



int _3DRender::GetPos(char *pcTxt,GLfloat *pfPos)
{
	int i,j=0,k;
	char cData[10];
	float fVal;
	for(i=0;i<9;i++)
	{
		k=j;
		while(pcTxt[j]!=' '&&pcTxt[j]!='\n')
		{
			j++;
		}

		memset(cData,0,10);
		memcpy(cData,&pcTxt[k],j-k);
		fVal=atof(cData);
		pfPos[i]=fVal;
		//printf("idx:%d,%f\n",i,pfPos[i]);
		j++;
	}
	return 0;
}


int _3DRender::GetColor(char *pcTxt,GLubyte *pusPos)
{
	int i,j=0,k;
	char cData[10];
	for(i=0;i<9;i++)
	{
		k=j;
		while(pcTxt[j]!=' '&&pcTxt[j]!='\n')
		{
			j++;
		}
		memset(cData,0,10);
		memcpy(cData,&pcTxt[k],j-k);
		pusPos[i]=atoi(cData);
		j++;
	}
	return 0;
}

int _3DRender::DrawPCD()
{
	int i,j;

	for(i=0;i<SUPPORT_CLEINT_NUM;i++)
	{
		if(m_bSnapSync[i])
		{
			for(j=0;j<640*480;j=j+4)
			{
				glPushMatrix();
				glTranslated(m_pstPCDData[i][j].fX,m_pstPCDData[i][j].fY,m_pstPCDData[i][j].fZ);
				glColor3ub(m_pstPCDData[i][j].ucR, m_pstPCDData[i][j].ucG, m_pstPCDData[i][j].ucB);
				glutSolidSphere(0.01,3,3);
				glPopMatrix();
			}
		}
	}
}
int g_nImgIdx=0;
void _3DRender::RenderScene()
{
	int i,j,k;
	int m,n;

	//char *pcText="Camera #";

	char cTmp[100];

	//if(m_bRenderOneFrame)
	{
		glLoadIdentity ();
		gluLookAt(0,0,-2,
				0,0,1,
				0,1,0);
		glClear(GL_COLOR_BUFFER_BIT);

		for(i=0;i<6;i++)
		{
			if(m_nClientStates[i]==1)
			{
				memset(cTmp,0,100);
				sprintf(cTmp,"Camera #:  %d : Connecting",i);
				ShowText(cTmp,1.4,1-0.1*i,1,1);
			}
			else if(m_nClientStates[i]==-1)
			{
				memset(cTmp,0,100);
				sprintf(cTmp,"Camera #:  %d : Xtion Err",i);
				ShowText(cTmp,1.4,1-0.1*i,1,2);
			}
			else if(m_nClientStates[i]==-200)
			{
				memset(cTmp,0,100);
				sprintf(cTmp,"Camera #:  %d : Lost",i);
				ShowText(cTmp,1.4,1-0.1*i,1,3);
			}
			else if(m_nClientStates[i]==-100)
			{
				memset(cTmp,0,100);
				sprintf(cTmp,"Camera #:  %d : Sync Failed",i);
				ShowText(cTmp,1.4,1-0.1*i,1,5);
			}
		}

		glEnableClientState(GL_COLOR_ARRAY);
		glEnableClientState(GL_VERTEX_ARRAY);
		glColorPointer(3,GL_UNSIGNED_BYTE,0,m_pucColorBuff);
		glVertexPointer(3,GL_FLOAT,0,m_pfPosBuff);

		glPushMatrix();



		glTranslated(m_fTranslationX, m_fTranslationY, m_fTranslationZ);
		glRotated(m_fRotationZ, 0, 0, 1);
		glRotated(m_fRotationY, 0, 1, 0);
		glRotated(m_fRotationX, 1, 0, 0);



		for(m=0;m<SUPPORT_CLEINT_NUM;m++)
		{
			if(m_bSnapSync[m])
			{
				glPushMatrix();

				if(m==3)
				{
					m_stPosOffset[m].fOffsetParams[1]=0.61;		
				}

				printf("ID:%d, X:%f,Y:%f,Z:%f\n",m,
						m_stPosOffset[m].fOffsetParams[0],
						m_stPosOffset[m].fOffsetParams[1],
						m_stPosOffset[m].fOffsetParams[2]);

				glTranslated(m_stPosOffset[m].fOffsetParams[0],
						m_stPosOffset[m].fOffsetParams[1],
						m_stPosOffset[m].fOffsetParams[2]);



				glRotated(m_stPosOffset[m].fOffsetParams[3], 0, 0, 1);
				glBegin(GL_TRIANGLES);
				for (i=m_nTriangleIdx[m*2];i<m_nTriangleIdx[m*2+1];i++)
				{
					glArrayElement(i);
				}
				glEnd();

				glPopMatrix();
			}
		}


		//DrawPCD();

		glLineWidth( 3 );//ÉèÖÃÏßµÄ¿í¶ÈÎª6
		glBegin(GL_LINES);

		glColor3f(1.0,0.0,0.0);
		glVertex3f(0.0,0,0);
		glVertex3f(1,0,0);

		glColor3f(0.0,1.0,0.0);
		glVertex3f(0.0,0,0);
		glVertex3f(0.0,1,0);

		glColor3f(0.0,0.0,1.0);
		glVertex3f(0.0,0,0);
		glVertex3f(0.0,0,1);
		glEnd();

		glPopMatrix();



		glutSwapBuffers();
		//m_bRenderOneFrame=false;
	}
	usleep(25000);
}


GLvoid _3DRender::ReSizeGLScene(GLsizei Width, GLsizei Height)
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

	gluLookAt(0,0,-2,
			0,0,1,
			0,1,0);

	//MoveWindow(hWnd,0,0,Width,Height,false);

	m_fScreenTopY=-0.60;
	m_fScreenTopX=m_fScreenTopY*Width/Height;
}


void _3DRender::ProcessMouse(int button, int state, int x, int y)
{
	// Estimate
	// TicToc tt;

	if(button == GLUT_LEFT_BUTTON)
	{
		if(state == GLUT_UP)
		{
			m_nLBD=0;
			m_nMousePos[0]=0;
			m_nMousePos[1]=0;
		}
		else if(state == GLUT_DOWN)
		{
			m_nLBD=1;
			m_nMousePos[0]=x;
			m_nMousePos[1]=y;
		}
	}
	else if(button == GLUT_RIGHT_BUTTON)
	{
		if(state == GLUT_UP)
		{
			m_nRBD=0;
			m_nMousePos[0]=0;
			m_nMousePos[1]=0;
		}
		else if(state == GLUT_DOWN)
		{
			m_nRBD=1;
			m_nMousePos[0]=x;
			m_nMousePos[1]=y;
		}
	}
	else if(button == GLUT_WHEEL_UP)
	{
		m_fTranslationZ+=0.8;
	}
	else if(button == GLUT_WHEEL_DOWN)
	{
		m_fTranslationZ-=0.8;
	}
}


void _3DRender::MouseCB(int x, int y)
{

	if(m_nLBD==1)
	{
		GLdouble dx = m_nMousePos[0]-x;
		GLdouble dy = y-m_nMousePos[1];
		m_fRotationY += 360 * dx / m_nSceenWidth;
		m_fRotationX += 360 * dy / m_nSceenHeight;
	}
	else if (m_nRBD==1)
	{
		GLdouble dx = x-m_nMousePos[0];
		GLdouble dy = y -m_nMousePos[1];
		m_fTranslationX -= 10 * dx / m_nSceenWidth;
		m_fTranslationY -= 10 * dy / m_nSceenHeight;
	}
	m_nMousePos[0]=x;
	m_nMousePos[1]=y;
}



int _3DRender::MapRenderRun()
{
	/* GLUT环境初始化*/
	int nArgc=0;
	glutInit(&nArgc, NULL);

	glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE);


	glutInitWindowPosition(0, 0);

	glutInitWindowSize(WINDOW_WIDTH, WINDOW_HEIGHT);


	glutCreateWindow("OpenGL 3D View");

	InitGL(WINDOW_WIDTH, WINDOW_HEIGHT);

	glutDisplayFunc(RenderScene);
	glutIdleFunc(RenderScene);
	glutReshapeFunc (ReSizeGLScene);


	glutMouseFunc(ProcessMouse);
	glutMotionFunc(MouseCB);

	glutMainLoop( );

	return 0;
}






int _3DRender::MapRenderStop()
{
	return 0;
}
GLvoid _3DRender::InitGL(GLsizei Width, GLsizei Height)	// This Will Be Called Right After The GL Window Is Created
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
	gluLookAt(0,0,-2,
			0,0,1,
			0,1,0);
	m_fScreenTopY=-0.60;
	m_fScreenTopX=m_fScreenTopY*Width/Height;
}


const unsigned g_PenColor[][3]={
		{61,  89,  171},   /* 0. 钴色   */
		{48,  128, 20 },   /* 1. 暗绿色 */
		{30,  144, 255},   /* 2. dodger blue */
		{255, 97,  0  },   /* 3. 橙色   */
		{176, 23,  31 },   /* 4. 印度红 */
		{135, 38,  87 },   /* 5. 草莓色 */
		{255, 215, 0  },   /* 6. 金黄色 */
		{11,  23,  70 }    /* 7. jackie blue */
};


void _3DRender::ShowText(char *text_toshow, double x, double y,double z, int colorid)
{
	/* There are 7 bitmap fonts available in GLUT. They are
		GLUT_BITMAP_8_BY_13
		GLUT_BITMAP_9_BY_15
		GLUT_BITMAP_TIMES_ROMAN_10
		GLUT_BITMAP_TIMES_ROMAN_24
		GLUT_BITMAP_HELVETICA_10
		GLUT_BITMAP_HELVETICA_12
		GLUT_BITMAP_HELVETICA_18 */
	int i;
	void * font = GLUT_BITMAP_HELVETICA_18;
	glColor3ub( g_PenColor[colorid][0],
			g_PenColor[colorid][1],
			g_PenColor[colorid][2]);
	glRasterPos3f(x, y,z);
	for (i=0; i<=strlen(text_toshow); ++i) {
		char c = text_toshow[i];
		glutBitmapCharacter(font, c);

	}

}
