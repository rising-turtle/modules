//#include "stdafx.h"
#include "SLAM.h"
#include "CommandAndControl.h"
#include "Point_Cloud.h"
#include "Openni.h"
#include <time.h>
bool CSLAM::m_bFrameReady;
bool CSLAM::m_bFinishFrame;
unsigned char *CSLAM::m_pucRGBBuff;
unsigned short *CSLAM::m_pusDepth;
bool g_bStartToRun=false;

int CSLAM::m_nCount;
unsigned int CSLAM::m_uiFrameNum;
CallBack_RunState CSLAM::m_cCallBackState;
double *CSLAM::m_pdIMUData;
extern DWORD g_ulRunTimeBuff[LOG_ARRAY_LEN][8];
int CSLAM::m_nApplyIMU;
int CSLAM::m_nApplyIMU2;
int CSLAM::m_nCheckIMU;
CSLAM::CSLAM()
{
	m_nCount=0;
	m_nApplyIMU2=1;
	m_nApplyIMU=0;
}


CSLAM::~CSLAM()
{

}

void CSLAM::CallBack_XYZRGB(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud,void *pContext)
{

}


UINT CSLAM::ThreadRGBDDynamicReceive(LPVOID lpParam)
{
	int nRtn=0,i;
	CSLAM *pCSLAM=(CSLAM *)lpParam;
	COpenNI *pOpenNI=(COpenNI *)pCSLAM->m_stClassPtrs.pOpenNi;
	struct RGBDData *pBuffList=pCSLAM->m_pstRGBDDataListHead;

	for (i=0;i<MAX_STORE_BUFF_LEN;i++)
	{
		pCSLAM->m_pstRGBDDataCurWrite=pBuffList;
		pCSLAM->m_pucRGBBuff=pBuffList->pucRGB;
		pCSLAM->m_pusDepth=pBuffList->pusDepth;
		pCSLAM->m_pdIMUData=pBuffList->dIMUData;
		while(pOpenNI->GetOneFrame()!=0)
		{
			Sleep(10);
		}
		pCSLAM->m_bFrameReady=false;
		while (!pCSLAM->m_bFrameReady)
		{
			Sleep(5);
		}
		pBuffList->bReady=true;
		pBuffList=pBuffList->pstNext;
		pCSLAM->m_nLeftBuffLen++;
		Sleep(30);
	}

	pCSLAM->m_nFirstFrame=1;
	while (1)
	{
		while (pCSLAM->m_nLeftBuffLen<MAX_STORE_BUFF_LEN)
		{
			if (!pBuffList->bReady&&!pBuffList->bUse)
			{
				//printf("List num reci:  %d  \n",pBuffList->nIdx);
				pCSLAM->m_pstRGBDDataCurWrite=pBuffList;
				pCSLAM->m_pucRGBBuff=pBuffList->pucRGB;
				pCSLAM->m_pusDepth=pBuffList->pusDepth;
				pCSLAM->m_pdIMUData=pBuffList->dIMUData;
				while(pOpenNI->GetOneFrame()!=0)
				{
					Sleep(10);
				}
				pCSLAM->m_bFrameReady=false;
				while (!pCSLAM->m_bFrameReady)
				{
					Sleep(5);
				}
				pBuffList->bReady=true;
				pBuffList=pBuffList->pstNext;
				pCSLAM->m_nLeftBuffLen++;
			}
		}
		Sleep(30);
	}
}


UINT CSLAM::ThreadRGBDDynamicManage(LPVOID lpParam)
{
	int nRtn=0;
	CSLAM *pCSLAM=(CSLAM *)lpParam;
	COpenNI *pOpenNI=(COpenNI *)pCSLAM->m_stClassPtrs.pOpenNi;
	int i,nCurrentBuffLen=0;
	pCSLAM->m_pstRGBDDataReady=pCSLAM->m_pstRGBDDataListHead;

	while (!pCSLAM->m_nFirstFrame)
	{
		Sleep(10);
	}
	while (1)
	{
		if (pCSLAM->m_bIWanaOneFrame)
		{
			if (pCSLAM->m_pstRGBDDataReady->bReady)
			{
				
				//pCSLAM->m_bIWanaOneFrame=false;
				pCSLAM->m_pstRGBDDataReady->bUse=1;
				//printf("List num mana1:  %d  \n",pCSLAM->m_pstRGBDDataReady->nIdx);
				pCSLAM->m_pstRGBDDataCurRead=pCSLAM->m_pstRGBDDataReady;
				//printf("List num mana2:  %d  \n",pCSLAM->m_pstRGBDDataCurRead->nIdx);
				pCSLAM->m_pstRGBDDataReady=pCSLAM->m_pstRGBDDataReady->pstNext;
				//printf("List num mana3:  %d  \n",pCSLAM->m_pstRGBDDataCurRead->nIdx);
				pCSLAM->m_nLeftBuffLen--;
				pCSLAM->m_bIWanaOneFrame=false;
			}
		}
		Sleep(5);
	}
}






CSLAM::CSLAM(void *pCCmdCtrl)
{
	m_pCCmdCtrl=pCCmdCtrl;
}



void CSLAM::NewCommand(char *pCmd)
{
	m_stAlgOpt.cNewCmdFlag=1;
	memcpy(&m_stAlgOpt.cAlgorithmFlag,pCmd,ALGORITHM_NUM);
}

int CSLAM::CreateRGBDDataBuffList(int nLen)
{
	int i;
	struct RGBDData *pList=NULL;
	m_pstRGBDDataListHead=new struct RGBDData;
	m_pstRGBDDataListHead->nIdx=0;
	m_pstRGBDDataListHead->bReady=0;
	m_pstRGBDDataListHead->bUse=0;
	m_pstRGBDDataListHead->pucRGB=new unsigned char[640*480*3];
	m_pstRGBDDataListHead->pusDepth=new unsigned short [640*480];

	memset(m_pstRGBDDataListHead->pucRGB,0,640*480*3);
	memset(m_pstRGBDDataListHead->pusDepth,0,640*480*2);

	pList=m_pstRGBDDataListHead;
	for (i=0;i<nLen-1;i++)
	{
		struct RGBDData *pTmp=new struct RGBDData;
		pTmp->nIdx=i+1;
		pTmp->bUse=0;
		pTmp->bReady=0;
		pTmp->pucRGB=new unsigned char[640*480*3];
		pTmp->pusDepth=new unsigned short [640*480];

		memset(pTmp->pucRGB,0,640*480*3);
		memset(pTmp->pusDepth,0,640*480*2);
		pList->pstNext=pTmp;
		pList=pTmp;
	}

	pList->pstNext=m_pstRGBDDataListHead;
	return 1;
}

int CSLAM::DeleteRGBDDataBuffList(int nLen)
{
	int i;
	struct RGBDData *pList=m_pstRGBDDataListHead;
	struct RGBDData *pTmp=NULL;
	for (i=0;i<nLen-1;i++)
	{
		pTmp=pList;
		pList=pList->pstNext;
		delete [] pTmp->pucRGB;
		delete [] pTmp->pusDepth;
		delete pTmp;
	}
	delete [] pList->pucRGB;
	delete [] pList->pusDepth;
	delete pList;
	return 0;
}


void CSLAM::SLAMInit(IoT_CallBackFuncSet *pstCallBackFuncSet)
{
	int i;
	//	pstCallBackFuncSet->callBack_XYZRGB=CallBack_XYZRGB;
	pstCallBackFuncSet->callBack_RGB24_Depth=CallBack_RGB24_Depth;
	pstCallBackFuncSet->pXYZRGBContext=this;
	m_nLeftBuffLen=0;
	CreateRGBDDataBuffList(RGBD_DATA_BUFF_LEN);
	m_pstRGBDDataCurRead=m_pstRGBDDataListHead;
	m_cRGBDSlam.IoTRobot_RGBDSLAM_Init();
	m_bFinishFrame=1;
	m_uiFrameNum=0;
	m_nFirstFrame=0;
	m_nCheckIMU=0;


//	m_pucRGBBuff=new unsigned char[640*480*3];
	//m_pusDepth=new unsigned short[640*480];

	m_pfVertex=new float[640*480*3];
}
void CSLAM::SLAMUninit()
{
	DeleteRGBDDataBuffList(RGBD_DATA_BUFF_LEN);
}

int CSLAM::TestPC(unsigned short *psDepth,float *pfPC)
{
	int i,j,k,m;
	m=0;
	k=0;
	for (i=-240;i<240;i++)
	{
		for (j=-320;j<320;j++)
		{
			if (psDepth[m]!=0)
			{
				pfPC[k+2]=1.0*(float)psDepth[m]/1000.0;
				pfPC[k]=(float)j*0.001904*pfPC[k+2];
				pfPC[k+1]=(float)i*0.001904*pfPC[k+2];
			}
			else
			{
				pfPC[k+2]=-1000000;
				pfPC[k]=-1000000;
				pfPC[k+1]=-100000;
			}



			if (pfPC[k+1]>0)
			{
				int fff=100;
			}
			k+=3;
			m++;

		}
	}
	return 0;
}





void CSLAM::CallBack_RGB24_Depth(unsigned char *pucRGB24,unsigned short *pusDepth,void *pContext)
{
	//printf("SLAM callback run!!!! \n");
	//	if (m_bFinishFrame)
	{
		m_bFinishFrame=0;
		g_bStartToRun=1;
		memcpy(m_pucRGBBuff,pucRGB24,640*480*3);
		memcpy(m_pusDepth,pusDepth,640*480*2);
		m_bFrameReady=true;

	}
}

void CSLAM::CallBack_SLAM_IMU(unsigned char*pucRGB,unsigned short *pusDepth, char *pcIMUData,void *pContext)
{
	m_bFinishFrame=0;
	g_bStartToRun=1;

	memcpy(m_pucRGBBuff,pucRGB,640*480*3);
	memcpy(m_pusDepth,pusDepth,640*480*2);
	memcpy(m_pdIMUData,pcIMUData,48);

	if (m_nCheckIMU==0)
	{
		if (m_pdIMUData[0]==6&&
			m_pdIMUData[1]==5&&
			m_pdIMUData[2]==4&&
			m_pdIMUData[3]==3&&
			m_pdIMUData[4]==2&&
			m_pdIMUData[5]==1)
		{
			char cTmp=1;
			m_nApplyIMU2=0;
			m_nApplyIMU=0;
			m_cCallBackState(cTmp);
		}
		else
		{

			char cTmp=2;
			m_cCallBackState(cTmp);
			m_nApplyIMU2=1;
			m_nApplyIMU=1;
		}
		m_nCheckIMU=1;
	}
	

	m_bFrameReady=true;
}



UINT CSLAM::ThreadRGBDSlam(LPVOID lpParam)
{
	int nRtn=0;
	CSLAM *pCSLAM=(CSLAM *)lpParam;
	CPoint_Cloud  *pPointCould=(CPoint_Cloud *)pCSLAM->m_stClassPtrs.pPointCloud;
	COpenNI *pOpenNI=(COpenNI *)pCSLAM->m_stClassPtrs.pOpenNi;
	//int nRunTimeIdx=0;
	DWORD *pulRunTimeId=NULL;
	DWORD ulRunTime[6];
	bool bProcessOneFrame;

	while (!pCSLAM->m_nFirstFrame)
	{
		Sleep(10);
	}


	while (1)
	{
		pCSLAM->m_bIWanaOneFrame=1;

		while (pCSLAM->m_bIWanaOneFrame)
		{
			Sleep(5);
		}
	//	printf("List num use:  %d  \n",pCSLAM->m_pstRGBDDataCurRead->nIdx);
		//printf("pCSLAM->m_nLeftBuffLen  :   %d  \n",pCSLAM->m_nLeftBuffLen);
		g_ulRunTimeBuff[m_nCount][1]=::GetTickCount();
		boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> point_cloud(new pcl::PointCloud<pcl::PointXYZRGB > ());
		Eigen::Matrix4f final_transformation;


	/*	printf("*********IMU DATA:   %f   %f   %f   %f   %f   %f   \n",pCSLAM->m_pstRGBDDataCurRead->dIMUData[0],
			pCSLAM->m_pstRGBDDataCurRead->dIMUData[1],
			pCSLAM->m_pstRGBDDataCurRead->dIMUData[2],
			pCSLAM->m_pstRGBDDataCurRead->dIMUData[3],
			pCSLAM->m_pstRGBDDataCurRead->dIMUData[4],
			pCSLAM->m_pstRGBDDataCurRead->dIMUData[5]);*/


		//printf("IIIIIIIIIIIIIIIIIIIIMU  %d \n",m_nApplyIMU);
		//printf("IIIIIIIIIIIIIIIIIIIIMU2222222222222222  %d \n",m_nApplyIMU2);

		nRtn=pCSLAM->m_cRGBDSlam.IoTRobot_RGBDSLAM_RunOneFrame(pCSLAM->m_pstRGBDDataCurRead->pucRGB,
			pCSLAM->m_pstRGBDDataCurRead->pusDepth,
			point_cloud,final_transformation,
			&g_ulRunTimeBuff[m_nCount][2],pCSLAM->m_pstRGBDDataCurRead->dIMUData,m_nApplyIMU);
		//	pCSLAM->TestPC(pCSLAM->m_pstRGBDDataCurRead->pusDepth,pCSLAM->m_pfVertex);
		//	pCSLAM->m_cbPointCould(pCSLAM->m_pstRGBDDataCurRead->pucRGB,pCSLAM->m_pfVertex,NULL);




		//printf("SLAM DLL PASS!!!!   \n");
		g_ulRunTimeBuff[m_nCount][3]=::GetTickCount();

		m_cCallBackState((char) nRtn);
		if (nRtn==0)
		{
			while (pPointCould->m_bNewFrame)
			{
				Sleep(1);
			}
			g_ulRunTimeBuff[m_nCount][4]=::GetTickCount();
			pPointCould->NewFrameCloudIn(point_cloud,&final_transformation);

			g_ulRunTimeBuff[m_nCount][7]=m_uiFrameNum;
			m_nCount++;
			m_uiFrameNum++;


			if (m_nCount==LOG_ARRAY_LEN)
			{
				m_nCount=0;
			}
			bProcessOneFrame=true;
		}
		g_ulRunTimeBuff[m_nCount][6]=::GetTickCount();
		pCSLAM->m_pstRGBDDataCurRead->bUse=false;
		pCSLAM->m_pstRGBDDataCurRead->bReady=false;
		Sleep(100);
	}

	return 0;
}
/*

void CSLAM::CallBack_RGB24_Depth(unsigned char *pucRGB24,unsigned short *pusDepth,void *pContext)
{
//printf("SLAM callback run!!!! \n");
//	if (m_bFinishFrame)
{
m_bFinishFrame=0;
g_bStartToRun=1;
memcpy(m_pucRGBBuff,pucRGB24,640*480*3);
memcpy(m_pusDepth,pusDepth,640*480*2);
m_bFrameReady=true;

}
}

UINT CSLAM::ThreadRGBDSlam(LPVOID lpParam)
{
int nRtn=0;
CSLAM *pCSLAM=(CSLAM *)lpParam;
CPoint_Cloud  *pPointCould=(CPoint_Cloud *)pCSLAM->m_stClassPtrs.pPointCloud;
COpenNI *pOpenNI=(COpenNI *)pCSLAM->m_stClassPtrs.pOpenNi;
//int nRunTimeIdx=0;
DWORD *pulRunTimeId=NULL;
DWORD ulRunTime[6];
bool bProcessOneFrame;


//pulRunTimeId=m_ulRunTimeBuff;
while(pOpenNI->GetOneFrame()!=0)
{
Sleep(10);
}

while (1)
{
if (m_bFrameReady)
{
printf("get time log Start!!!!   \n");
if (pCSLAM->m_nFirstFrame==1)
{
g_ulRunTimeBuff[0][0]=0;
pCSLAM->m_nFirstFrame=0;
}
else
{
if (bProcessOneFrame)
{
if(m_nCount==0)
{
g_ulRunTimeBuff[0][0]=g_ulRunTimeBuff[LOG_ARRAY_LEN-1][6];
}
else g_ulRunTimeBuff[m_nCount][0]=g_ulRunTimeBuff[m_nCount-1][6];
}
else
{
g_ulRunTimeBuff[m_nCount][0]=g_ulRunTimeBuff[m_nCount][6];
}
}

printf("get time log PASS!!!!   \n");
bProcessOneFrame=0;
g_ulRunTimeBuff[m_nCount][1]=::GetTickCount();
m_bFinishFrame=1;
m_bFrameReady=0;

printf("SLAM DLL Start!!!!   \n");
boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> point_cloud(new pcl::PointCloud<pcl::PointXYZRGB > ());
Eigen::Matrix4f final_transformation;
nRtn=pCSLAM->m_cRGBDSlam.IoTRobot_RGBDSLAM_RunOneFrame(m_pucRGBBuff,m_pusDepth,point_cloud,final_transformation,&g_ulRunTimeBuff[m_nCount][2]);

printf("SLAM DLL PASS!!!!   \n");
g_ulRunTimeBuff[m_nCount][3]=::GetTickCount();


printf("SLAM222222222222 DLL Start!!!!   \n");
m_cCallBackState((char) nRtn);
if (nRtn==0)
{
while (pPointCould->m_bNewFrame)
{
Sleep(1);
}
g_ulRunTimeBuff[m_nCount][4]=::GetTickCount();
pPointCould->NewFrameCloudIn(point_cloud,&final_transformation);

g_ulRunTimeBuff[m_nCount][7]=m_uiFrameNum;
m_nCount++;
m_uiFrameNum++;


if (m_nCount==LOG_ARRAY_LEN)
{
m_nCount=0;
}
bProcessOneFrame=true;
}
g_ulRunTimeBuff[m_nCount][6]=::GetTickCount();

printf("SLAM222222222222 DLL end!!!!   \n");
pOpenNI->GetOneFrame();
}
Sleep(10);
}

return 0;
}

*/
void CSLAM::SLAMRun()
{
	ID_RGBDSlam=0;
	ID_RGBDDynamicManage=0;
	ID_RGBDDynamicReceive=0;

	CreateThread(NULL,0,(LPTHREAD_START_ROUTINE)ThreadRGBDSlam,this,0,ID_RGBDSlam);
	CreateThread(NULL,0,(LPTHREAD_START_ROUTINE)ThreadRGBDDynamicManage,this,0,ID_RGBDDynamicManage);
	CreateThread(NULL,0,(LPTHREAD_START_ROUTINE)ThreadRGBDDynamicReceive,this,0,ID_RGBDDynamicReceive);
}