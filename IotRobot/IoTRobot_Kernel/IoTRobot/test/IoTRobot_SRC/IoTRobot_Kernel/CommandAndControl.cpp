#include "CommandAndControl.h"
#include "SLAM.h"


DWORD g_ulRunTimeBuff[LOG_ARRAY_LEN][8];
DWORD g_ulFrameCount;

float *g_pfPC=new float[640*480*3];
void* g_pPCCallback;;
CCommandAndControl::CCommandAndControl(void)
{
	m_stClassPtrs.pSLAM=(void *)&m_CSLAM;
	m_stClassPtrs.p3Dmap=(void *)&m_c3DMap;
	m_stClassPtrs.pMapBuilder=(void *)&m_CMapBuilder;
	m_stClassPtrs.pPointCloud=(void *)&m_cPointCloud;
	m_stClassPtrs.pOpenNi=(void *)&m_cOpenNI;

	m_CSLAM.m_stClassPtrs=m_stClassPtrs;
	m_c3DMap.m_stClassPtrs=m_stClassPtrs;
	m_CMapBuilder.m_stClassPtrs=m_stClassPtrs;
	m_cPointCloud.m_stClassPtrs=m_stClassPtrs;
}

CCommandAndControl::~CCommandAndControl(void)
{
}


void CCommandAndControl::GetSettings(int nPos,void *pSettings)
{
	m_CSettings.GetData(nPos,(char *)pSettings);

}


void CCommandAndControl::StoreSettings(int nPos,void *pSettings)
{
	m_CSettings.SetData(nPos,(char *)pSettings);
	switch (nPos)
	{
	case 0:
		SendCmd2SLAM((char *)pSettings);
		break;
	default:
		break;
	}
}


void CCommandAndControl::SendCmd2SLAM(char *pCmd)
{
	m_CSLAM.NewCommand(pCmd);	
}


void CCommandAndControl::PushCtrlCmd(void *pMove)
{
	m_CCmdCtrl.AddMsg((char *)pMove);
}


UINT CCommandAndControl::ThreadSLAM(LPVOID lpParam)
{
	CCommandAndControl::SLAMPARAM *pstSlam=(CCommandAndControl::SLAMPARAM*)lpParam;
//	pstSlam->m_CSLAM.SLAMInit();
	return 0;
}
void CCommandAndControl::StartSLAM()
{
	LPDWORD ID=0;
	m_stSlamParam.m_CSLAM=m_CSLAM;
	CreateThread(NULL,0,(LPTHREAD_START_ROUTINE)ThreadSLAM,&m_stSlamParam,0,ID);
}

void CCommandAndControl::CmdCtrlRun()
{
	StartSLAM();
}
typedef struct IoTRobot_GUIParam
{
	void *pPIPCallBack;
	void *pPCCallBack;
	void *pSLAMPCCallBack;
	void *pStateCallBack;
	void *pPathCallBack;
}IoTRobot_GUIParam;


UINT CCommandAndControl::ThreadLog(LPVOID lpParam)
{
	CCommandAndControl *pCC=(CCommandAndControl *)lpParam;
	int i=0;
	while (1)
	{
		if (g_ulRunTimeBuff[i][5]!=0&&g_ulRunTimeBuff[i][4]!=0)
		{
			pCC->m_cLog.writeintolog(g_ulRunTimeBuff[i],true);
			i++;
			if (i==LOG_ARRAY_LEN)
			{
				i=0;
			}
			g_ulRunTimeBuff[i][5]=0;
			g_ulRunTimeBuff[i][4]=0;

		}
		Sleep(1);
	}
}
void CallBack_RGB24_Depth(unsigned char *pucRGB24,unsigned short *pusDepth,void *pContext)
{
	int i,j,m=0,k=0;
	for (i=-240;i<240;i++)
	{
		for (j=-320;j<320;j++)
		{
			if (pusDepth[m]!=0)
			{
				g_pfPC[k+2]=1.0*(float)pusDepth[m]/1000.0;
			}
			else
			{
				g_pfPC[k+2]=-1000000;
			}

			g_pfPC[k]=(float)j*0.001904*g_pfPC[k+2];
			g_pfPC[k+1]=(float)i*0.001904*g_pfPC[k+2];
			k+=3;
			m++;
		}
	}

	CallBack_PointCloud pTmp=(CallBack_PointCloud)g_pPCCallback;
	pTmp(pucRGB24,g_pfPC,NULL);
}
int CCommandAndControl::CCInit()
{
	//printf("haha1 \n");
	void *pTmpCB=NULL;
	pTmpCB=m_cGUI.IoTRobot_GUI_Init();
	IoTRobot_GUIParam *pstGUIInit=(IoTRobot_GUIParam *)pTmpCB;

	m_stCallBackSet.callBack_RGB24=(OpenNICallBack_RGB24)pstGUIInit->pPIPCallBack;
	m_stCallBackSet.callBack_XYZRGB=NULL;
	m_stCallBackSet.callBack_RGB=NULL;
	//m_stCallBackSet.callBack_RGB24_Depth=CallBack_RGB24_Depth;
	m_stCallBackSet.callBack_RGB24_Depth=m_CSLAM.CallBack_RGB24_Depth;
	m_stCallBackSet.cbSlam_IMU=m_CSLAM.CallBack_SLAM_IMU;


	g_pPCCallback=pstGUIInit->pPCCallBack;

	m_CSLAM.m_cbPointCould=(CallBack_PointCloud)pstGUIInit->pPCCallBack;
	//m_c3DMap.m_cbPointCould=(CallBack_PointCloud)pstGUIInit->pPCCallBack;
	m_c3DMap.m_cbSLAMPC=(CallBack_SLAM_PC)pstGUIInit->pSLAMPCCallBack;
	m_c3DMap.m_cbPath=(CallBack_Path)pstGUIInit->pPathCallBack;
	m_CSLAM.m_cCallBackState=(CallBack_RunState)pstGUIInit->pStateCallBack;

	//printf("haha2 \n");
	m_CSLAM.SLAMInit(&m_stCallBackSet);
	//printf("haha3 \n");
	m_cOpenNI.OpenNIInit(m_stCallBackSet);
	m_c3DMap.C3DMapInit();

	return 1;
}

int CCommandAndControl::CCRun()
{
	LPMSG msg=NULL;
	m_cPointCloud.PointCloudRun();
	m_c3DMap.C3DMapRun();
	m_CSLAM.SLAMRun();
	m_cOpenNI.OpenNIRun();
	LPDWORD ID=0;
	CreateThread(NULL,0,(LPTHREAD_START_ROUTINE)ThreadListenCmdFromGUI,this,0,ID);
	m_cGUI.IoTRobot_GUI_Run();

	

	while (1)
	{
		Sleep(30);
	}
	return 1;
}


int CCommandAndControl::CCUninit()
{
	return 1;
}


int main()
{
	//printf("Init sucessfully!!!\n");
	CCommandAndControl test;
	test.CCInit();
	//printf("Init sucessfully!!!\n");
	test.CCRun();
}




UINT CCommandAndControl::ThreadListenCmdFromGUI(LPVOID lpParam)
{
	CCommandAndControl *pCC=(CCommandAndControl *)lpParam;
	IoTRobot_GUICMD stGUICmd;
	memset(&stGUICmd,0,sizeof(IoTRobot_GUICMD));

	IoTRobot_GUICMD *pGUICmd=(IoTRobot_GUICMD *)&pCC->m_cGUI.m_stGUICMD;
	while (1)
	{
		if(memcmp(&stGUICmd,pGUICmd,sizeof(IoTRobot_GUICMD))!=0)
		{
			memcpy(&stGUICmd,pGUICmd,sizeof(IoTRobot_GUICMD));
			if(pCC->m_CSLAM.m_nApplyIMU2!=0)
			{
				pCC->m_CSLAM.m_nApplyIMU=stGUICmd.nSLAMCMD;
			}
		}
		Sleep(25);
	}
}







