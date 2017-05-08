#include "CommandAndControl.h"
#include "SLAM.h"
#include "CPose3D.h"

DWORD g_ulRunTimeBuff[LOG_ARRAY_LEN][8];
DWORD g_ulFrameCount;

float *g_pfPC=new float[640*480*3];
void* g_pPCCallback;;

IoTRobot_MSGArray CCommandAndControl::m_stMSGArray;

CCommandAndControl *g_pCC=NULL;

// get last pose from CSLAM
// m_sfLastPose[0-5] x,y,z,roll,pitch,yaw
//



bool ConsoleHandler(unsigned long CEvent)
{
	char mesg[128];
	switch(CEvent)
	{
	case CTRL_C_EVENT:
		printf("1");
		break;
	case CTRL_BREAK_EVENT:
		printf("2");
		break;
	case CTRL_CLOSE_EVENT:
		g_pCC->CCUninit();
		g_pCC=NULL;
		break;
	case CTRL_LOGOFF_EVENT:
		printf("4");
		break;
	case CTRL_SHUTDOWN_EVENT:
		printf("5");
		break;
	}
	return true;
}




CCommandAndControl::CCommandAndControl(void)
{
	m_stClassPtrs.pSLAM=(void *)&m_CSLAM;
	m_stClassPtrs.p3Dmap=(void *)&m_c3DMap;
	m_stClassPtrs.pMapBuilder=(void *)&m_CMapBuilder;
	m_stClassPtrs.pPointCloud=(void *)&m_cPointCloud;
	m_stClassPtrs.pHostPotal=(void *)&m_CHostPotal;

	m_CSLAM.m_stClassPtrs=m_stClassPtrs;
	m_c3DMap.m_stClassPtrs=m_stClassPtrs;
	m_CMapBuilder.m_stClassPtrs=m_stClassPtrs;
	m_cPointCloud.m_stClassPtrs=m_stClassPtrs;
	g_pCC=this;
}

CCommandAndControl::~CCommandAndControl(void)
{
	int iii=100;
	m_CHostPotal.HostPotalUninit();
	m_CSLAM.SLAMUninit();
//	m_cPointCloud.PointCloudUninit();
//	m_c3DMap.C3DMapUninit();
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
		//SendCmd2SLAM((char *)pSettings);
		break;
	default:
		break;
	}
}


void CCommandAndControl::SendCmd2SLAM(const IoTRobot_Message MSG)
{
	m_CSLAM.NewCommand(MSG);	
}

void CCommandAndControl::SendCmd2MB(const IoTRobot_Message MSG)
{
	m_CMapBuilder.NewCommand(MSG);
}

void CCommandAndControl::SendCmd23Map(const IoTRobot_Message MSG)
{
	m_c3DMap.NewCommand(MSG);
}

void CCommandAndControl::SendCmd2HostPotal(const IoTRobot_Message MSG)
{
	m_CHostPotal.NewCommand(MSG);
}

void CCommandAndControl::SendCmd2IDrive(const IoTRobot_Message MSG)
{
	m_CIDrive.NewCommand(MSG);
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
	void *pData2OpenGL;
	void *pPlaneCallBack;
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



int g_test(void *pMessage)
{
	return 0;
}


int CCommandAndControl::CCInit()
{

	m_bSystemQuit=false;

	void *pTmpCB=NULL;
	pTmpCB=m_cGUI.IoTRobot_GUI_Init(CCommandAndControl::AddMSG);
	IoTRobot_GUIParam *pstGUIInit=(IoTRobot_GUIParam *)pTmpCB;

	
	m_stCallBackSet.cbSLAM=m_CSLAM.Callback_SLAM;
	m_stCallBackSet.pSLAMContext=&m_CSLAM;
	m_stCallBackSet.cdData2OpenGl=(NetServerCallBack_Data2OpenGL)pstGUIInit->pData2OpenGL;
	m_stCallBackSet.pData2OpenGlContext=NULL;

	g_pPCCallback=pstGUIInit->pPCCallBack;

	m_c3DMap.m_cbSLAMPC=(CallBack_SLAM_PC)pstGUIInit->pSLAMPCCallBack;
	m_c3DMap.m_cbPath=(CallBack_Path)pstGUIInit->pPathCallBack;
	m_CSLAM.m_cbState=(CallBack_RunState)pstGUIInit->pStateCallBack;
	m_CIDrive.m_cbPIPQVGA=(CallBack_PIPQVGA)pstGUIInit->pPIPCallBack;
	m_c3DMap.m_cbPlane=(CallBack_Session_Planes)pstGUIInit->pPlaneCallBack;

	m_stCallBackSet.cbGetDataFromNet=CallBack_GetDataFromNet;

	//printf("haha2 \n");
	m_CSLAM.SLAMInit(&m_stCallBackSet);
	//printf("haha3 \n");
	m_CHostPotal.HostPotalInit(m_stCallBackSet);
	m_c3DMap.C3DMapInit();
	m_CIDrive.IDriveInit();
	InitMSGArray();

	return 1;
}

int CCommandAndControl::CCRun()
{
	LPMSG msg=NULL;
	m_cPointCloud.PointCloudRun();
	m_c3DMap.C3DMapRun();
	m_CSLAM.SLAMRun();
	m_CHostPotal.HostPotalRun();
	LPDWORD ID=0;
	CreateThread(NULL,0,(LPTHREAD_START_ROUTINE)ThreadListenCmdFromGUI,this,0,ID);
	LPDWORD ID1=0;
	CreateThread(NULL,0,(LPTHREAD_START_ROUTINE)ThreadSendDeviceState2GUI,this,0,ID1);
	m_cGUI.IoTRobot_GUI_Run();

	return 1;
}



void CCommandAndControl::CallBack_GetDataFromNet(unsigned char*pucData,unsigned int uiDataLen,void *pContext)
{
	IoTRobot_MobileState stMobileState;

	printf("hhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhh\n");

	if(memcmp(pucData,"MobileMasterRobot!",18)==0)
	{
		stMobileState.cType=1;
		stMobileState.stDeviceState.cIMU=0;
		g_pCC->m_cGUI.IoTRobot_GUI_UpdateMobileState((void*)&stMobileState);
	}
	else  if (memcmp(pucData,"MobileAbandonRobot",18)==0)
	{
		stMobileState.cType=1;
		stMobileState.stDeviceState.cIMU=1;
		g_pCC->m_cGUI.IoTRobot_GUI_UpdateMobileState((void*)&stMobileState);
	}
	
}

int CCommandAndControl::CCUninit()
{
	m_CHostPotal.HostPotalUninit();
	m_CSLAM.SLAMUninit();
	return 1;
}


int main()
{

	if (SetConsoleCtrlHandler((PHANDLER_ROUTINE)ConsoleHandler,true)==false)
	{
		// unable to install handler... 
		// display message to the user
		printf("Unable to install handler!\n");
		return -1;
	} 
	//printf("Init sucessfully!!!\n");
	CCommandAndControl test;
	g_pCC=&test;
	test.CCInit();
	//printf("Init sucessfully!!!\n");
	test.CCRun();
}


int CCommandAndControl::InitMSGArray()
{
	int i;
	memset(&m_stMSGArray,0,sizeof(IoTRobot_MSGArray));

	for (i=0;i<KERNEL_MSG_LEN-1;i++)
	{
		m_stMSGArray.stMsgArray[i].pstNext=&m_stMSGArray.stMsgArray[i+1];
	}
	m_stMSGArray.stMsgArray[KERNEL_MSG_LEN-1].pstNext=&m_stMSGArray.stMsgArray[0];

	m_stMSGArray.pstWritePos=&m_stMSGArray.stMsgArray[0];
	m_stMSGArray.pstReadPos=&m_stMSGArray.stMsgArray[0];

	return 0;
}


int CCommandAndControl::AddMSG(const IoTRobot_Message *pstMessage)
{
	if(m_stMSGArray.pstWritePos->cStateFlag==0)
	{
		memcpy(&m_stMSGArray.pstWritePos->stContent,pstMessage,MSG_BUFF_SIZE);
		m_stMSGArray.pstWritePos->cStateFlag=1;
		m_stMSGArray.pstWritePos=m_stMSGArray.pstWritePos->pstNext;

		return 1;
	}
	else return 0;
}

int CCommandAndControl::PumpMSG(IoTRobot_Message *pstMessage)
{
	if (m_stMSGArray.pstReadPos->cStateFlag==1)
	{
		memcpy(pstMessage,&m_stMSGArray.pstReadPos->stContent,MSG_BUFF_SIZE);
		m_stMSGArray.pstReadPos->cStateFlag=0;
		m_stMSGArray.pstReadPos=m_stMSGArray.pstReadPos->pstNext;
		return 0;
	}
	else return -1;
}


UINT CCommandAndControl::ThreadListenCmdFromGUI(LPVOID lpParam)
{
	CCommandAndControl *pCC=(CCommandAndControl *)lpParam;
	IoTRobot_Message stMessage;
	while (!pCC->m_bSystemQuit)
	{
		while (pCC->PumpMSG(&stMessage)==0)
		{
		//	printf("Get Message   \nFrom module:  %d \nTo module:  %d \n Command: %d \n Priority : %d \n Param1 : %d  \n  Param2  %d  \n",
		//		stMessage.cFromModule,stMessage.cDestModule,stMessage.cCommand,stMessage.cPriority,stMessage.nParam1,stMessage.nParam2);
			switch (stMessage.cDestModule)
			{
			case SLAM_MSG:   //SLAM Module
				pCC->SendCmd2SLAM(stMessage);
				break;
			case MAP_BUILDER_MSG:   //Map Builder Module
				pCC->SendCmd2MB(stMessage);
				break;
			case DMAP_MSG:
				pCC->SendCmd23Map(stMessage);
				break;
			case HOSTPOTAL_MSG:
				pCC->SendCmd2HostPotal(stMessage);
				break;
			case IDRIVE_MSG:
				pCC->SendCmd2IDrive(stMessage);
				break;
			case CMDCTRL_MSG:
				pCC->HandleCmdFromOtherModule(stMessage);
				break;
			default:
				break;
			}
		}
		Sleep(5);
	}
	return 0;
}

UINT CCommandAndControl::ThreadSendDeviceState2GUI(LPVOID lpParam)
{
	int nState;
	IoTRobot_MobileState stMobileState;
	CCommandAndControl *pCC=(CCommandAndControl *)lpParam;


		while (pCC->m_CHostPotal.GetDeviceState(&nState))
		{
			Sleep(10);
		}
		stMobileState.cType=0;
		stMobileState.stDeviceState.cIMU=nState;
		pCC->m_cGUI.IoTRobot_GUI_UpdateMobileState((void*)&stMobileState);
	
	
	return 0;
}




void CCommandAndControl::HandleCmdFromOtherModule(const IoTRobot_Message MSG)
{
	switch (MSG.cCommand)
	{
		case CMDCTRL_MSG_GETLASTSLAMPOS:
			GetLastPoseFromSlam();

			break;
		case CMDCTRL_MSG_RESETSESSION:
			ResetSession();
			break;
		default:
			break;
	}
}

void CCommandAndControl::GetLastPoseFromSlam()
{
	//std::map<int, CPose3D>::iterator it_cell_pose = Id_Cells_Pose.find(*it_id_node);
	CPose3D LastPose=m_CMapBuilder.Id_Cells_Pose.rbegin()->second;
	double xyz[3];
	double rpy[3];
	LastPose.getXYZ(xyz);
	LastPose.getrpy(rpy);

	CCommandAndControl::m_sfLastPose[0]=xyz[0];
	CCommandAndControl::m_sfLastPose[1]=xyz[1];
	CCommandAndControl::m_sfLastPose[2]=xyz[2];

	CCommandAndControl::m_sfLastPose[3]=rpy[0];
	CCommandAndControl::m_sfLastPose[4]=rpy[1];
	CCommandAndControl::m_sfLastPose[5]=rpy[2];
}

void CCommandAndControl::ResetSession()
{
	GetLastPoseFromSlam();

	IoTRobot_Message MSG;
	MSG.cCommand=HOSTPORTAL_MSG_RESETSESSION;
	MSG.cDestModule=HOSTPORTAL_MODULE;
	MSG.cFromModule=CMDCTRL_MODULE;
	MSG.pcParam3=(char *)m_sfLastPose;
	MSG.nParam1=0;
	MSG.nParam2=0;
	AddMSG(&MSG);
}