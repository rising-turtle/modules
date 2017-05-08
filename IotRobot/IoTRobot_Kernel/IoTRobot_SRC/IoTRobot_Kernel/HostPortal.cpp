
#pragma once
#include "CommandAndControl.h"
#include "HostPotal.h"
CHostPotal::CHostPotal()
{

}

CHostPotal::~CHostPotal()
{

}


int CHostPotal::HostPotalInit(IoT_CallBackFuncSet stCallBackSet)
{
	IoT_NetCallBackSet stNetServerCallBack;
	//stNetServerCallBack.
	//stNetServerCallBack.cbDisplay=stCallBackSet.callBack_RGB24;
	//stNetServerCallBack.cbSlam=stCallBackSet.callBack_RGB24_Depth;
	//stNetServerCallBack.cbSlam_IMU=stCallBackSet.cbSlam_IMU;
	stNetServerCallBack.cbSlam=stCallBackSet.cbSLAM;
	stNetServerCallBack.pSlamContext=stCallBackSet.pSLAMContext;
	

	stNetServerCallBack.cdData2OpenGl=stCallBackSet.cdData2OpenGl;
	stNetServerCallBack.pData2OpenGlContext=stCallBackSet.pData2OpenGlContext;

	stNetServerCallBack.cbData2Kernel=stCallBackSet.cbGetDataFromNet;
	stNetServerCallBack.pData2Kernel=NULL;
	m_CNetServer.NetServer_Init((void *)&stNetServerCallBack,(void *)&m_stSendCMDFuncSet);
	return 0;
}

int CHostPotal::HostPotalRun()
{
	LPDWORD ID=0;
	m_nStopMonitorNS=0;
	CreateThread(NULL,0,(LPTHREAD_START_ROUTINE)ThreadNetServer,this,0,ID);
	CreateThread(NULL,0,(LPTHREAD_START_ROUTINE)ThreadMonitorNetServerStatus,this,0,ID);

	return 0;
}

int CHostPotal::HostPotalUninit()
{
	m_CNetServer.NetServer_Stop();
	m_CNetServer.NetServer_Uninit();
	return 0;
}

int CHostPotal::HostPotalStop()
{
	m_nStopMonitorNS=true;
	return 0;
}

int CHostPotal::GetDeviceState(int *pnState)
{
	return  m_CNetServer.NetServer_GetDeviceState(pnState);
}

int CHostPotal::SendSysCmd(char *pContent,int nLen)
{
	m_stSendCMDFuncSet.scSendSysCMD(pContent,nLen,0,m_stSendCMDFuncSet.pSendSysCMDContext);
	
	return 0;
}

int CHostPotal::SendCtrlCmd(char *pContent,int nLen)
{
	m_stSendCMDFuncSet.scSendCtlCMD(pContent,nLen,0,m_stSendCMDFuncSet.pSendCtlCMDContext);
	return 0;
}


void CHostPotal::NewCommand(const IoTRobot_Message MSG)
{
	switch (MSG.cCommand)
	{
	case HOSTPORTAL_MSG_IDRIVE:
		SendSysCmd("IDrive",6);
//		m_CNetServer.NetServer_ChangeRunMode(0,1);
		break;
	case HOSTPORTAL_MSG_AUTODRIVE:
		SendSysCmd("AutoDr",6);
	//	m_CNetServer.NetServer_ChangeRunMode(0,0);
		break;

	case HOSTPORTAL_MSG_SPECIFIEDPAH:
		char cLen[4];
		char *pcAddr;
		SendSysCmd("SpPath",6);
		memcpy(cLen,&MSG.nParam2,4);
		SendSysCmd(cLen,4);
		SendSysCmd(MSG.pcParam3,MSG.nParam2*3*4);
		break;
	case HOSTPORTAL_MSG_CTRL_CMD:
		SendCtrlCmd(MSG.pcParam3,16);

		int nAngle,nGear,nRot,nType;
		memcpy(&nAngle,MSG.pcParam3,4);
		memcpy(&nGear,MSG.pcParam3+4,4);
		memcpy(&nRot,MSG.pcParam3+8,4);
		memcpy(&nType,MSG.pcParam3+12,4);
		printf("HOSTPORTAL  angle :%d   gear :%d  nRot :%d  nType :%d \n",nAngle,nGear,nRot,nType);
		break;

	case HOSTPORTAL_MSG_EMERGENCYBREAK:
		SendSysCmd("STOP!!",6);
		break;

	case HOSTPORTAL_MSG_RESETSESSION:
		SendSysCmd("RESSES",6);
		SendSysCmd(MSG.pcParam3,48);
		break;
	}
}

UINT CHostPotal::ThreadNetServer(LPVOID lpParam)
{
	CHostPotal *pHostPotal=(CHostPotal *)lpParam;
	pHostPotal->m_CNetServer.NetServer_Run();
	return 0;
}


UINT CHostPotal::ThreadMonitorNetServerStatus(LPVOID lpParam)
{
	CHostPotal *pHostPotal=(CHostPotal *)lpParam;
	char cBuff[20];
	IoTRobot_Message MSG;
	while (!pHostPotal->m_nStopMonitorNS)
	{
		if (pHostPotal->m_CNetServer.NetServer_GetNSState(cBuff)==0)
		{
			MSG.cFromModule=HOSTPOTAL_MSG;
			MSG.cDestModule=IDRIVE_MSG;
			MSG.cCommand=IDRIVE_IP_INFO;
			MSG.pcParam3=cBuff;
			CCommandAndControl::AddMSG(&MSG);
		}
		Sleep(1000);
	}
	return 0;
}