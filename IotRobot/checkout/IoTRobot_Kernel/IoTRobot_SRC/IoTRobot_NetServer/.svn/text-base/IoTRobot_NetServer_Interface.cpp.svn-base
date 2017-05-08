#include "IoTRobot_NetServer_Interface.h"
#include "IoTRobot_NetServer.h"


IoTRobot_NetServer g_CNetServer;
int g_nCnCNum;
IoTRobot_NetServer_Interface::IoTRobot_NetServer_Interface(void)
{
}

IoTRobot_NetServer_Interface::~IoTRobot_NetServer_Interface(void)
{
}

int IoTRobot_NetServer_Interface::NetServer_Init(void * pCallBackSet,void *pSendCmdFuncSet)
{
	IoT_NetCallBackSet *pstCallBackSet=(IoT_NetCallBackSet *)pCallBackSet;
	IoT_NetSendCMDFuncSet *pstSendFuncSet=(IoT_NetSendCMDFuncSet *)pSendCmdFuncSet;
	g_nCnCNum=0;
	return g_CNetServer.NetServer_Init(*pstCallBackSet,pstSendFuncSet);
}


int IoTRobot_NetServer_Interface::NetServer_Run()
{
	return g_CNetServer.NetServer_Run();
}

int IoTRobot_NetServer_Interface::NetServer_Stop()
{
	return g_CNetServer.NetServer_Stop();
}


int IoTRobot_NetServer_Interface::NetServer_Uninit()
{
	g_CNetServer.NetServer_Uninit();
	return 0;
}


int IoTRobot_NetServer_Interface::NetServer_GetDeviceState(int *pnState)
{
	return g_CNetServer.NetServer_GetDeviceState(pnState);
}


int IoTRobot_NetServer_Interface::NetServer_GetNSState(char *pData)
{
	if (g_nCnCNum!=g_CNetServer.m_nCnCNum)
	{
		g_nCnCNum=g_CNetServer.m_nCnCNum;
		memcpy(pData,g_CNetServer.m_stTCPIPParamSet.stTCPIPParam[g_nCnCNum-1].ucClientIP,4);
		memcpy(pData+4,&g_CNetServer.m_stTCPIPParamSet.stTCPIPParam[g_nCnCNum-1].nServerRTPVideoPort,4);
		memcpy(pData+8,&g_CNetServer.m_stTCPIPParamSet.stTCPIPParam[g_nCnCNum-1].nServerRTPAudioPort,4);
		memcpy(pData+12,&g_CNetServer.m_stTCPIPParamSet.stTCPIPParam[g_nCnCNum-1].nClientRTPVideoPort,4);
		memcpy(pData+16,&g_CNetServer.m_stTCPIPParamSet.stTCPIPParam[g_nCnCNum-1].nClientRTPAudioPort,4);
		
		return 0;
	}
	return -1;
}