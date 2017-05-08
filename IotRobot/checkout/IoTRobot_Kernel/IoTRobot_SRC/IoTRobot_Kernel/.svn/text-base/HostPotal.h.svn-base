#ifndef IOT_HOSTPOTAL_H
#define IOT_HOSTPOTAL_H

#pragma once
#include "InternalDefine.h"
//#include "IoTRobot_NetServer.h"
#include "IoTRobot_NetServer_Interface.h"
#include "IoTRobot_NetServer_Define.h"

//#ifdef _DEBUG
//#pragma comment(lib,"jrtplib_d.lib")
//#pragma comment(lib,"jthread_d.lib")
//#pragma comment(lib,"RTP/jrtplib.lib")
//#pragma comment(lib,"RTP/jthread.lib")
//#else
//#pragma comment(lib,"jrtplib.lib")
//#pragma comment(lib,"jthread.lib")
//#endif


class CHostPotal
{
public:
	CHostPotal();
	~CHostPotal();


	int HostPotalInit(IoT_CallBackFuncSet stCallBackSet);
	int HostPotalRun();
	int HostPotalStop();
	int HostPotalUninit();
	int GetDeviceState(int *pnState);
	int SendSysCmd(char *pContent,int nLen);
	int SendCtrlCmd(char *pContent,int nLen);

	void NewCommand(const IoTRobot_Message MSG);

	IoTRobot_NetServer_Interface m_CNetServer;
	IoT_NetSendCMDFuncSet m_stSendCMDFuncSet;

	static UINT ThreadNetServer(LPVOID lpParam);


	int m_nStopMonitorNS;
	static UINT ThreadMonitorNetServerStatus(LPVOID lpParam);

protected:


private:
};

#endif