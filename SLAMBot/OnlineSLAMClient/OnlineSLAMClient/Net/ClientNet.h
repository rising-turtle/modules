#pragma once


#include <Winsock2.h>
#include <string.h>
#include <stdio.h>
#include <mstcpip.h>
#define TRY_TIME 10
#pragma comment (lib,"wsock32.lib")
#pragma comment (lib,"ws2_32.lib")

typedef int (*CallBack_RecvData)(char *pcData,int nDataLen);

typedef struct RegisterInfo
{
	int nID;
	float fOffsetPamras[4];
}RegisterInfo;


class ClientNet
{
public:
	ClientNet(void);
	~ClientNet(void);

	virtual int ComInit(void *pcParams);
	virtual int ComRun(void *pcParams);
	virtual int ComStop(void *pcParams);
	virtual int ComUninit(void *pcParams);

	int ClientNetInit(char *pucConfig);
	int ClientNetRun();
	//int Listen2Host();
	bool m_bStopListen;
	int SendData(char *pcData,int nDataLen);
	int GetNetStatus();

	CallBack_RecvData m_cbRecvData;
	RegisterInfo m_stRegisterInfo;
private:
	int SetAddr(int nPort,unsigned char *pucIP,sockaddr_in &addr);
	int SetSocket(int nPort,unsigned char *pucIP,sockaddr_in &addr);
	int Try2CncHost();
	int RcvStream(int nGoalSize,char *pcTmpBuff,int sockfd);
	int SendStream(int nGoalSize,char *pcTmpBuff,int sockfd);
	int ParseIP(char *pcData,char *pcIP);
	int m_nNetStatus;

	struct sockaddr_in m_HostAddr;
	struct sockaddr_in m_ClientAddr;
	
	int m_nClientSock;


	int m_nClientPort;
	char m_cClientIP[4];



	char m_cRegisterBuff[100];
};
