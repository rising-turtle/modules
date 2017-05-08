#include "IoTRobot_NetServer.h"
#include "IoTRobot_NetProtocol.h"

#pragma once
char *g_pcAnswerModel="OK. CmdPort:xxxx, KinectPort:xxxx, IDrivePort:xxxx, RTPVideoPort:xxxx, RTPAudioPort:xxxx";


typedef struct ThreadParam
{
	IoTRobot_NetServer *pCNetServer;
	IoTRobot_TCPIPParam *pstTCPIPParam;
	int nNewFd;
}ThreadParam;
ThreadParam g_stThreadParam;

void *IoTRobot_NetServer::m_pNetServer=NULL;

int IoTRobot_NetServer::RcvStream(int nGoalSize,char *pcTmpBuff,int sockfd)
{
	int nRecvCount,nLeftLen,nNumBytes;

	nLeftLen=nGoalSize;
	nRecvCount=0;
	while (nRecvCount<nGoalSize)
	{
		if ((nNumBytes=recv(sockfd, pcTmpBuff, nLeftLen, 0)) == -1) 
		{
			return -1;
		}
		nRecvCount+=nNumBytes;
		pcTmpBuff+=nNumBytes;
		nLeftLen=nGoalSize-nRecvCount;
	}
	return 1;
}



int IoTRobot_NetServer::SendStream(int nGoalSize,char *pcTmpBuff,int sockfd)
{
	int nRecvCount,nLeftLen,nNumBytes;

	nLeftLen=nGoalSize;
	nRecvCount=0;
	while (nRecvCount<nGoalSize)
	{
		if ((nNumBytes=send(sockfd, pcTmpBuff, nLeftLen, 0)) == -1) 
		{
			return -1;
		}
		nRecvCount+=nNumBytes;
		pcTmpBuff+=nNumBytes;
		nLeftLen=nGoalSize-nRecvCount;
	}
	return 1;
}




IoTRobot_NetServer::IoTRobot_NetServer(void)
{
	m_pcMemoryPool=NULL;
//	m_bNewKinectConf=false;
	//memset(&m_stNetCleintInfoSet,0,sizeof(IoT_NetClientInfoSet));
}

IoTRobot_NetServer::~IoTRobot_NetServer(void)
{
}

int IoTRobot_NetServer::GetHostIP(BYTE *ucMyIP)
{
	WSADATA wsadata; 
	WORD dwVersionRequested; 

	memset(ucMyIP,0,4);
	dwVersionRequested  =  MAKEWORD( 1, 1 );//版本号1.1


	int err; 
	err=WSAStartup(dwVersionRequested,&wsadata); 


	char hostname[128]; 
	if(gethostname(hostname,128)==0) 
	{ 
	}
	struct hostent *pHost = gethostbyname(hostname);
	WSACleanup(); 
	if (pHost!=NULL)
	{
		struct in_addr *pAddr=(struct in_addr *)pHost->h_addr_list[0];

		int i=100;
		ucMyIP[0]=pAddr->S_un.S_un_b.s_b1;
		ucMyIP[1]=pAddr->S_un.S_un_b.s_b2;
		ucMyIP[2]=pAddr->S_un.S_un_b.s_b3;
		ucMyIP[3]=pAddr->S_un.S_un_b.s_b4;

		return 0;
	}
	return -1;
}


int IoTRobot_NetServer::NetServer_Init(IoT_NetCallBackSet stCallBackSet, IoT_NetSendCMDFuncSet *pstSendCMDFuncSet)
{
	int i,nPort,nCount=0,nRslt;
	char *pcTmp1=NULL;
	WORD wVersionRequested;
	WSADATA wsaData;


	if(GetHostIP(m_ucHostIP)==0)
	{
		wVersionRequested = MAKEWORD( 1, 1 );
		WSAStartup( wVersionRequested, &wsaData );
		memset(&m_stTCPIPParamSet,0,sizeof(IoTRobot_TCPIPParamSet));

		nPort=MY_PORT;
		for (i=0;i<MAX_CLIENT_NUM;i++)
		{
			m_stTCPIPParamSet.stTCPIPParam[i].nCmdPort=(++nPort);
			m_stTCPIPParamSet.stTCPIPParam[i].nSLAMPort=(++nPort);
			m_stTCPIPParamSet.stTCPIPParam[i].nIDrivePort=(++nPort);
			m_stTCPIPParamSet.stTCPIPParam[i].nServerRTPVideoPort=(++nPort);//RTP Port must be odd
			nPort++;
			m_stTCPIPParamSet.stTCPIPParam[i].nServerRTPAudioPort=(++nPort);//RTP Port must be odd

			m_stTCPIPParamSet.stTCPIPParam[i].CmdAddr.sin_family=AF_INET;
			m_stTCPIPParamSet.stTCPIPParam[i].CmdAddr.sin_addr.S_un.S_un_b.s_b1=m_ucHostIP[0];
			m_stTCPIPParamSet.stTCPIPParam[i].CmdAddr.sin_addr.S_un.S_un_b.s_b2=m_ucHostIP[1];
			m_stTCPIPParamSet.stTCPIPParam[i].CmdAddr.sin_addr.S_un.S_un_b.s_b3=m_ucHostIP[2];
			m_stTCPIPParamSet.stTCPIPParam[i].CmdAddr.sin_addr.S_un.S_un_b.s_b4=m_ucHostIP[3];


			m_stTCPIPParamSet.stTCPIPParam[i].CmdAddr.sin_port=htons(m_stTCPIPParamSet.stTCPIPParam[i].nCmdPort);


			if ((m_stTCPIPParamSet.stTCPIPParam[i].nSockCmd = socket(AF_INET, SOCK_STREAM, 0)) == -1) {
				printf("Get CMD Sock Failed!!! \n");
				return -1;
			}
			if (nRslt=bind(m_stTCPIPParamSet.stTCPIPParam[i].nSockCmd,
				(struct sockaddr *)&m_stTCPIPParamSet.stTCPIPParam[i].CmdAddr,
				sizeof(struct sockaddr))==-1)
			{
				printf("Bind CMD Sock Failed!!! \n");
				return -2;
			}



			m_stTCPIPParamSet.stTCPIPParam[i].SLAMAddr.sin_family=AF_INET;
			m_stTCPIPParamSet.stTCPIPParam[i].SLAMAddr.sin_addr.S_un.S_un_b.s_b1=m_ucHostIP[0];
			m_stTCPIPParamSet.stTCPIPParam[i].SLAMAddr.sin_addr.S_un.S_un_b.s_b2=m_ucHostIP[1];
			m_stTCPIPParamSet.stTCPIPParam[i].SLAMAddr.sin_addr.S_un.S_un_b.s_b3=m_ucHostIP[2];
			m_stTCPIPParamSet.stTCPIPParam[i].SLAMAddr.sin_addr.S_un.S_un_b.s_b4=m_ucHostIP[3];
			m_stTCPIPParamSet.stTCPIPParam[i].SLAMAddr.sin_port=htons(m_stTCPIPParamSet.stTCPIPParam[i].nSLAMPort);
			if ((m_stTCPIPParamSet.stTCPIPParam[i].nSockSLAM = socket(AF_INET, SOCK_STREAM, 0)) == -1) {
				return -3;
			}
			if (nRslt=bind(m_stTCPIPParamSet.stTCPIPParam[i].nSockSLAM,
				(struct sockaddr *)&m_stTCPIPParamSet.stTCPIPParam[i].SLAMAddr,
				sizeof(struct sockaddr))==-1)
			{
				return -4;
			}


			m_stTCPIPParamSet.stTCPIPParam[i].IDriveAddr.sin_family=AF_INET;
			m_stTCPIPParamSet.stTCPIPParam[i].IDriveAddr.sin_addr.S_un.S_un_b.s_b1=m_ucHostIP[0];
			m_stTCPIPParamSet.stTCPIPParam[i].IDriveAddr.sin_addr.S_un.S_un_b.s_b2=m_ucHostIP[1];
			m_stTCPIPParamSet.stTCPIPParam[i].IDriveAddr.sin_addr.S_un.S_un_b.s_b3=m_ucHostIP[2];
			m_stTCPIPParamSet.stTCPIPParam[i].IDriveAddr.sin_addr.S_un.S_un_b.s_b4=m_ucHostIP[3];
			m_stTCPIPParamSet.stTCPIPParam[i].IDriveAddr.sin_port=htons(m_stTCPIPParamSet.stTCPIPParam[i].nIDrivePort);
			if ((m_stTCPIPParamSet.stTCPIPParam[i].nSockIDrive = socket(AF_INET, SOCK_STREAM, 0)) == -1) {
				return -5;
			}
			if (nRslt=bind(m_stTCPIPParamSet.stTCPIPParam[i].nSockIDrive,
				(struct sockaddr *)&m_stTCPIPParamSet.stTCPIPParam[i].IDriveAddr,
				sizeof(struct sockaddr))==-1)
			{
				return -6;
			}
		}


		m_pcMemoryPool=new char[IOT_MEMORY_POOL_SIZE];
		memset(m_pcMemoryPool,0,IOT_MEMORY_POOL_SIZE);
		pcTmp1=m_pcMemoryPool;

		MEMORY_ALLOCATION(m_stDataPtrSet.pcTmpBuff,char*,pcTmp1,IOT_TMP_BUFF_SIZE,nCount);
		MEMORY_ALLOCATION(m_stDataPtrSet.pcTmpSLAMBuff,char*,pcTmp1,IOT_TMP_BUFF_SIZE,nCount);
		MEMORY_ALLOCATION(m_stDataPtrSet.pcTmpCmdBuff,char*,pcTmp1,IOT_TMP_BUFF_SIZE,nCount);
		MEMORY_ALLOCATION(m_stDataPtrSet.pcTmpIDriveBuff,char*,pcTmp1,IOT_TMP_BUFF_SIZE,nCount);

		m_stCallBackSet=stCallBackSet;


		pstSendCMDFuncSet->scSendCtlCMD=SendCtlCmd;
		pstSendCMDFuncSet->scSendSysCMD=SendSysCmd;
		pstSendCMDFuncSet->pSendCtlCMDContext=this;
		pstSendCMDFuncSet->pSendSysCMDContext=this;
		m_nIMUState=-1;

		m_bStopListen=true;
		m_bStopThreadSLAMSock=true;
		m_bStopThreadCmdSock=true;
		m_bStopThreadIDriveSock=true;

		m_pNetServer=this;
		return 0;
	}
	else
	{
		printf("Error,can not get Host IP!!! \n");
	}
	return -7;
}

typedef struct AnwserRequest
{
	int nNewFD;
	unsigned char ucClientIP[4];
}AnwserRequest;

int IoTRobot_NetServer::NetServer_Run()
{

	int nRslt,nNewFd;
	BYTE ucClientIP[4];

	AnwserRequest stAnwserRequest;
	struct sockaddr_in ClientAddr;
	int nAddrLen=sizeof(struct sockaddr_in);

	m_bStopListen=false;
	m_bStopThreadSLAMSock=false;
	m_bStopThreadCmdSock=false;
	m_bStopThreadIDriveSock=false;

	if ((m_nSockCmd = socket(AF_INET, SOCK_STREAM, 0)) == -1) {
		return -1;
	}

	m_MyAddr.sin_family=AF_INET;
	m_MyAddr.sin_port=htons(MY_PORT);
	m_MyAddr.sin_addr.S_un.S_un_b.s_b1=m_ucHostIP[0];
	m_MyAddr.sin_addr.S_un.S_un_b.s_b2=m_ucHostIP[1];
	m_MyAddr.sin_addr.S_un.S_un_b.s_b3=m_ucHostIP[2];
	m_MyAddr.sin_addr.S_un.S_un_b.s_b4=m_ucHostIP[3];
	memset(&m_MyAddr.sin_zero,0,8);


	if (nRslt=bind(m_nSockCmd,(struct sockaddr *)&m_MyAddr,sizeof(struct sockaddr))==-1)
	{
		printf("bind error!! %d\n" ,nRslt);
		return -2;
	}

	if(listen(m_nSockCmd,BACKLOG)==-1)
	{
		printf("listen error!! \n");
		return -3;
	}

	int nNumber=0;

	while (!m_bStopListen)
	{
		printf("I am waiting for connecting request!!!!! \n");
		if((nNewFd=accept(m_nSockCmd,
			(struct sockaddr *)&ClientAddr,&nAddrLen))==-1)
		{
			printf("accept error!!! \n");
			continue;
		}
		//printf("new_fd: %d \n",nNewFd);
		printf("server :got connection from %s\n",
			inet_ntoa(ClientAddr.sin_addr));

		ucClientIP[0]=ClientAddr.sin_addr.S_un.S_un_b.s_b1;
		ucClientIP[1]=ClientAddr.sin_addr.S_un.S_un_b.s_b2;
		ucClientIP[2]=ClientAddr.sin_addr.S_un.S_un_b.s_b3;
		ucClientIP[3]=ClientAddr.sin_addr.S_un.S_un_b.s_b4;

		stAnwserRequest.nNewFD=nNewFd;
		memcpy(stAnwserRequest.ucClientIP,ucClientIP,4);
		
		//AnwserRequests(nNewFd,ucClientIP);
		LPDWORD ID=0;
		CreateThread(NULL,0,(LPTHREAD_START_ROUTINE)ThreadAnwserRequests,&stAnwserRequest,0,ID);
		Sleep(20);
	}
	
	
	WSACleanup();
	return 1;
}


UINT IoTRobot_NetServer::ThreadAnwserRequests(LPVOID lpParam)
{
	IoTRobot_NetServer *pNetServer=(IoTRobot_NetServer *)m_pNetServer;
	AnwserRequest *pAnwserRequest=(AnwserRequest *)lpParam;

	Sleep(10);
	pNetServer->AnwserRequests(pAnwserRequest->nNewFD,pAnwserRequest->ucClientIP);
	return 0;
}

int IoTRobot_NetServer::AnwserRequests(int nSockFD,BYTE *pucClientIP)
{
	char cMobilePrefix[]= "Mobile";
	char cRobotPrefix[]="I am R";
	char cTmp[98];
	int nCount=0;
	while (nCount<10000)
	{
		if(RcvStream(97,m_stDataPtrSet.pcTmpBuff,nSockFD)==1)
		{
			m_stDataPtrSet.pcTmpBuff[98]='\0';
			memcpy(cTmp,cRobotPrefix,97);

			if (memcmp(m_stDataPtrSet.pcTmpBuff,cMobilePrefix,6)==0)
			{
				AnwserMobileRequests(nSockFD,pucClientIP);
			}
			else if (memcmp(m_stDataPtrSet.pcTmpBuff,cRobotPrefix,6)==0)
			{
				AnwserRobotRequests(nSockFD,pucClientIP);
			}
			break;
		}
		nCount++;
	}


	return 0;
}


int IoTRobot_NetServer::AnwserMobileRequests(int nSockFD,BYTE *pucClientIP)
{
	char cContent[64];
	int nPos=0;
	char cTmp;
	short sTmp;
	if (m_stTCPIPParamSet.nOccupyFlag[0]==1)
	{
		memcpy(cContent+nPos,"OK",2);
		nPos+=2;
		
		memcpy(cContent+nPos,&m_stTCPIPParamSet.stTCPIPParam[0].ucClientIP[0],1);
		memcpy(&cTmp,cContent+nPos,1);
		printf("IP1  :%d", cTmp);
		nPos+=1;


		memcpy(cContent+nPos,&m_stTCPIPParamSet.stTCPIPParam[0].ucClientIP[1],1);
		memcpy(&cTmp,cContent+nPos,1);
		printf("IP2  :%d", cTmp);
		nPos+=1;

		memcpy(cContent+nPos,&m_stTCPIPParamSet.stTCPIPParam[0].ucClientIP[2],1);
		memcpy(&cTmp,cContent+nPos,1);
		printf("IP3  :%d", cTmp);
		nPos+=1;

		memcpy(cContent+nPos,&m_stTCPIPParamSet.stTCPIPParam[0].ucClientIP[3],1);
		memcpy(&cTmp,cContent+nPos,1);
		printf("IP4  :%d", cTmp);
		nPos+=1;
	
		memcpy(cContent+nPos,&m_stTCPIPParamSet.stTCPIPParam[0].ClientAddr.sin_port,2);
		memcpy(&sTmp,cContent+nPos,2);
		printf("Port  :%d", sTmp);
		nPos+=2;

		//m_stCallBackSet.cbData2Kernel(NULL,NULL,NULL);
		unsigned char ucTmp[18];
		memcpy(ucTmp,"MobileMasterRobot!",18);
		m_stCallBackSet.cbData2Kernel(ucTmp,18,NULL);
	}
	else
	{
		memcpy(cContent+nPos,"NO",2);
		nPos+=2;
	}
	
	//send ROBOT IP/PORT 给MOBILE
	if(SendStream(64,cContent,nSockFD))
	{

	}
	return 0;
}

int IoTRobot_NetServer::AnwserRobotRequests(int nSockFD,BYTE *pucClientIP)
{
	char cText1[100];
	int nPortNum;
	IoTRobot_TCPIPParam * pstTCPIPParam;
	memcpy(cText1,g_pcAnswerModel,88);
	cText1[89]='\0';


	if((pstTCPIPParam=GetValidCIPos())!=NULL)
	{
		m_nCnCNum++;
		memcpy(&m_nIMUState,&m_stDataPtrSet.pcTmpBuff[92],4);
		memcpy(&pstTCPIPParam->nClientRTPVideoPort,&m_stDataPtrSet.pcTmpBuff[59],4);
		memcpy(&pstTCPIPParam->nClientRTPAudioPort,&m_stDataPtrSet.pcTmpBuff[75],4);
		memcpy(pstTCPIPParam->ucClientIP,pucClientIP,4);

		nPortNum=pstTCPIPParam->nCmdPort;
		memcpy(&cText1[12],&nPortNum,4);
		nPortNum=pstTCPIPParam->nSLAMPort;
		memcpy(&cText1[29],&nPortNum,4);
		nPortNum=pstTCPIPParam->nIDrivePort;
		memcpy(&cText1[46],&nPortNum,4);
		nPortNum=pstTCPIPParam->nServerRTPVideoPort;
		memcpy(&cText1[65],&nPortNum,4);
		nPortNum=pstTCPIPParam->nServerRTPAudioPort;
		memcpy(&cText1[84],&nPortNum,4);

		CreateThreadForEachSock(pstTCPIPParam);
		if(SendStream(88,cText1,nSockFD))
		{
		}
	}
	else
	{
		printf("Connection Number Has Reached Max!!!! \n");
		return -1;
	}
	return 0;
}



UINT IoTRobot_NetServer::ThreadSLAMSock(LPVOID lpParam)
{
	int nNewFd;
	ThreadParam *pParam=(ThreadParam*)lpParam;
	struct sockaddr_in ClientAddr;
	int nAddrLen=sizeof(struct sockaddr_in);
	int nSLAMDataLen=0;
	char cSLAMDataLenBuff[4];
	char cTmp[4];
	LPDWORD ID=0;
	char cIMUData[48];



	if(listen(pParam->pstTCPIPParam->nSockSLAM,BACKLOG)==-1)
	{
		printf("listen error!! \n");
		return -1;
	}

	while (!pParam->pCNetServer->m_bStopThreadSLAMSock)
	{
		if((nNewFd=accept(pParam->pstTCPIPParam->nSockSLAM,
			(struct sockaddr *)&ClientAddr,&nAddrLen))==-1)
		{
			printf("accept error!!! \n");
			continue;
		}
		while(!pParam->pCNetServer->m_bStopThreadSLAMSock)
		{
			if (pParam->pCNetServer->RcvStream(4,cTmp,nNewFd))
			{
				if(memcmp(cTmp,"SLAM",4)==0)
				{
					if(pParam->pCNetServer->RcvStream(4,cSLAMDataLenBuff,nNewFd)!=-1)
					{
						memcpy(&nSLAMDataLen,cSLAMDataLenBuff,4);
						if(pParam->pCNetServer->RcvStream(nSLAMDataLen,pParam->pCNetServer->m_stDataPtrSet.pcTmpSLAMBuff,nNewFd))
						{
							memcpy(cIMUData,&nSLAMDataLen,4);
							if (pParam->pCNetServer->m_stCallBackSet.cbSlam!=NULL)
							{
								pParam->pCNetServer->m_stCallBackSet.cbSlam((unsigned char*)pParam->pCNetServer->m_stDataPtrSet.pcTmpSLAMBuff,
									cIMUData);
							}
						}
						else
						{
							printf("Lost connection!!!!!!!!!!! \n");
							return 0;
						}
					}
					else
					{
						printf("Lost connection!!!!!!!!!!! \n");
						return 0;
					}
				}
				else
				{
					Sleep(30);
				}
			}
			else
			{
				return -1;
			}
		}
	}
}

UINT IoTRobot_NetServer::ThreadCmdSock(LPVOID lpParam)
{
	int nNewFd;
	ThreadParam *pParam=(ThreadParam*)lpParam;
	struct sockaddr_in ClientAddr;
	int nAddrLen=sizeof(struct sockaddr_in);

	if(listen(pParam->pstTCPIPParam->nSockCmd,BACKLOG)==-1)
	{
		printf("listen error!! \n");
		return -1;
	}

	while (!pParam->pCNetServer->m_bStopThreadCmdSock)
	{
		if((nNewFd=accept(pParam->pstTCPIPParam->nSockCmd,
			(struct sockaddr *)&ClientAddr,&nAddrLen))==-1)
		{
			printf("accept error!!! \n");
			continue;
		}
		pParam->pstTCPIPParam->nCmdNewFd=nNewFd;
	}

	Sleep(30);
}

UINT IoTRobot_NetServer::ThreadIDriveSock(LPVOID lpParam)
{
	int nNewFd;
	ThreadParam *pParam=(ThreadParam*)lpParam;
	struct sockaddr_in ClientAddr;
	int nAddrLen=sizeof(struct sockaddr_in);

	if(listen(pParam->pstTCPIPParam->nSockIDrive,BACKLOG)==-1)
	{
		printf("listen error!! \n");
		return -1;
	}


	while (!pParam->pCNetServer->m_bStopThreadIDriveSock)
	{
		if((nNewFd=accept(pParam->pstTCPIPParam->nSockIDrive,
			(struct sockaddr *)&ClientAddr,&nAddrLen))==-1)
		{
			printf("accept error!!! \n");
			continue;
		}
		pParam->pstTCPIPParam->nIDriveNewFd=nNewFd;
		while(!pParam->pCNetServer->m_bStopThreadIDriveSock)
		{
			if(pParam->pCNetServer->RcvStream(18,pParam->pCNetServer->m_stDataPtrSet.pcTmpBuff,nNewFd))
			{
				printf("$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$\n");
				if (memcmp(pParam->pCNetServer->m_stDataPtrSet.pcTmpBuff,"AutoDriveExtraHops",18)==0)
				{
					pParam->pCNetServer->RcvStream(8,pParam->pCNetServer->m_stDataPtrSet.pcTmpBuff,nNewFd);
					pParam->pCNetServer->m_stCallBackSet.cdData2OpenGl((unsigned char*)pParam->pCNetServer->m_stDataPtrSet.pcTmpBuff,8,NULL);
				}
				else if(memcmp(pParam->pCNetServer->m_stDataPtrSet.pcTmpBuff,"AutoDriveSuccessfu",18)==0)
				{
					unsigned char ucBuff[10]="success";
					pParam->pCNetServer->m_stCallBackSet.cdData2OpenGl((unsigned char*)ucBuff,10,NULL);
				}
				else if(memcmp(pParam->pCNetServer->m_stDataPtrSet.pcTmpBuff,"MobileMasterRobot!",18)==0)
				{
					unsigned char ucTmp[18];
					memcpy(ucTmp,"MobileMasterRobot!",18);
					pParam->pCNetServer->m_stCallBackSet.cbData2Kernel(ucTmp,18,NULL);
				}
				else if (memcmp(pParam->pCNetServer->m_stDataPtrSet.pcTmpBuff,"MobileAbandonRobot",18)==0)
				{
					unsigned char ucTmp[18];
					memcpy(ucTmp,"MobileAbandonRobot",18);
					pParam->pCNetServer->m_stCallBackSet.cbData2Kernel(ucTmp,18,NULL);
				}
				else if (memcmp(pParam->pCNetServer->m_stDataPtrSet.pcTmpBuff,"MileStone",9)==0)
				{
					printf("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");
					unsigned char ucTmp[18];
					memcpy(ucTmp,pParam->pCNetServer->m_stDataPtrSet.pcTmpBuff,18);
					pParam->pCNetServer->m_stCallBackSet.cdData2OpenGl(ucTmp,18,NULL);
				}
				//pParam->pCNetServer->m_stDataPtrSet.pcTmpBuff[5]='\0';
			}
		}
	}

	Sleep(30);
}

int IoTRobot_NetServer::CreateThreadForEachSock(IoTRobot_TCPIPParam * pstTCPIPParam)
{
	g_stThreadParam.pCNetServer=this;
	g_stThreadParam.pstTCPIPParam=pstTCPIPParam;
	pstTCPIPParam->hThreadCmd=CreateThread(NULL,0,(LPTHREAD_START_ROUTINE)ThreadCmdSock,&g_stThreadParam,0,pstTCPIPParam->ID_Cmd);
	pstTCPIPParam->hThreadSLAM=CreateThread(NULL,0,(LPTHREAD_START_ROUTINE)ThreadSLAMSock,&g_stThreadParam,0,pstTCPIPParam->ID_SLAM);
	pstTCPIPParam->hThreadIDrive=CreateThread(NULL,0,(LPTHREAD_START_ROUTINE)ThreadIDriveSock,&g_stThreadParam,0,pstTCPIPParam->ID_IDrive);
	Sleep(30);
	return 1;
}






IoTRobot_TCPIPParam * IoTRobot_NetServer::GetValidCIPos()
{
	int i;
	for (i=0;i<MAX_CLIENT_NUM;i++)
	{
		if (m_stTCPIPParamSet.nOccupyFlag[i]==0)
		{
			m_stTCPIPParamSet.nOccupyFlag[i]=1;
			return &m_stTCPIPParamSet.stTCPIPParam[i];
		}
	}
	return NULL;
}



int IoTRobot_NetServer::NetServer_GetDeviceState(int *pnState)
{
	if (m_nIMUState!=-1)
	{
		*pnState=m_nIMUState;
		return 0;
	}
	return -1;
}


int IoTRobot_NetServer::NetServer_Stop()
{
	m_bStopListen=true;
	m_bStopThreadSLAMSock=true;
	m_bStopThreadCmdSock=true;
	m_bStopThreadIDriveSock=true;
	
	WaitForSingleObject(m_stTCPIPParamSet.stTCPIPParam[0].hThreadCmd,INFINITE);
	WaitForSingleObject(m_stTCPIPParamSet.stTCPIPParam[0].hThreadIDrive,INFINITE);
	WaitForSingleObject(m_stTCPIPParamSet.stTCPIPParam[0].hThreadSLAM,INFINITE);


	closesocket(m_nSockCmd);
	closesocket(m_stTCPIPParamSet.stTCPIPParam[0].nSockCmd);
	closesocket(m_stTCPIPParamSet.stTCPIPParam[0].nSockSLAM);
	closesocket(m_stTCPIPParamSet.stTCPIPParam[0].nSockIDrive);


	return 0;
}

int IoTRobot_NetServer::NetServer_Uninit()
{
	if (m_pcMemoryPool!=NULL)
	{
		delete [] m_pcMemoryPool;
		m_pcMemoryPool=NULL;
	}
	return 0;
}


int IoTRobot_NetServer::SendCtlCmd(char *pcContent,int nLen,int nIdx,void *pContext)
{
	IoTRobot_NetServer *pServer=(IoTRobot_NetServer *)pContext;
	if (pServer->m_stTCPIPParamSet.nOccupyFlag[nIdx])
	{
		if (pServer->m_stTCPIPParamSet.stTCPIPParam[nIdx].nIDriveNewFd!=0)
		{
			if(pServer->SendStream(nLen,pcContent,pServer->m_stTCPIPParamSet.stTCPIPParam[nIdx].nIDriveNewFd))
			{
				return 0;
			}
			else
			{
				return -1;
			}
		}
		else return -2;
		
	}
	else return -3;
}


int IoTRobot_NetServer::SendSysCmd(char *pcContent,int nLen,int nIdx,void *pContext)
{
	IoTRobot_NetServer *pServer=(IoTRobot_NetServer *)pContext;

	if (pServer->m_stTCPIPParamSet.nOccupyFlag[nIdx])
	{
		if (pServer->m_stTCPIPParamSet.stTCPIPParam[nIdx].nCmdNewFd!=0)
		{
			if(pServer->SendStream(nLen,pcContent,pServer->m_stTCPIPParamSet.stTCPIPParam[nIdx].nCmdNewFd))
			{
				return 0;
			}
			else
			{
				return -1;
			}
		}
		else return -2;

	}
	else return -3;
}






















/**************************************************************************************************************************************/
/*****************************************************************************************************************************************/
/********************************************************************************************************************************************/

/*


UINT IoTRobot_NetServer::ThreadRecvRealTimeVideo(LPVOID lpParam)
{
int nRslt,nNewFd;
ThreadParam *pParam=(ThreadParam*)lpParam;
pParam->pCNetServer->m_CRTP.RTPRecvData();
return 0;
}


int IoTRobot_NetServer::VGA2QVGA(unsigned char *pucVGA,unsigned char *pucQVGA)
{
	int i,j;
	unsigned char ucTmp;
	unsigned char *pucPixel=pucQVGA+320*240*3-3;
	unsigned char *pucOrgImg=pucVGA;


	j=0;
	for(i=0;i<320*240;i++)
	{
		*pucPixel=*(pucOrgImg);
		*(pucPixel+1)=*(pucOrgImg+1);
		*(pucPixel+2)=*(pucOrgImg+2);
		pucPixel-=3;
		pucOrgImg+=6;
		j++;
		if(j==320)
		{
			j=0;
			pucOrgImg+=640*3;
		}
	}

	return 1;
}
int IoTRobot_NetServer::ChangeRunMode(int nIdx,int nMode)
{
	IoTRobot_RTPParam stRTPVideoParam;
	if (m_nRunMode!=nMode)
	{
		if (nMode==0)
		{
			m_CRTVA.RealTimeVAStop();
			m_CRTVA.RealTimeVAUninit();
		}
		else
		{
			stRTPVideoParam.pucDestIPAddr=m_stNetCleintInfoSet.stNetClientInfo[nIdx].ucClientIP;
			stRTPVideoParam.usDestPort=m_stNetCleintInfoSet.stNetClientInfo[nIdx].nClientRTPVideoPort;
			stRTPVideoParam.usMyPort=m_stNetCleintInfoSet.stNetClientInfo[nIdx].nServerRTPVideoPort;



		//	m_CRTVA.RealTimeVAInit(stRTPParam,stRTPParam,
		//		m_stCallBackSet.cbDisplay,m_stCallBackSet.pDisplayContext,20);
		//	m_CRTVA.RealTimeVARun();


			//m_CRTV.RealTimeVideoInit(m_stNetCleintInfoSet.stNetClientInfo[nIdx].nServerRTPPort,
			//	m_stNetCleintInfoSet.stNetClientInfo[nIdx].ucClientIP,
			//	m_stNetCleintInfoSet.stNetClientInfo[nIdx].nClientRTPPort,0,
			//	m_stCallBackSet.cbDisplay,m_stCallBackSet.pDisplayContext,20);
			//m_CRTV.RealTimeVideoRun();
		}
		m_nRunMode=nMode;
	}
	return 0;
}

#ifdef H263_COMPRESS
IoTRobot_Decode_H263 IoTRobot_NetServer::m_CH263Decode;
#endif

char *g_pcKinect_Conf = "Conf: x, x, x, x, x, x, x, x";
char *g_pcKinect_Conf_Init="Conf: 0, 0, 0, 0, 0, 0, 0, 0";


char *g_pcKinect_NoConf="No_Conf";

#define  KINECT_NET_NO_CONF_LEN 7

typedef struct ThreadParam
{
	IoTRobot_NetServer *pCNetServer;
	IoT_NetClientInfo * pstCI;
	int nNewFd;
}ThreadParam;
ThreadParam g_stThreadParam;


int IoTRobot_NetServer::NetServer_Init(IoT_NetCallBackSet stCallBackSet)
{
	int i,j,nPort,nCount=0,nRslt;
	char *pcTmp1=NULL;
	WORD wVersionRequested;
	WSADATA wsaData;


	if(GetHostIP(m_ucHostIP)==0)
	{
		wVersionRequested = MAKEWORD( 1, 1 );
		WSAStartup( wVersionRequested, &wsaData );
		memset(&m_stNetCleintInfoSet,0,sizeof(IoT_NetClientInfoSet));

		nPort=MY_PORT;
		for (i=0;i<MAX_CLIENT_NUM;i++)
		{
			m_stNetCleintInfoSet.stNetClientInfo[i].nCmdPort=(++nPort);
			m_stNetCleintInfoSet.stNetClientInfo[i].nKinectPort=(++nPort);
			m_stNetCleintInfoSet.stNetClientInfo[i].nIDrivePort=(++nPort);
			m_stNetCleintInfoSet.stNetClientInfo[i].nServerRTPVideoPort=(++nPort);//RTP Port must be odd
			nPort++;
			m_stNetCleintInfoSet.stNetClientInfo[i].nServerRTPAudioPort=(++nPort);//RTP Port must be odd

			m_stNetCleintInfoSet.stNetClientInfo[i].CmdAddr.sin_family=AF_INET;
			m_stNetCleintInfoSet.stNetClientInfo[i].CmdAddr.sin_addr.S_un.S_un_b.s_b1=m_ucHostIP[0];
			m_stNetCleintInfoSet.stNetClientInfo[i].CmdAddr.sin_addr.S_un.S_un_b.s_b2=m_ucHostIP[1];
			m_stNetCleintInfoSet.stNetClientInfo[i].CmdAddr.sin_addr.S_un.S_un_b.s_b3=m_ucHostIP[2];
			m_stNetCleintInfoSet.stNetClientInfo[i].CmdAddr.sin_addr.S_un.S_un_b.s_b4=m_ucHostIP[3];


			m_stNetCleintInfoSet.stNetClientInfo[i].CmdAddr.sin_port=htons(m_stNetCleintInfoSet.stNetClientInfo[i].nCmdPort);


			if ((m_stNetCleintInfoSet.stNetClientInfo[i].nSockCmd = socket(AF_INET, SOCK_STREAM, 0)) == -1) {
				return -1;
			}
			if (nRslt=bind(m_stNetCleintInfoSet.stNetClientInfo[i].nSockCmd,
				(struct sockaddr *)&m_stNetCleintInfoSet.stNetClientInfo[i].CmdAddr,
				sizeof(struct sockaddr))==-1)
			{
				return -2;
			}



			m_stNetCleintInfoSet.stNetClientInfo[i].KinectAddr.sin_family=AF_INET;
			m_stNetCleintInfoSet.stNetClientInfo[i].KinectAddr.sin_addr.S_un.S_un_b.s_b1=m_ucHostIP[0];
			m_stNetCleintInfoSet.stNetClientInfo[i].KinectAddr.sin_addr.S_un.S_un_b.s_b2=m_ucHostIP[1];
			m_stNetCleintInfoSet.stNetClientInfo[i].KinectAddr.sin_addr.S_un.S_un_b.s_b3=m_ucHostIP[2];
			m_stNetCleintInfoSet.stNetClientInfo[i].KinectAddr.sin_addr.S_un.S_un_b.s_b4=m_ucHostIP[3];
			m_stNetCleintInfoSet.stNetClientInfo[i].KinectAddr.sin_port=htons(m_stNetCleintInfoSet.stNetClientInfo[i].nKinectPort);
			if ((m_stNetCleintInfoSet.stNetClientInfo[i].nSockKinect = socket(AF_INET, SOCK_STREAM, 0)) == -1) {
				return -3;
			}
			if (nRslt=bind(m_stNetCleintInfoSet.stNetClientInfo[i].nSockKinect,
				(struct sockaddr *)&m_stNetCleintInfoSet.stNetClientInfo[i].KinectAddr,
				sizeof(struct sockaddr))==-1)
			{
				return -4;
			}


			m_stNetCleintInfoSet.stNetClientInfo[i].IDriveAddr.sin_family=AF_INET;
			m_stNetCleintInfoSet.stNetClientInfo[i].IDriveAddr.sin_addr.S_un.S_un_b.s_b1=m_ucHostIP[0];
			m_stNetCleintInfoSet.stNetClientInfo[i].IDriveAddr.sin_addr.S_un.S_un_b.s_b2=m_ucHostIP[1];
			m_stNetCleintInfoSet.stNetClientInfo[i].IDriveAddr.sin_addr.S_un.S_un_b.s_b3=m_ucHostIP[2];
			m_stNetCleintInfoSet.stNetClientInfo[i].IDriveAddr.sin_addr.S_un.S_un_b.s_b4=m_ucHostIP[3];
			m_stNetCleintInfoSet.stNetClientInfo[i].IDriveAddr.sin_port=htons(m_stNetCleintInfoSet.stNetClientInfo[i].nIDrivePort);
			if ((m_stNetCleintInfoSet.stNetClientInfo[i].nSockIDrive = socket(AF_INET, SOCK_STREAM, 0)) == -1) {
				return -5;
			}
			if (nRslt=bind(m_stNetCleintInfoSet.stNetClientInfo[i].nSockIDrive,
				(struct sockaddr *)&m_stNetCleintInfoSet.stNetClientInfo[i].IDriveAddr,
				sizeof(struct sockaddr))==-1)
			{
				return -6;
			}
		}


		m_pcMemoryPool=new char[IOT_MEMORY_POOL_SIZE];
		memset(m_pcMemoryPool,0,IOT_MEMORY_POOL_SIZE);

		pcTmp1=m_pcMemoryPool;

		MEMORY_ALLOCATION(m_stDataPtrSet.pTmpBuff,char*,pcTmp1,IOT_TMP_BUFF_SIZE,nCount);
		MEMORY_ALLOCATION(m_stDataPtrSet.pTmpKinectBuff,char*,pcTmp1,IOT_TMP_BUFF_SIZE,nCount);
		MEMORY_ALLOCATION(m_stDataPtrSet.pTmpCmdBuff,char*,pcTmp1,IOT_TMP_BUFF_SIZE,nCount);
		MEMORY_ALLOCATION(m_stDataPtrSet.pTmpIDriveBuff,char*,pcTmp1,IOT_TMP_BUFF_SIZE,nCount);
		MEMORY_ALLOCATION(m_stDataPtrSet.pucRGB,unsigned char*,pcTmp1,IOT_RGBBUFF_SIZE,nCount);
		MEMORY_ALLOCATION(m_stDataPtrSet.pusDepth,unsigned short*,pcTmp1,IOT_DEPTHBUFF_SIZE,nCount);
		MEMORY_ALLOCATION(m_stDataPtrSet.pucQVGARGB,unsigned char*,pcTmp1,IOT_QVGARGBBUFF_SIZE,nCount);

		MEMORY_ALLOCATION(m_stDataPtrSet.pucCIFBuff,unsigned char*,pcTmp1,352*288*3,nCount);
		MEMORY_ALLOCATION(m_stDataPtrSet.pucBGB24,unsigned char*,pcTmp1,352*288*3,nCount);
		MEMORY_ALLOCATION(m_stDataPtrSet.pucR,unsigned char*,pcTmp1,352*288,nCount);
		MEMORY_ALLOCATION(m_stDataPtrSet.pucG,unsigned char*,pcTmp1,352*288,nCount);
		MEMORY_ALLOCATION(m_stDataPtrSet.pucB,unsigned char*,pcTmp1,352*288,nCount);

		MEMORY_ALLOCATION(m_stDataPtrSet.pucRGBBuffA,unsigned char*,pcTmp1,IOT_RGBBUFF_SIZE,nCount);
		MEMORY_ALLOCATION(m_stDataPtrSet.pucEncodeBuff,unsigned char*,pcTmp1,400000,nCount);
		MEMORY_ALLOCATION(m_stDataPtrSet.pcSLAMData,char*,pcTmp1,1024*1024*5,nCount);

		m_stCallBackSet=stCallBackSet;

#ifdef H263_COMPRESS
		m_CH263Decode.IoTRobot_DecodeInit();
		m_CH263Encode.IoTRobot_EncodeInit(10,10);
#endif
		//	m_CUSBCam.USBCameraInit(0,CallBack_USBCamera,this);
		m_nIMUState=-1;
		return 0;
	}
	return -7;
}

int main()
{
IoTRobot_NetServer test;
IoT_NetCallBackSet stCallback;

memset(&stCallback,0,sizeof(IoT_NetCallBackSet));
NetServerCallBack tmp=NULL;
test.NetServer_Init(stCallback);
test.NetServer_Run();
return 0;
}

void IoTRobot_NetServer::CallBack_USBCamera(unsigned char *pucImgData,void *pContext)
{
	int i;
	IoTRobot_NetServer *pNetServer=(IoTRobot_NetServer *)pContext;
	for (i=0;i<320*240;i++)
	{
		pNetServer->m_stDataPtrSet.pucRGBBuffA[i*3]=pucImgData[i*3+2];
		pNetServer->m_stDataPtrSet.pucRGBBuffA[i*3+1]=pucImgData[i*3+1];
		pNetServer->m_stDataPtrSet.pucRGBBuffA[i*3+2]=pucImgData[i*3];
	}
	//memcpy(pNetServer->m_stDataPtrSet.pucRGBBuffA,pucImgData,320*240*3);
}

UINT IoTRobot_NetServer::ThreadSendRealTimeVideo(LPVOID lpParam)
{
	//DWORD start=0,end=0;
	unsigned char ucLook[200];
	int i;
	int nRslt,nNewFd,nEncodeBuffLen;
	ThreadParam *pParam=(ThreadParam*)lpParam;
	unsigned char cTmp[320*240*3];
	//unsigned char pcSendData[]="I am Host!!!!";
	while (!pParam->pCNetServer->m_bStopSendRealTimeVideo)
	{
#ifdef H263_COMPRESS
		pParam->pCNetServer->m_CH263Encode.IoTRobot_EncodeOneFrame(pParam->pCNetServer->m_stDataPtrSet.pucRGBBuffA,
			pParam->pCNetServer->m_stDataPtrSet.pucEncodeBuff,&nEncodeBuffLen,VGA_RGB24);
#endif

#ifdef JPEG_COMPRESS
		for (i=0;i<320*240;i++)
		{
			cTmp[i*3]=pParam->pCNetServer->m_stDataPtrSet.pucRGBBuffA[i*3+2];
			cTmp[i*3+1]=pParam->pCNetServer->m_stDataPtrSet.pucRGBBuffA[i*3+1];
			cTmp[i*3+2]=pParam->pCNetServer->m_stDataPtrSet.pucRGBBuffA[i*3];
		}
		//pParam->pCNetServer->m_CJPEG.RGBToJpg(320,240,pParam->pCNetServer->m_stDataPtrSet.pucRGBBuffA,
		//	&pParam->pCNetServer->m_stDataPtrSet.pucEncodeBuff,(unsigned long *)&nEncodeBuffLen,20);

		pParam->pCNetServer->m_CJPEG.RGBToJpg(320,240,cTmp,
			&pParam->pCNetServer->m_stDataPtrSet.pucEncodeBuff,(unsigned long *)&nEncodeBuffLen,20);

		memcpy(ucLook,pParam->pCNetServer->m_stDataPtrSet.pucEncodeBuff,200);
		//	pParam->pstNetClient->m_CJPEG.RGBToJpg(QVGA_X,QVGA_Y,m_stDataPtrSet.pucQVGARGBBuffA,
		//		&m_stDataPtrSet.pcEncodeData,(unsigned long *)&nEncodeDataLen,pParam->pstNetClient->m_nJPEGQulity);
#endif
		pParam->pCNetServer->m_CRTP.RTPSendData(pParam->pCNetServer->m_stDataPtrSet.pucEncodeBuff,nEncodeBuffLen);
	}
	return 0;
}

#pragma pack(1)
void IoTRobot_NetServer::CallBack_RTPData(unsigned char*pucRTPData,unsigned int uiDataLen,void *pContext)
{
	IoTRobot_NetServer *pNetServer=(IoTRobot_NetServer *)pContext;
	int nDecodeBuffLen=352*288*3;

	unsigned char ucLook[200];
#ifdef H263_COMPRESS

	pNetServer->m_CH263Decode.IoTRobot_DecodeOneFrame(pucRTPData,uiDataLen,pNetServer->m_stDataPtrSet.pucCIFBuff,nDecodeBuffLen);
	pNetServer->m_stCallBackSet.cbDisplay(pNetServer->m_stDataPtrSet.pucCIFBuff,
		pNetServer->m_stCallBackSet.pDisplayContext);
#endif


	//这里可以优化,直接用jpeg刷屏，以后完善
#ifdef JPEG_COMPRESS
	pNetServer->m_CJPEG.jpgToRGB(pucRTPData,uiDataLen,pNetServer->m_stDataPtrSet.pucQVGARGB,(unsigned int*)&nDecodeBuffLen);
	memcpy(ucLook,pNetServer->m_stDataPtrSet.pucQVGARGB,200);
	pNetServer->m_stCallBackSet.cbDisplay(pNetServer->m_stDataPtrSet.pucQVGARGB,
		pNetServer->m_stCallBackSet.pDisplayContext);
#endif

}


int IoTRobot_NetServer::AnwserClientRequests(int nSockFD,BYTE *pucClientIP)
{
char cText1[65];
int nPortNum;
IoT_NetClientInfo * pstCI;
memcpy(cText1,g_pcText1,64);
cText1[64]='\0';
if(RcvStream(76,m_stDataPtrSet.pTmpBuff,nSockFD))
{
m_stDataPtrSet.pTmpBuff[77]='\0';
}
if((pstCI=GetValidCIPos())!=NULL)
{
memcpy(&m_nIMUState,&m_stDataPtrSet.pTmpBuff[72],4);
memcpy(&pstCI->nClientRTPPort,&m_stDataPtrSet.pTmpBuff[54],4);
memcpy(pstCI->ucClientIP,pucClientIP,4);

nPortNum=pstCI->nCmdPort;
memcpy(&cText1[12],&nPortNum,4);
nPortNum=pstCI->nKinectPort;
memcpy(&cText1[29],&nPortNum,4);
nPortNum=pstCI->nIDrivePort;
memcpy(&cText1[46],&nPortNum,4);
nPortNum=pstCI->nServerRTPPort;
memcpy(&cText1[60],&nPortNum,4);

//	printf("reach1!!!!!!!!!!!!!!\n");


//m_CRTP.RTPInit(pstCI->nServerRTPPort,pstCI->ucClientIP,pstCI->nClientRTPPort,0,CallBack_RTPData,this);
//m_CRTP.RTPRun();
//	printf("reach2!!!!!!!!!!!!!!\n");
m_bStopSendRealTimeVideo=false;
//	printf("reach3!!!!!!!!!!!!!!\n");
OpenThreadForClient(pstCI);
//	printf("reach4!!!!!!!!!!!!!!\n");

if(SendStream(64,cText1,nSockFD))
{
//printf("send %s",cText1);
}
//	m_CUSBCam.USBCameraRun();
}


return 0;

}


int IoTRobot_NetServer::NetServer_Conf(int nClientIdx,IoT_NetConf stNetConf)
{
	if (m_stNetCleintInfoSet.stNetClientInfo[nClientIdx].nKinectNewFd!=0
		&&m_stNetCleintInfoSet.stNetClientInfo[nClientIdx].nCmdNewFd!=0
		&&m_stNetCleintInfoSet.stNetClientInfo[nClientIdx].nIDriveNewFd!=0)
	{

		//	printf("start to send config!!!!\n");
		char tmp[KINECT_NET_CONF_LEN+1];
		tmp[KINECT_NET_CONF_LEN]='\0';
		memcpy(&m_stNetCleintInfoSet.stNetClientInfo[nClientIdx].stNetConf,&stNetConf,sizeof(IoT_NetConf));
		memcpy(m_cKinectConf,g_pcKinect_Conf,strlen(g_pcKinect_Conf));
		m_cKinectConf[KINECT_NET_CONF_LEN-1]='\0';
		//send cmd to kinect port
		m_cKinectConf[6]=stNetConf.stSLAMConf.cType;
		m_cKinectConf[9]=stNetConf.stSLAMConf.cFrequency;
		m_cKinectConf[12]=stNetConf.stSLAMConf.cResolution;
		m_cKinectConf[15]=stNetConf.stSLAMConf.cCompression;

		m_cKinectConf[18]=stNetConf.stVisulizationConf.cType;
		m_cKinectConf[21]=stNetConf.stVisulizationConf.cFrequency;
		m_cKinectConf[24]=stNetConf.stVisulizationConf.cResolution;
		m_cKinectConf[27]=stNetConf.stVisulizationConf.cCompression;

		memcpy(tmp,m_cKinectConf,KINECT_NET_CONF_LEN);
		SendStream(KINECT_NET_CONF_LEN,m_cKinectConf,m_stNetCleintInfoSet.stNetClientInfo[nClientIdx].nKinectNewFd);
		//printf("finish to send config!!!!\n");
		//send cmd to cmd port

		//send cmd to idrive port

		return 0;
	}
	else
	{
		return -1;
	}



	int IoTRobot_NetServer::NetServer_GetOneSlamFrame(int nClientIdx)
	{
	if (m_stNetCleintInfoSet.stNetClientInfo[nClientIdx].nKinectNewFd!=0)
	{
	if (m_stNetCleintInfoSet.stNetClientInfo[nClientIdx].stNetConf.stSLAMConf.cType==2)
	{
	SendStream(4,"GetO",m_stNetCleintInfoSet.stNetClientInfo[nClientIdx].nKinectNewFd);
	return 0;
	}
	else
	{
	return -2;
	}
	}
	else
	{
	return -1;

}*/