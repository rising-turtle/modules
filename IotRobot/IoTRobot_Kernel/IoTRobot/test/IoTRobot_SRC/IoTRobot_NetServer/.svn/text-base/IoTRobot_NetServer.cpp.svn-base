#include "IoTRobot_NetServer.h"
#include "IoTRobot_NetProtocol.h"
/*#include <string.h>
#include <vtkConeSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkImageViewer.h>
#include <../io/vtkBMPReader.h>
#include <../Filtering/vtkImageData.h>
#include <../Filtering/vtkPointData.h>

#include <vtkGraphicsFactory.h>
#include <vtkWin32RenderWindowInteractor.h>
#include <vtkCommand.h>
*/

#pragma once
char *g_pcText1="OK. CmdPort:xxxx, KinectPort:xxxx, IDrivePort:xxxx.";

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
IoTRobot_NetServer::IoTRobot_NetServer(void)
{
	m_bNewKinectConf=false;
	memset(&m_stNetCleintInfoSet,0,sizeof(IoT_NetClientInfoSet));
	//memset(&m_stNetConf,0,sizeof(IoT_NetConf));
	//NetServer_Conf(m_stNetConf);
}

IoTRobot_NetServer::~IoTRobot_NetServer(void)
{
}


/*int IoTRobot_NetServer::NetServer_Conf(int nClientIdx,IoT_NetConf stNetConf)
{
	memcpy(&m_stNetConf,&stNetConf,sizeof(IoT_NetConf));
	memcpy(m_cKinectConf,g_pcKinect_Conf,strlen(g_pcKinect_Conf));

	m_cKinectConf[6]=stNetConf.stSLAMConf.cType;
	m_cKinectConf[9]=stNetConf.stSLAMConf.cFrequency;
	m_cKinectConf[12]=stNetConf.stSLAMConf.cResolution;
	m_cKinectConf[15]=stNetConf.stSLAMConf.cCompression;

	m_cKinectConf[18]=stNetConf.stVisulizationConf.cType;
	m_cKinectConf[21]=stNetConf.stVisulizationConf.cFrequency;
	m_cKinectConf[24]=stNetConf.stVisulizationConf.cResolution;
	m_cKinectConf[27]=stNetConf.stVisulizationConf.cCompression;

	m_bNewKinectConf=true;
	return 0;
}*/
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
		//printf("%s\n",hostname);//计算机名字 
	} 



	struct hostent *pHost = gethostbyname(hostname); 
	/*for (int i = 0; pHost != NULL && pHost->h_addr_list[i] != NULL; i++) 
	{ 
	printf("%s\n",inet_ntoa(*(struct in_addr *)pHost->h_addr_list[i])); 
	}*/ 


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

int IoTRobot_NetServer::NetServer_Init(IoT_NetCallBackSet stCallBackSet)
{
	int i,j,nPort,nCount=0,nRslt;
	char *pcTmp1=NULL;
	WORD wVersionRequested;
	WSADATA wsaData;


	if(GetHostIP(m_ucHostIP)==0)
	{
	
	//	printf("Get Ip goooooooooooooooood!!!! %d.%d.%d.%d \n",m_ucHostIP[0],m_ucHostIP[1],m_ucHostIP[2],m_ucHostIP[3]);
		wVersionRequested = MAKEWORD( 1, 1 );
		WSAStartup( wVersionRequested, &wsaData );
		memset(&m_stNetCleintInfoSet,0,sizeof(IoT_NetClientInfoSet));

		nPort=MY_PORT;
		for (i=0;i<MAX_CLIENT_NUM;i++)
		{
			m_stNetCleintInfoSet.stNetClientInfo[i].nCmdPort=(++nPort);
			m_stNetCleintInfoSet.stNetClientInfo[i].nKinectPort=(++nPort);
			m_stNetCleintInfoSet.stNetClientInfo[i].nIDrivePort=(++nPort);

			m_stNetCleintInfoSet.stNetClientInfo[i].CmdAddr.sin_family=AF_INET;
		//	m_stNetCleintInfoSet.stNetClientInfo[i].CmdAddr.sin_addr.s_addr=inet_addr(MY_IP);
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
		//	m_stNetCleintInfoSet.stNetClientInfo[i].KinectAddr.sin_addr.s_addr=inet_addr(MY_IP);

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
			//m_stNetCleintInfoSet.stNetClientInfo[i].IDriveAddr.sin_addr.s_addr=inet_addr(MY_IP);

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

		m_stCallBackSet=stCallBackSet;

		return 0;
	}
//	printf("Get Ip baaaaaaaaaaaaaaaaaaaaaaad!!!!  \n");
	return -7;
}
int IoTRobot_NetServer::NetServer_Run()
{

	int nRslt,nNewFd;
	struct sockaddr_in ClientAddr;
	int nAddrLen=sizeof(struct sockaddr_in);



	if ((m_nSockCmd = socket(AF_INET, SOCK_STREAM, 0)) == -1) {
		return -1;
	}

	m_MyAddr.sin_family=AF_INET;
	m_MyAddr.sin_port=htons(MY_PORT);
	//m_MyAddr.sin_addr.s_addr=inet_addr(MY_IP);
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
	m_bStopListen=true;
	while (m_bStopListen)
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
		AnwserClientRequests(nNewFd);
	}

	WSACleanup();
	return 1;
}

#pragma pack(1)

int IoTRobot_NetServer::AnwserClientRequests(int nSockFD)
{
	char cText1[52];
	int nPortNum;
	IoT_NetClientInfo * pstCI;
	memcpy(cText1,g_pcText1,51);
	cText1[51]='\0';
	if(RcvStream(37,m_stDataPtrSet.pTmpBuff,nSockFD))
	{
		m_stDataPtrSet.pTmpBuff[38]='\0';
	//	printf("First recv : %s",m_stDataPtrSet.pTmpBuff );
	}
	if((pstCI=GetValidCIPos())!=NULL)
	{
		nPortNum=pstCI->nCmdPort;
		memcpy(&cText1[12],&nPortNum,4);
		nPortNum=pstCI->nKinectPort;
		memcpy(&cText1[29],&nPortNum,4);
		nPortNum=pstCI->nIDrivePort;
		memcpy(&cText1[46],&nPortNum,4);

		OpenThreadForClient(pstCI);

		if(SendStream(51,cText1,nSockFD))
		{
			//printf("send %s",cText1);
		}
	}


	return 0;

}


/*UINT IoTRobot_NetServer::ThreadKinectSendCmd(LPVOID lpParam)
{
	ThreadParam *pParam=(ThreadParam*)lpParam;
	while (1)
	{
		if (pParam->pCNetServer->m_bNewKinectConf)
		{
			pParam->pCNetServer->SendStream(KINECT_NET_CONF_LEN,pParam->pCNetServer->m_cKinectConf,pParam->nNewFd);
			pParam->pCNetServer->m_bNewKinectConf=false;
		}

		if (pParam->pCNetServer->m_stNetConf.stSLAMConf.cType==2)
		{
			if (pParam->pCNetServer->m_bGetOneSlamFrame)
			{
				pParam->pCNetServer->SendStream(4,"GetO",pParam->nNewFd);
				pParam->pCNetServer->m_bGetOneSlamFrame=false;
			}
		}
		Sleep(10);
	}
}*/

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
	}
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
}

UINT IoTRobot_NetServer::ThreadKinectSock(LPVOID lpParam)
{
	int nRslt,nNewFd,i,j;
	ThreadParam *pParam=(ThreadParam*)lpParam;
	struct sockaddr_in ClientAddr;
	int nAddrLen=sizeof(struct sockaddr_in);
	char cTmp[4];
	LPDWORD ID=0;
	char cIMUData[48];



	if(listen(pParam->pstCI->nSockKinect,BACKLOG)==-1)
	{
		printf("listen error!! \n");
		return -1;
	}

	while (1)
	{
		if((nNewFd=accept(pParam->pstCI->nSockKinect,
			(struct sockaddr *)&ClientAddr,&nAddrLen))==-1)
		{
			printf("accept error!!! \n");
			continue;
		}
		pParam->pstCI->nKinectNewFd=nNewFd;
	//	pParam->pCNetServer->SendStream(KINECT_NET_CONF_LEN,g_pcKinect_Conf_Init,nNewFd);


		while(1)
		{
			if (RcvStream(4,cTmp,nNewFd))
			{
				//if (cTmp=="SLAM")
				if(memcmp(cTmp,"SLAM",4)==0)
				{
					//printf("start to Recv SLAM!!!!\n");
					RcvStream(IOT_RGBBUFF_SIZE,(char *)pParam->pCNetServer->m_stDataPtrSet.pucRGB,nNewFd);
					RcvStream(IOT_DEPTHBUFF_SIZE,(char *)pParam->pCNetServer->m_stDataPtrSet.pusDepth,nNewFd);
					RcvStream(4,cTmp,nNewFd);
					RcvStream(48,cIMUData,nNewFd);
					if (pParam->pCNetServer->m_stCallBackSet.cbSlam!=NULL)
					{
						pParam->pCNetServer->m_stCallBackSet.cbSlam_IMU(pParam->pCNetServer->m_stDataPtrSet.pucRGB,
							pParam->pCNetServer->m_stDataPtrSet.pusDepth,cIMUData,
							pParam->pCNetServer->m_stCallBackSet.pSlamContext);
					}


					//printf("finish to Recv SLAM!!!!\n");
				}
				//else if (cTmp=="VISU")
				else if (memcmp(cTmp,"VISU",4)==0)
				{
					//printf("start to Recv VISU!!!!\n");
					if(RcvStream(IOT_QVGARGBBUFF_SIZE,(char *)pParam->pCNetServer->m_stDataPtrSet.pucQVGARGB,nNewFd))
					{

						if (pParam->pCNetServer->m_stCallBackSet.cbDisplay!=NULL)
						{
							//printf("NetServer Display run normally!!! \n");
							//printf("Display callback address: %d \n",pParam->pCNetServer->m_stCallBackSet.cbDisplay);
							pParam->pCNetServer->m_stCallBackSet.cbDisplay(pParam->pCNetServer->m_stDataPtrSet.pucQVGARGB,
								pParam->pCNetServer->m_stCallBackSet.pDisplayContext);
						}
					}
					//printf("finish to Recv VISU!!!!\n");
				}
				else
				{
					Sleep(30);
				}
			}
		}
	}
}

UINT IoTRobot_NetServer::ThreadCmdSock(LPVOID lpParam)
{
	int nRslt,nNewFd;
	ThreadParam *pParam=(ThreadParam*)lpParam;
	struct sockaddr_in ClientAddr;
	int nAddrLen=sizeof(struct sockaddr_in);

	if(listen(pParam->pstCI->nSockCmd,BACKLOG)==-1)
	{
		printf("listen error!! \n");
		return -1;
	}

	while (1)
	{
		if((nNewFd=accept(pParam->pstCI->nSockCmd,
			(struct sockaddr *)&ClientAddr,&nAddrLen))==-1)
		{
			printf("accept error!!! \n");
			continue;
		}

		pParam->pstCI->nCmdNewFd=nNewFd;
		while(1)
		{
			if(RcvStream(4,pParam->pCNetServer->m_stDataPtrSet.pTmpBuff,nNewFd))
			{
				pParam->pCNetServer->m_stDataPtrSet.pTmpBuff[5]='\0';
				//printf("CMD recv : %s",pParam->pCNetServer->m_stDataPtrSet.pTmpBuff );
			}
		}



		int ff=1000;
	}

	Sleep(30);
}

UINT IoTRobot_NetServer::ThreadIDriveSock(LPVOID lpParam)
{
	int nRslt,nNewFd;
	ThreadParam *pParam=(ThreadParam*)lpParam;
	struct sockaddr_in ClientAddr;
	int nAddrLen=sizeof(struct sockaddr_in);

	if(listen(pParam->pstCI->nSockIDrive,BACKLOG)==-1)
	{
		printf("listen error!! \n");
		return -1;
	}


	while (1)
	{
		if((nNewFd=accept(pParam->pstCI->nSockIDrive,
			(struct sockaddr *)&ClientAddr,&nAddrLen))==-1)
		{
			printf("accept error!!! \n");
			continue;
		}
		pParam->pstCI->nIDriveNewFd=nNewFd;
		while(1)
		{
			if(RcvStream(4,pParam->pCNetServer->m_stDataPtrSet.pTmpBuff,nNewFd))
			{
				pParam->pCNetServer->m_stDataPtrSet.pTmpBuff[5]='\0';
				//printf("IDRive : %s",pParam->pCNetServer->m_stDataPtrSet.pTmpBuff );
			}
		}


		int ff=1000;
	}

	Sleep(30);
}


int IoTRobot_NetServer::OpenThreadForClient(IoT_NetClientInfo * pstCI)
{
	g_stThreadParam.pCNetServer=this;
	g_stThreadParam.pstCI=pstCI;
	pstCI->hThreadCmd=CreateThread(NULL,0,(LPTHREAD_START_ROUTINE)ThreadCmdSock,&g_stThreadParam,0,pstCI->ID_Cmd);
	pstCI->hThreadKinect=CreateThread(NULL,0,(LPTHREAD_START_ROUTINE)ThreadKinectSock,&g_stThreadParam,0,pstCI->ID_Kinect);
	pstCI->hThreadIDrive=CreateThread(NULL,0,(LPTHREAD_START_ROUTINE)ThreadIDriveSock,&g_stThreadParam,0,pstCI->ID_IDrive);
	Sleep(30);
	return 1;
}


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


/*int main()
{
	IoTRobot_NetServer test;
	NetServerCallBack tmp=NULL;
	test.NetServer_Init(tmp);
	test.NetServer_Run();
	return 0;
}*/

IoT_NetClientInfo * IoTRobot_NetServer::GetValidCIPos()
{
	int i;
	for (i=0;i<MAX_CLIENT_NUM;i++)
	{
		if (m_stNetCleintInfoSet.nOccupyFlag[i]==0)
		{
			m_stNetCleintInfoSet.nOccupyFlag[i]=1;
			return &m_stNetCleintInfoSet.stNetClientInfo[i];
		}
	}
	return NULL;
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