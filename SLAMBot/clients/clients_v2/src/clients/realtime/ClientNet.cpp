#include "Net/ClientNet.h"




ClientNet::ClientNet(void)
{
	m_nNetStatus=0;
}

ClientNet::~ClientNet(void)
{
}


int ClientNet::ComInit(void *pcParams)
{

	char *pcConfig=(char *)pcParams;
	WORD wVersionRequested;
	WSADATA wsaData;

	int nHostPort,nClientPort;
	unsigned char ucHostIP[4],ucClientIP[4];

	char cMyIPstr[16],cHostIPstr[16];
	memset(cMyIPstr,0,16);
	memset(cHostIPstr,0,16);

	memcpy(cMyIPstr,pcConfig,16);
	memcpy(cHostIPstr,pcConfig+20,16);


	ParseIP(cMyIPstr,(char *)ucClientIP);
	memcpy(&nClientPort,pcConfig+16,4);
	ParseIP(cHostIPstr,(char *)ucHostIP);
	memcpy(&nHostPort,pcConfig+36,4);


	m_nClientPort=nClientPort;
	memcpy(m_cClientIP,ucClientIP,4);


	wVersionRequested = MAKEWORD( 1, 1 );
	WSAStartup( wVersionRequested, &wsaData );


	SetAddr(nHostPort,ucHostIP,m_HostAddr);

	m_bStopListen=false;

	memset(m_cRegisterBuff,0,100);
	char *pcRegisterBuff=m_cRegisterBuff;
	pcRegisterBuff[0]=10;//register 
	pcRegisterBuff++;
	memcpy(pcRegisterBuff,&m_stRegisterInfo,sizeof(RegisterInfo));
	
	if((m_nClientSock=SetSocket(nClientPort,ucClientIP,m_ClientAddr))<0)
	{
		printf("Set Socket failed!!!\n");
		return -1;
	}
	printf("Com init finish!!!\n");
	return 0;
}
int ClientNet::SetAddr(int nPort,unsigned char *pucIP,sockaddr_in &addr)
{
	addr.sin_family=AF_INET;
	addr.sin_port=htons(nPort);
	addr.sin_addr.S_un.S_un_b.s_b1=pucIP[0];
	addr.sin_addr.S_un.S_un_b.s_b2=pucIP[1];
	addr.sin_addr.S_un.S_un_b.s_b3=pucIP[2];
	addr.sin_addr.S_un.S_un_b.s_b4=pucIP[3];
	memset(addr.sin_zero,0,8);
	return 0;
}


int ClientNet::SetSocket(int nPort,unsigned char *pucIP,sockaddr_in &addr)
{
	int nSock,nRtn;
	linger sLinger;

	BOOL bKeepAlive=TRUE;
	tcp_keepalive live,liveout;

	live.keepaliveinterval=10000;//每10秒发送探测包，发5次没有回应就断开
	live.keepalivetime=1000;//1秒没有数据，就发送探测包
	live.onoff=TRUE;

	sLinger.l_onoff=1;
	sLinger.l_linger=0;

	if ((nSock = socket(AF_INET, SOCK_STREAM, 0)) == -1) {
		return -3;
	}
	int nReuseAddr=1;
	setsockopt(nSock,SOL_SOCKET,SO_REUSEADDR,(char*)&nReuseAddr,sizeof(nReuseAddr));

	setsockopt(nSock ,SOL_SOCKET,SO_LINGER,(const char*)&sLinger,sizeof(linger));
	nRtn=setsockopt(nSock,SOL_SOCKET,SO_KEEPALIVE,(char*)&bKeepAlive,sizeof(bKeepAlive));
	if (nRtn==0)
	{
		DWORD dw;
		if (WSAIoctl(nSock,SIO_KEEPALIVE_VALS,&live,sizeof(live),&liveout,sizeof(liveout),&dw,NULL,NULL)==SOCKET_ERROR)
		{
			return -1;
		}
	}

	addr.sin_family=AF_INET;


	addr.sin_port=0;
	//addr.sin_port=htons(nPort);
	addr.sin_addr.S_un.S_un_b.s_b1=pucIP[0];
	addr.sin_addr.S_un.S_un_b.s_b2=pucIP[1];
	addr.sin_addr.S_un.S_un_b.s_b3=pucIP[2];
	addr.sin_addr.S_un.S_un_b.s_b4=pucIP[3];
	memset(addr.sin_zero,0, 8);

	if(bind(nSock,(struct sockaddr *)&addr,sizeof(struct sockaddr))==-1)
	{
		return -4;
	}
	return nSock;

}


int ClientNet::Try2CncHost()
{

	int i,nRtn=-1;
	int nCncRlst,nErr;
	printf("try 2 cnc with server!!!\n");
	for(i=0;i<100;i++)
	{
		m_nClientSock=SetSocket(m_nClientPort,(unsigned char *)m_cClientIP,m_ClientAddr);
		nCncRlst=connect(m_nClientSock,(struct sockaddr *)&m_HostAddr,sizeof(struct sockaddr));
		if(nCncRlst==0)
		{
			m_bStopListen=false;
			return 0;
			break;
		}
		else
		{
			nErr=WSAGetLastError();
			if (nErr==10048)
			{
				m_nClientPort+=1;
				m_nClientSock=SetSocket(m_nClientPort,(unsigned char *)m_cClientIP,m_ClientAddr);
			}
			
		}
		Sleep(1000);
	}
	return nRtn;
}


int ClientNet::RcvStream(int nGoalSize,char *pcTmpBuff,int sockfd)
{
	int nRecvCount,nLeftLen,nNumBytes;

	nLeftLen=nGoalSize;
	nRecvCount=0;
	while (nRecvCount<nGoalSize)
	{
		if ((nNumBytes=recv(sockfd, pcTmpBuff, nLeftLen, 0)) <=0) 
		{
			return -1;
		}
		nRecvCount+=nNumBytes;
		pcTmpBuff+=nNumBytes;
		nLeftLen=nGoalSize-nRecvCount;
	}
	return 0;
}



int ClientNet::SendStream(int nGoalSize,char *pcTmpBuff,int sockfd)
{
	int nRecvCount,nLeftLen,nNumBytes;

	nLeftLen=nGoalSize;
	nRecvCount=0;
	while (nRecvCount<nGoalSize)
	{
		if ((nNumBytes=send(sockfd, pcTmpBuff, nLeftLen, 0)) <=0) 
		{
			return -1;
		}
		nRecvCount+=nNumBytes;
		pcTmpBuff+=nNumBytes;
		nLeftLen=nGoalSize-nRecvCount;
	}
	return 0;
}

int ClientNet::GetNetStatus()
{
	return m_nNetStatus;
}


int ClientNet::ComRun(void *pcParams)
{
	int nRtn,nRecvDataLen;

	struct timeval timeout={0,200};
	bool bJump=false;

	while (!m_bStopListen)
	{
		if(Try2CncHost()==-1)
		{
			printf("cnc host  failed...\n");

			Sleep(5000);
			//m_bStopListen=true;
			//m_nNetStatus=-2;//Server shut down
		}
		else
		{

			bJump=false;
			m_nNetStatus=1;
			char cTmp[1000];
			SendData(m_cRegisterBuff,sizeof(RegisterInfo)+1);
			while (!bJump&&!m_bStopListen)
			{
				fd_set fdR; 
				FD_ZERO(&fdR);
				FD_SET(m_nClientSock,&fdR);
				switch (select(m_nClientSock + 1, &fdR, NULL, NULL , &timeout)) 
				{
				case -1:
					bJump=1;
					printf("sock value :  %d  \n",m_nClientSock);
					printf("ThreadCmd Loss Connection with Host!!!!!\n");
					break;
				case 0:   //no new data come
					break;
				default:
					nRtn=FD_ISSET(m_nClientSock,&fdR);
					if (nRtn)
					{
						if(RcvStream(4,(char*)&nRecvDataLen,m_nClientSock)!=-1)   
						{
							if(RcvStream(nRecvDataLen,cTmp,m_nClientSock)!=-1)
							{
								//Parse Cmd
								m_cbRecvData(cTmp,nRecvDataLen);
							}
							else
							{
								bJump=true;
							}
						}
						else
						{

							bJump=true;
						}
					}
				}

				Sleep(20);
			}
			closesocket(m_nClientSock);
		}
	
		m_nNetStatus=-3;//lost cnc with host
	}

	closesocket(m_nClientSock);
	if (!bJump)
	{
		return 0;
	}
	else return-1;
	
}



int ClientNet::SendData(char *pcData,int nDataLen)
{
	char cLen[4];
	if (m_nNetStatus!=1)
	{
		return -1;
	}
	memcpy(cLen,&nDataLen,4);
	if(SendStream(4,cLen,m_nClientSock)==0)
	{
		if (SendStream(nDataLen,pcData,m_nClientSock)==0)
		{
			return 0;
		}
		else 
		{
			m_nNetStatus=-3;
			return -1;
		}
	//	return SendStream(nDataLen,pcData,m_nClientSock);
	}
	else
	{
		m_nNetStatus=-3;
		return -1;
	}
}


int ClientNet::ComStop(void *pcParams)
{
	return 0;
}
int ClientNet::ComUninit(void *pcParams)
{
	return 0;
}

int ClientNet::ParseIP(char *pcData,char *pcIP)
{
	char cTmp[10];
	int i,j=0,k=0;
	for (i=0;i<(int)strlen(pcData);i++)
	{
		if (pcData[i]=='.')
		{
			pcIP[k++]=atoi(cTmp);
			j=0;
			memset(cTmp,0,10);

		}
		else
		{
			cTmp[j++]=pcData[i];
		}
	}
	pcIP[k]=atoi(cTmp);

	return 0;
}