#include "IoTRobot_RTP.h"


IoTRobot_RTP::IoTRobot_RTP()
{

}

IoTRobot_RTP::~IoTRobot_RTP()
{

}

int IoTRobot_RTP::RTPInit(unsigned short usMyPort,unsigned char *pucDestIPAddr,unsigned short usDestPort, 
						  int nType,NetServerCallBack_RecvRTPData cbRecvRTPData,void *pContext)
{
//	printf("B1!!!!!\n");
	WSADATA dat;
	WSAStartup(MAKEWORD(2,2), &dat);

//	printf("B2!!!!!\n");
	m_CRTPSessionparams.SetOwnTimestampUnit(1.0/90000.0);	//for video ---see RFC2190

	RTPUDPv4TransmissionParams transparams;
	transparams.SetPortbase(usMyPort);
//	printf("B3!!!!!\n");
	if(m_CRTPSession.Create(m_CRTPSessionparams,&transparams)<0)
	{
		return -1;
	}
//	printf("B4!!!!!\n");
	RTPIPv4Address addr(pucDestIPAddr, usDestPort);
	if(m_CRTPSession.AddDestination(addr)<0)
	{
		return -2;
	}
	m_CRTPSession.SetDefaultPayloadType(34);	//PT for H.263----see RFC2190
	m_CRTPSession.SetDefaultMark(true);		//true or false ?? both ok for my test	
	m_CRTPSession.SetDefaultTimestampIncrement(3600);	// =90000/25
	m_CRTPSession.SetMaximumPacketSize(RTP_MAX_PACK_SIZE);


	//RTPSession session_rec;
	RTPSessionParams sessionparams_rec;
	sessionparams_rec.SetOwnTimestampUnit(1.0/90000.0);	//for video ---see RFC2190
	RTPUDPv4TransmissionParams transparams_rec;
	transparams_rec.SetPortbase(10006);
	if(m_CRTPSession2.Create(sessionparams_rec,&transparams_rec)<0)
	{
		return -1;
	}

	RTPIPv4Address addr2(pucDestIPAddr, usDestPort);
	m_CRTPSession2.AddDestination(addr2);

	m_CRTPSession2.SetDefaultPayloadType(34);	//PT for H.263----see RFC2190
	m_CRTPSession2.SetDefaultMark(true);		//true or false ?? both ok for my test	
	m_CRTPSession2.SetDefaultTimestampIncrement(3600);	// =90000/25
	m_CRTPSession2.SetMaximumPacketSize(RTP_MAX_PACK_SIZE);

	WSACleanup(); 

	m_bStopRecvData=false;
	m_cbRecvRTPData=cbRecvRTPData;
	m_pContext=pContext;
	return 0;
}


int IoTRobot_RTP::RTPSendData(unsigned char *pucRTPSendData,int nSendDataLen)
{
	if (nSendDataLen < RTP_MAX_PACK_SIZE)
	{
		m_CRTPSession2.SendPacket(pucRTPSendData, nSendDataLen);
	}
	return 0;
}
int IoTRobot_RTP::RTPUninit()
{
	return 0;
}
int IoTRobot_RTP::RTPRecvData()
{
	unsigned int uiDataLen=0;
	unsigned char *pucRecvData;
//	unsigned char look[100];
	DWORD end=0,start=0;
	while (!m_bStopRecvData)
	{
		m_CRTPSession.BeginDataAccess();
		if (m_CRTPSession.GotoFirstSourceWithData())
		{
			do 
			{
				//printf("wait time:  %d  \n",GetTickCount()-end);
				RTPPacket *packet;
				start=GetTickCount();
				while( (packet = m_CRTPSession.GetNextPacket()) != NULL)
				{
					uiDataLen = packet->GetPayloadLength();
					pucRecvData = packet->GetPayloadData();
					memcpy(m_cRTPDataBuff,pucRecvData,uiDataLen);
				//	memcpy(look,pucRecvData,100);
					m_cbRecvRTPData(m_cRTPDataBuff,uiDataLen,m_pContext);
					m_CRTPSession.DeletePacket(packet);

				}
			//	printf("recv time:  %d  \n",GetTickCount()-start);
				end=GetTickCount();
			//	Sleep(10);
			} while (m_CRTPSession.GotoNextSourceWithData());

		}
		m_CRTPSession.EndDataAccess();
		Sleep(1);
	}
	return 0;
}