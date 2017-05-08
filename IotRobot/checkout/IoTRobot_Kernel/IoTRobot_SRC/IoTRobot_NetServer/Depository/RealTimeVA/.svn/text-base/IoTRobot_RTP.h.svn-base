#pragma once
#include "../IoTRobot_NetServer_Define.h"
#include ".././RTP/rtpheader.h"
using namespace jrtplib;
class IoTRobot_RTP
{
public:
	IoTRobot_RTP();
	~IoTRobot_RTP();

	int RTPInit(unsigned short usMyPort,unsigned char  *pucDestIPAddr,unsigned short usDestPort,
	int nType,NetServerCallBack_RecvRTPData cbRecvRTPData,void *pContext);
	int RTPRun();
	int RTPUninit();


	int RTPRecvData();
	int RTPSendData(unsigned char *pucRTPSendData,int nSendDataLen);
	int RTPControl();

	RTPSession m_CRTPSession;
	RTPSession m_CRTPSession2;
	RTPSessionParams m_CRTPSessionparams;

	NetServerCallBack_RecvRTPData m_cbRecvRTPData;
	bool m_bStopRecvData;

	unsigned char m_cRTPDataBuff[RTP_MAX_PACK_SIZE];
	void *m_pContext;
protected:
private:
};