//#include "stdafx.h"
#include "IDrive.h"

CIDrive::CIDrive()
{

}


CIDrive::~CIDrive()
{

}

int CIDrive::IDriveInit()
{
	m_stRealTimeVA_VideoParam.doesUseDefaultCamera=true;
	m_stRealTimeVA_VideoParam.frameRate=30;
	m_stRealTimeVA_VideoParam.jpegQulity=20;
	m_stRealTimeVA_VideoParam.videoHeight=240;
	m_stRealTimeVA_VideoParam.videoWidth=320;

	m_nStopRealTimeVA=true;
	m_hThreadRTVA=0;
	m_nRunIDrive=0;
	m_nRealTimeHasInit=0;
	m_nRobotCnc=0;
	return 0;
}
void CIDrive::NewCommand(const IoTRobot_Message MSG)
{
	switch (MSG.cCommand)
	{
	case IDRIVE_MSG_CTRL:
		//AddMsg();
		break;
	case IDRIVE_IP_INFO:

		int nTmp;
		memcpy(&m_stRTPParam.remoteIp,MSG.pcParam3,4);
		memcpy(&nTmp,MSG.pcParam3+4,4);
		m_stRTPParam.localVideoPort=nTmp;
		memcpy(&nTmp,MSG.pcParam3+8,4);
		m_stRTPParam.localAudioRtpPort=nTmp;
		memcpy(&nTmp,MSG.pcParam3+12,4);
		m_stRTPParam.remoteVideoPort=nTmp;
		memcpy(&nTmp,MSG.pcParam3+16,4);
		m_stRTPParam.remoteAudioRtpPort=nTmp;

		m_nRobotCnc=1;
		//AddMsg();
		break;
	case IDRIVE_REALTIMEVA:
		if (m_nRunIDrive&&m_nRealTimeHasInit)
		{
			if(MSG.nParam1==REALTIME_RUN)
			{	
				LPDWORD ID=0;
				m_nStopRealTimeVA=false;
				m_hThreadRTVA=CreateThread(NULL,0,(LPTHREAD_START_ROUTINE)ThreadRealTimeVA,this,0,ID);
				m_CRealTimeVA.SetLocalAudioPlayingStatus(true);

				printf("Open Audio!!! \n");

			}
			else if(MSG.nParam1==REALTIME_STOP)
			{
				m_nStopRealTimeVA=true;
				m_CRealTimeVA.SetLocalAudioPlayingStatus(false);
				printf("Close Audio!!! \n");
				if (m_hThreadRTVA!=0)
				{
					WaitForSingleObject(m_hThreadRTVA,INFINITE);
					m_hThreadRTVA=false;
				}
			//	m_CRealTimeVA.RealTimeVAStop();
			}
		}

	break;
	case IDRIVE_SWITCH:
		if (MSG.nParam1==1)
		{
			if (!m_nRunIDrive)
			{
				m_nRunIDrive=1;
				//if (!m_nRealTimeHasInit)
				{
					if (m_nRobotCnc)
					{
						m_stRealTimeVA_AudioParam.hWnd=(HWND)MSG.pcParam3;
						m_CRealTimeVA.RealTimeVAInit(m_stRTPParam,m_stRealTimeVA_VideoParam,m_stRealTimeVA_AudioParam);
						m_nRealTimeHasInit=1;
					}
					else
					{
						break;
					}
				}
				m_CRealTimeVA.SetLocalAudioPlayingStatus(false);
				m_CRealTimeVA.RealTimeVARun();
			}
		}
		else if(MSG.nParam1==0)
		{
			m_nRunIDrive=0;
			m_nStopRealTimeVA=true;
			if (m_hThreadRTVA!=0)
			{
				WaitForSingleObject(m_hThreadRTVA,INFINITE);
				m_hThreadRTVA=0;
			}
			m_CRealTimeVA.RealTimeVAStop();
			m_CRealTimeVA.RealTimeVAUninit();
		}
	default:
		break;
}
}
void CIDrive::CreateMsgArray()
{
	int i;
	memset(&m_stMsgArray,0,sizeof(CtrlDrv_MSGArray));

	for (i=0;i<MSGARRAY_LEN-1;i++)
	{
		m_stMsgArray.stMsgArray[i].pstNext=&m_stMsgArray.stMsgArray[i+1];
	}
	m_stMsgArray.stMsgArray[MSGARRAY_LEN-1].pstNext=&m_stMsgArray.stMsgArray[0];

	m_stMsgArray.pstWritePos=&m_stMsgArray.stMsgArray[0];
	m_stMsgArray.pstReadPos=&m_stMsgArray.stMsgArray[0];
}

UINT CIDrive::ThreadRealTimeVA(LPVOID lpParam)
{
	CIDrive *pIDrive=(CIDrive *)lpParam;
	unsigned char *pImg=new unsigned char[320*240*3];
	UINT nLen=320*240*3;
	while (!pIDrive->m_nStopRealTimeVA)
	{
		pIDrive->m_CRealTimeVA.GetRemoteVideoFrame(pImg,&nLen);
		pIDrive->m_cbPIPQVGA(pImg,NULL);
		Sleep(30);
	}
	delete [] pImg;
	return 0;
}

int CIDrive::AddMsg(char *pcContent)
{
	if(m_stMsgArray.pstWritePos->cStateFlag!=1)
	{
		m_stMsgArray.pstWritePos->cContent=(*pcContent);
		m_stMsgArray.pstWritePos->cStateFlag=1;
		m_stMsgArray.pstWritePos=m_stMsgArray.pstWritePos->pstNext;
		return 1;
	}
	else return 0;
}




int CIDrive::DrawMsg(char *pcContent)
{
	if (m_stMsgArray.pstReadPos->pstNext!=0)
	{
		*pcContent=m_stMsgArray.pstReadPos->cStateFlag;
		m_stMsgArray.pstReadPos->cStateFlag=0;
		m_stMsgArray.pstReadPos=m_stMsgArray.pstReadPos->pstNext;
		return 1;
	}
	else return 0;
}


int  CIDrive::SendCtrlCmd2Robot()
{
	while(1)
	{

	}
}



