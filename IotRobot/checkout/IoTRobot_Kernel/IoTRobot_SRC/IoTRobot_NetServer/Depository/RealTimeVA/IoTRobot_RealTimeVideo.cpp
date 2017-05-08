#include "IoTRobot_RealTimeVideo.h"

IoTRobot_RealTimeVideo::IoTRobot_RealTimeVideo()
{
	m_pucEncodeBuff=new unsigned char[400000];
	m_pucVideoBuffA=new unsigned char[VIDEO_SIZE];
	m_pucVideoBuffB=new unsigned char[VIDEO_SIZE];
}

IoTRobot_RealTimeVideo::~IoTRobot_RealTimeVideo()
{
	if (m_pucEncodeBuff!=NULL)
	{
		delete [] m_pucEncodeBuff;
		m_pucEncodeBuff=NULL;
	}

	if (m_pucVideoBuffA!=NULL)
	{
		delete [] m_pucVideoBuffA;
		m_pucVideoBuffA=NULL;
	}

	if (m_pucVideoBuffB!=NULL)
	{
		delete [] m_pucVideoBuffB;
		m_pucVideoBuffB=NULL;
	}
}


int IoTRobot_RealTimeVideo::RealTimeVideoInit(unsigned short usMyPort,unsigned char  *pucDestIPAddr,unsigned short usDestPort,
											  int nType,NetServerCallBack_Display cbDisplay,void *pContext,int nVideoQuality)
{
	m_CRTP.RTPInit(usMyPort,pucDestIPAddr,usDestPort,nType,CallBack_RTPData,this);
	m_CUSBCam.USBCameraInit(0,CallBack_USBCamera,this);

	m_hThreadRecvRTV=0;
	m_hThreadSendRTV=0;

	m_bStopSendRealTimeVideo=true;
	m_bStopRecvRealTimeVideo=true;
	m_CRTP.m_bStopRecvData=true;

	m_nVideoQuality=nVideoQuality;
	m_cbDisplay=cbDisplay;
	m_pDisplayContext=pContext;

	return 0;
}


int IoTRobot_RealTimeVideo::RealTimeVideoRun()
{
	m_CUSBCam.USBCameraRun();
	m_IDThreadRecvRTV=0;
	m_IDThreadSendRTV=0;
	m_bStopSendRealTimeVideo=false;
	m_bStopRecvRealTimeVideo=false;

	m_CRTP.m_bStopRecvData=false;
	m_hThreadRecvRTV=CreateThread(NULL,0,(LPTHREAD_START_ROUTINE)ThreadRecvRealTimeVideo,this,0,m_IDThreadRecvRTV);
	m_hThreadSendRTV=CreateThread(NULL,0,(LPTHREAD_START_ROUTINE)ThreadSendRealTimeVideo,this,0,m_IDThreadSendRTV);
	return 0;
}



int IoTRobot_RealTimeVideo::RealTimeVideoStop()
{
	m_CUSBCam.USBCameraStop();
	m_bStopSendRealTimeVideo=true;
	m_bStopRecvRealTimeVideo=true;
	m_CRTP.m_bStopRecvData=true;
	WaitForSingleObject(ThreadRecvRealTimeVideo,INFINITE);
	WaitForSingleObject(ThreadSendRealTimeVideo,INFINITE);
	return 0;
}


int IoTRobot_RealTimeVideo::RealTimeVideoUninit()
{
	m_CUSBCam.USBCameraUninit();
	m_CRTP.RTPUninit();
	m_CUSBCam.USBCameraUninit();

	return 0;
}


void IoTRobot_RealTimeVideo::CallBack_USBCamera(unsigned char *pucImgData,void *pContext)
{
	IoTRobot_RealTimeVideo *pRTV=(IoTRobot_RealTimeVideo *)pContext;
	memcpy(pRTV->m_pucVideoBuffA,pucImgData,VIDEO_SIZE);
}

UINT IoTRobot_RealTimeVideo::ThreadSendRealTimeVideo(LPVOID lpParam)
{
	//DWORD start=0,end=0;
	unsigned char ucLook[200];
	int nRslt,nNewFd,nEncodeBuffLen;

	IoTRobot_RealTimeVideo *pRTV=(IoTRobot_RealTimeVideo *)lpParam;

	while (!pRTV->m_bStopSendRealTimeVideo)
	{
#ifdef H263_COMPRESS
		pRTV->m_CH263Encode.IoTRobot_EncodeOneFrame(pRTV->m_pucVideoBuffA,
			pRTV->m_pucEncodeBuff,&nEncodeBuffLen,VGA_RGB24);
#endif

#ifdef JPEG_COMPRESS
		pRTV->m_CJPEG.RGBToJpg(VIDEO_WIDTH,VIDEO_HEIGHT,pRTV->m_pucVideoBuffA,
			&pRTV->m_pucEncodeBuff,(unsigned long *)&nEncodeBuffLen,pRTV->m_nVideoQuality);
#endif
		pRTV->m_CRTP.RTPSendData(pRTV->m_pucEncodeBuff,nEncodeBuffLen);
		Sleep(20);
	}
	return 0;
}



void IoTRobot_RealTimeVideo::CallBack_RTPData(unsigned char*pucRTPData,unsigned int uiDataLen,void *pContext)
{
	IoTRobot_RealTimeVideo *pRTV=(IoTRobot_RealTimeVideo *)pContext;
	int nDecodeBuffLen=352*288*3;

#ifdef H263_COMPRESS

	pRTV->m_CH263Decode.IoTRobot_DecodeOneFrame(pucRTPData,uiDataLen,pRTV->m_pucVideoBuffB,nDecodeBuffLen);
	pRTV->m_cbDisplay(pRTV->m_pucVideoBuffB,m_pDisplayContext);
#endif


	//这里可以优化,直接用jpeg刷屏，以后完善
#ifdef JPEG_COMPRESS
	pRTV->m_CJPEG.jpgToRGB(pucRTPData,uiDataLen,pRTV->m_pucVideoBuffB,(unsigned int*)&nDecodeBuffLen);
	pRTV->m_cbDisplay(pRTV->m_pucVideoBuffB,pRTV->m_pDisplayContext);
#endif

}



UINT IoTRobot_RealTimeVideo::ThreadRecvRealTimeVideo(LPVOID lpParam)
{
	int nRslt,nNewFd;
	IoTRobot_RealTimeVideo *pRTV=(IoTRobot_RealTimeVideo*)lpParam;
	pRTV->m_CRTP.RTPRecvData();
	return 0;
}