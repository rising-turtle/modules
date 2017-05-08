#include "IoTRobot_RealTimeVA.h"

IoTRobot_RealTimeVA::IoTRobot_RealTimeVA()
{
	m_pucEncodeBuff=new unsigned char[400000];
	m_pucVideoBuffA=new unsigned char[VIDEO_SIZE];
	m_pucVideoBuffB=new unsigned char[VIDEO_SIZE];
}

IoTRobot_RealTimeVA::~IoTRobot_RealTimeVA()
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


int IoTRobot_RealTimeVA::RealTimeVAInit(IoTRobot_RTPParam stRTPVideoParam,IoTRobot_RTPParam stRTPAudioParam,
										NetServerCallBack_Display cbDisplay,void *pContext,int nVideoQuality)
{
	BOOL bRslt;
//	m_pCAudioCapture	= new CDirectSoundRecorder();
//	m_pCAudioPlayer	= new CDirectSoundPlayer();



//	m_pCAudioRcvBuffer	= new CVideoAudioBuffer(/*buffer size = */AMR_BLOCK_SIZE * 6, /*max frame count = */150);
//	m_pCAudioSendBuffer	= new CVideoAudioBuffer(AMR_BLOCK_SIZE * 150, 150);
//	m_pCVedioRcvBuffer	= new CVideoAudioBuffer(/*buffer size = */1024 * 400, 1);



	m_CRTP_Video.RTPInit(stRTPVideoParam.usMyPort,
		                 stRTPVideoParam.pucDestIPAddr,
						 stRTPVideoParam.usDestPort,
						 0,CallBack_RTPVideoData,this);

	m_CRTP_Audio.RTPInit(stRTPAudioParam.usMyPort,
						 stRTPAudioParam.pucDestIPAddr,
						 stRTPAudioParam.usDestPort,
						 0,CallBack_RTPAudioData,this);


	m_CRTP_Audio.m_bStopRecvData=true;
	m_CRTP_Video.m_bStopRecvData=true;

	m_stRTPVideoStt.bStopRecvData=true;
	m_stRTPVideoStt.bStopSendData=true;
	m_stRTPVideoStt.IDThreadRecvRTV=0;
	m_stRTPVideoStt.IDThreadSendRTV=0;
	m_stRTPVideoStt.hThreadRecvRTV=0;
	m_stRTPVideoStt.hThreadSendRTV=0;



	m_stRTPAudioStt.bStopRecvData=true;
	m_stRTPAudioStt.bStopSendData=true;
	m_stRTPAudioStt.IDThreadRecvRTV=0;
	m_stRTPAudioStt.IDThreadSendRTV=0;
	m_stRTPAudioStt.hThreadRecvRTV=0;
	m_stRTPAudioStt.hThreadSendRTV=0;


	m_EncoderState = Encoder_Interface_init(dtx);
	m_DecoderState = Decoder_Interface_init();


	m_CUSBCam.USBCameraInit(0,CallBack_USBCamera,this);
	bRslt=m_CSoundRecorder.Open();

	m_nVideoQuality=nVideoQuality;
	m_cbDisplay=cbDisplay;
	m_pDisplayContext=pContext;
	return 0;
}

int IoTRobot_RealTimeVA::RealTimeVARun()
{
	m_CUSBCam.USBCameraRun();
	m_CSoundRecorder.BeginCapture(true,this);
	
	m_stRTPVideoStt.bStopRecvData=false;
	m_stRTPVideoStt.bStopSendData=false;

	m_stRTPAudioStt.bStopRecvData=false;
	m_stRTPAudioStt.bStopSendData=false;

	m_CRTP_Audio.m_bStopRecvData=false;
	m_CRTP_Video.m_bStopRecvData=false;


	m_stRTPVideoStt.hThreadRecvRTV=
		CreateThread(NULL,0,(LPTHREAD_START_ROUTINE)ThreadRecvRealTimeVideo,this,0,m_stRTPVideoStt.IDThreadRecvRTV);
	m_stRTPVideoStt.hThreadSendRTV=
		CreateThread(NULL,0,(LPTHREAD_START_ROUTINE)ThreadSendRealTimeVideo,this,0,m_stRTPVideoStt.IDThreadSendRTV);



	m_stRTPAudioStt.hThreadRecvRTV=
		CreateThread(NULL,0,(LPTHREAD_START_ROUTINE)ThreadRecvRealTimeAudio,this,0,m_stRTPAudioStt.IDThreadRecvRTV);
	m_stRTPAudioStt.hThreadSendRTV=
		CreateThread(NULL,0,(LPTHREAD_START_ROUTINE)ThreadSendRealTimeAudio,this,0,m_stRTPAudioStt.IDThreadSendRTV);
	return 0;
}

int IoTRobot_RealTimeVA::RealTimeVAStop()
{
	m_CUSBCam.USBCameraStop();

	m_stRTPAudioStt.bStopRecvData=true;
	m_stRTPAudioStt.bStopSendData=true;
	m_stRTPVideoStt.bStopRecvData=true;
	m_stRTPVideoStt.bStopSendData=true;
	m_CRTP_Audio.m_bStopRecvData=true;
	m_CRTP_Video.m_bStopRecvData=true;



	WaitForSingleObject(ThreadRecvRealTimeVideo,INFINITE);
	WaitForSingleObject(ThreadSendRealTimeVideo,INFINITE);
	WaitForSingleObject(ThreadRecvRealTimeAudio,INFINITE);
	WaitForSingleObject(ThreadSendRealTimeAudio,INFINITE);
	return 0;
}

int IoTRobot_RealTimeVA::RealTimeVAUninit()
{
	m_CUSBCam.USBCameraUninit();
	m_CRTP_Video.RTPUninit();

	Decoder_Interface_exit(m_DecoderState);
	Encoder_Interface_exit(m_EncoderState);
	return 0;
}


void IoTRobot_RealTimeVA::CallBack_RTPAudioData(unsigned char*pucRTPData,unsigned int uiDataLen,void *pContext)
{

}

void IoTRobot_RealTimeVA::CallBack_RTPVideoData(unsigned char*pucRTPData,unsigned int uiDataLen,void *pContext)
{
	IoTRobot_RealTimeVA *pRTVA=(IoTRobot_RealTimeVA *)pContext;
	int nDecodeBuffLen=352*288*3;

#ifdef H263_COMPRESS

	pRTVA->m_CH263Decode.IoTRobot_DecodeOneFrame(pucRTPData,uiDataLen,pRTVA->m_pucVideoBuffB,nDecodeBuffLen);
	pRTVA->m_cbDisplay(pRTVA->m_pucVideoBuffB,m_pDisplayContext);
#endif


	//这里可以优化,直接用jpeg刷屏，以后完善
#ifdef JPEG_COMPRESS
	pRTVA->m_CJPEG.jpgToRGB(pucRTPData,uiDataLen,pRTVA->m_pucVideoBuffB,(unsigned int*)&nDecodeBuffLen);
	pRTVA->m_cbDisplay(pRTVA->m_pucVideoBuffB,pRTVA->m_pDisplayContext);
#endif

}


UINT IoTRobot_RealTimeVA::ThreadRecvRealTimeVideo(LPVOID lpParam)
{
	IoTRobot_RealTimeVA *pRTVA=(IoTRobot_RealTimeVA*)lpParam;
	pRTVA->m_CRTP_Video.RTPRecvData();
	return 0;
}


UINT IoTRobot_RealTimeVA::ThreadSendRealTimeVideo(LPVOID lpParam)
{
	unsigned char ucLook[200];
	int nRslt,nNewFd,nEncodeBuffLen;

	IoTRobot_RealTimeVA *pRTVA=(IoTRobot_RealTimeVA*)lpParam;

	while (!pRTVA->m_stRTPVideoStt.bStopSendData)
	{
#ifdef H263_COMPRESS
		pRTVA->m_CH263Encode.IoTRobot_EncodeOneFrame(pRTVA->m_pucVideoBuffA,
			pRTVA->m_pucEncodeBuff,&nEncodeBuffLen,VGA_RGB24);
#endif

#ifdef JPEG_COMPRESS
		pRTVA->m_CJPEG.RGBToJpg(VIDEO_WIDTH,VIDEO_HEIGHT,pRTVA->m_pucVideoBuffA,
			&pRTVA->m_pucEncodeBuff,(unsigned long *)&nEncodeBuffLen,pRTVA->m_nVideoQuality);
#endif
		pRTVA->m_CRTP_Video.RTPSendData(pRTVA->m_pucEncodeBuff,nEncodeBuffLen);
		Sleep(20);
	}
	return 0;
}




UINT IoTRobot_RealTimeVA::ThreadRecvRealTimeAudio(LPVOID lpParam)
{
	IoTRobot_RealTimeVA *pRTVA=(IoTRobot_RealTimeVA*)lpParam;
	pRTVA->m_CRTP_Audio.RTPRecvData();
	return 0;
}
UINT IoTRobot_RealTimeVA::ThreadSendRealTimeAudio(LPVOID lpParam)
{
	return 0;
}



void IoTRobot_RealTimeVA::CallBack_USBCamera(unsigned char *pucImgData,void *pContext)
{
	int i;
	IoTRobot_RealTimeVA *pRTVA=(IoTRobot_RealTimeVA *)pContext;

	for (i=0;i<VIDEO_SIZE;i=i+3)
	{
		pRTVA->m_pucVideoBuffA[i]=pucImgData[i+2];
		pRTVA->m_pucVideoBuffA[i+1]=pucImgData[i+1];
		pRTVA->m_pucVideoBuffA[i+2]=pucImgData[i];
	}
}

void IoTRobot_RealTimeVA::AudioCaptureCallback(BYTE* pBuffer, long lBufferSize)
{

	int		byteSize;
	BYTE	amrEncoded[AMR_BLOCK_SIZE];
	UINT32	timestamp = GetTickCount();

	int dataBlockCount = lBufferSize / PCM_BLOCK_SIZE;//算出有多少块

	for(int blockIndex = 0; blockIndex < dataBlockCount; blockIndex++)
	{
		//将每块PCM压缩
		byteSize = Encoder_Interface_Encode(m_EncoderState, 
			AMR_mode, 
			(short*)(pBuffer + PCM_BLOCK_SIZE * blockIndex),
			amrEncoded,
			0);
		m_CRTP_Audio.RTPSendData(amrEncoded,byteSize);
		//pAudioSendBuffer->putOneImageIntoBuffer(amrEncoded, AMR_BLOCK_SIZE, timestamp);
	}


}