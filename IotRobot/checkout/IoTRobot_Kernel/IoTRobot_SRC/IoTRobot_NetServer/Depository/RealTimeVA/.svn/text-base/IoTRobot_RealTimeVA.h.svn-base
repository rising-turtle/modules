#pragma once



#include "IoTRobot_RTP.h"
#include ".././Encode/IoTRobot_Encode_H263.h"
#include ".././Decode/IoTRobot_Decode_H263.h"
#include "IoTRobot_USBCamera.h"
#include "jpegClass.h"
//#pragma comment(lib, "AMRNB.lib")
//#include "../amrnb/interf_dec.h"
//#include "../amrnb/interf_enc.h"
//#include "../amrnb/interf_rom.h"
//#include "../amrnb/rom_dec.h"
//#include "../amrnb/rom_enc.h"
//#include "../amrnb/sp_dec.h"
//#include "../amrnb/sp_enc.h"
//#include "../amrnb/typedef.h"

#include "CDirectSoundRecorder.h"
#include "CDirectSoundPlayer.h"
#include "CVideoAudioBuffer.h"
#include "AudioGlobal.h"

extern "C"
{
#include "../amrnb/typedef.h"
#include "../amrnb/interf_enc.h"
#include "../amrnb/interf_dec.h"
}

#define  JPEG_COMPRESS
#ifdef JPEG_COMPRESS
#define VIDEO_WIDTH 320
#define VIDEO_HEIGHT 240
#define VIDEO_SIZE VIDEO_WIDTH*VIDEO_HEIGHT*3
#endif

typedef struct IoTRobot_RTPParam
{
	unsigned short usMyPort;
	unsigned char  *pucDestIPAddr;
	unsigned short usDestPort;
}IoTRobot_RTPParam;


typedef struct IoTRobot_RTPState
{
	HANDLE hThreadRecvRTV;
	HANDLE hThreadSendRTV;
	LPDWORD IDThreadRecvRTV;
	LPDWORD IDThreadSendRTV;

	bool bStopSendData;
	bool bStopRecvData;
}IoTRobot_RTPState;
class IoTRobot_RealTimeVA :public CAudioCaptureHandler
{
public:
	IoTRobot_RealTimeVA();
	~IoTRobot_RealTimeVA();


	IoTRobot_RTPState m_stRTPVideoStt;
	IoTRobot_RTPState m_stRTPAudioStt;


	int RealTimeVAInit(IoTRobot_RTPParam stRTPVideoParam,IoTRobot_RTPParam stRTPAudioParam,
		NetServerCallBack_Display cbDisplay,void *pContext,int nVideoQuality);
	int RealTimeVARun();
	int RealTimeVAStop();
	int RealTimeVAUninit();

//	CDirectSoundRecorder*	m_pCAudioCapture;
//	CDirectSoundPlayer*		m_pCAudioPlayer;
	//音频接收缓冲区
//	CVideoAudioBuffer*	m_pCAudioRcvBuffer;
	//音频发送缓冲区
//	CVideoAudioBuffer*	m_pCAudioSendBuffer;
	//视频接收缓冲区
//	CVideoAudioBuffer*	m_pCVedioRcvBuffer;


	IoTRobot_RTP m_CRTP_Video;
	IoTRobot_RTP m_CRTP_Audio;



	static void CallBack_RTPAudioData(unsigned char*pucRTPData,unsigned int uiDataLen,void *pContext);
	static void CallBack_RTPVideoData(unsigned char*pucRTPData,unsigned int uiDataLen,void *pContext);



	static UINT ThreadRecvRealTimeVideo(LPVOID lpParam);
	static UINT ThreadSendRealTimeVideo(LPVOID lpParam);
	static UINT ThreadRecvRealTimeAudio(LPVOID lpParam);
	static UINT ThreadSendRealTimeAudio(LPVOID lpParam);




/************************************************************************************************/
/****************************VIDEO PART**********************************************************/
/************************************************************************************************/
//Decode & Encode Method for Video
#ifdef H263_COMPRESS
	IoTRobot_Encode_H263 m_CH263Encode;
	static IoTRobot_Decode_H263 m_CH263Decode;
#endif

#ifdef JPEG_COMPRESS
	jpegClass m_CJPEG;
#endif

	static void CallBack_USBCamera(unsigned char *pucImgData,void *pContext);
	IoTRobot_USBCamera m_CUSBCam;

	unsigned char *m_pucEncodeBuff;
	unsigned char *m_pucVideoBuffA;
	unsigned char *m_pucVideoBuffB;
	void *m_pDisplayContext;
	NetServerCallBack_Display m_cbDisplay;
	int m_nVideoQuality;


/************************************************************************************************/
/****************************AUDIO PART**********************************************************/
/************************************************************************************************/
CDirectSoundRecorder m_CSoundRecorder;
void AudioCaptureCallback(BYTE* pBuffer, long lBufferSize); 
void*		m_EncoderState;
void*		m_DecoderState;
enum Mode	AMR_mode;
int			dtx;
short		speech[PCM_BLOCK_SIZE / 2];
protected:
private:
};