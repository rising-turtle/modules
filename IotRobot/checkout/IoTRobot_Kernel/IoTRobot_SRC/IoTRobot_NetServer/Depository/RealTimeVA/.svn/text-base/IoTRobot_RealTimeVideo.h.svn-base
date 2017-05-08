
#pragma once
#include "IoTRobot_RTP.h"
#include ".././Encode/IoTRobot_Encode_H263.h"
#include ".././Decode/IoTRobot_Decode_H263.h"
#include "IoTRobot_USBCamera.h"
#include "jpegClass.h"


#ifdef JPEG_COMPRESS
#define VIDEO_WIDTH 320
#define VIDEO_HEIGHT 240
#define VIDEO_SIZE VIDEO_WIDTH*VIDEO_HEIGHT*3
#endif


#ifdef H263_COMPRESS
#define VIDEO_WIDTH 352
#define VIDEO_HEIGHT 288
#define VIDEO_SIZE VIDEO_WIDTH*VIDEO_HEIGHT*3
#endif
class IoTRobot_RealTimeVideo
{
public:
	IoTRobot_RealTimeVideo();
	~IoTRobot_RealTimeVideo();

	int RealTimeVideoInit(unsigned short usMyPort,unsigned char  *pucDestIPAddr,unsigned short usDestPort,
		int nType,NetServerCallBack_Display cbDisplay,void *pContext,int nVideoQuality);
	int RealTimeVideoRun();
	int RealTimeVideoStop();
	int RealTimeVideoUninit();


	IoTRobot_RTP m_CRTP;
	IoTRobot_USBCamera m_CUSBCam;

#ifdef H263_COMPRESS
	IoTRobot_Encode_H263 m_CH263Encode;
	static IoTRobot_Decode_H263 m_CH263Decode;
#endif

#ifdef JPEG_COMPRESS
	jpegClass m_CJPEG;
#endif

	void *m_pDisplayContext;
	NetServerCallBack_Display m_cbDisplay;
	int m_nVideoQuality;
	bool m_bStopSendRealTimeVideo;
	bool m_bStopRecvRealTimeVideo;

	static void CallBack_RTPData(unsigned char*pucRTPData,unsigned int uiDataLen,void *pContext);
	
	static void CallBack_USBCamera(unsigned char *pucImgData,void *pContext);
	int m_nIMUState;

	HANDLE m_hThreadRecvRTV;
	HANDLE m_hThreadSendRTV;
	LPDWORD m_IDThreadRecvRTV;
	LPDWORD m_IDThreadSendRTV;

	static UINT ThreadRecvRealTimeVideo(LPVOID lpParam);
	static UINT ThreadSendRealTimeVideo(LPVOID lpParam);

	unsigned char *m_pucEncodeBuff;
	unsigned char *m_pucVideoBuffA;
	unsigned char *m_pucVideoBuffB;
protected:
private:
};