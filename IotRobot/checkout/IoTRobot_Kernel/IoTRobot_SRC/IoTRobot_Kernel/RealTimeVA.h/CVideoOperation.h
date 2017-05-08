#ifndef _CVIDEOOPERRATION_H_
#define _CVIDEOOPERRATION_H_

#ifndef SAFE_DELETE
	#define SAFE_DELETE(p) { if(p) { delete (p); (p)=NULL; } }
#endif

#include "rtpTransfer.h"	//this header file must be put on the top of other headers,
							//or some conflict will happen.
#include "CVideoAudioBuffer.h"
#include "jpegClass.h"
#include "CameraDS.h"		//define operations on camera


class CVideoOperation
{
public:	//methods
	CVideoOperation();
	~CVideoOperation();

	bool Open(	bool doesUseCamera,
				int videoWidth,
				int videoHeight,
				BYTE* remoteIpAdd,
				USHORT localVideoPort,
				USHORT remoteVideoPort,
				int jpegQulity,
				int frameRate);

	bool GetRemoteVideoFrame(BYTE* pOutVideoBuffer, UINT* sizeOfData);
	bool SendLocalVideoFrame(BYTE* pInVideoBuffer, UINT sizeOfData);
	bool Close();

public:	//variables
	//If work wrong, this indicate what's wrong happened
	//0: unknow;
	//1: rtp err; 
	//2: directShow err 
	int  errorCode;	

	bool isWorkingNormal;



private: //methods
	bool Initialization();
	bool Release();

	static DWORD WINAPI Thread_RTP_VideoTransfer(LPVOID pvoid);	//Thread for sending and receiving video data
																//to and from remote

private: //variables
	bool isVideoRTPThreadCanQuit;
	bool doUseDefaultCamera;
	int width;
	int height;
	int jpegQulity;
	int frameRate;
	UINT32 tickInterval;

	//------------- Audio capture and play -----------------
	bool isVideoOperationRunning;
	UINT32 videoTimeStamp;

	CCameraDS*	pCamera;
	jpegClass*	pJpegDealer;
	BYTE		remoteIp[4];
	USHORT		localVideoRtpPort;
	USHORT		remoteVideoRtpPort;

	//Threads' handles
	HANDLE hTrdVideoRTP;

	CVideoAudioBuffer*	pVideoRcvBuffer;	//video buffer for receiving
	CVideoAudioBuffer*	pVideoSndBuffer;	//video buffer for sending

};


#endif //#ifndef _CVIDEOOPERRATION_H_