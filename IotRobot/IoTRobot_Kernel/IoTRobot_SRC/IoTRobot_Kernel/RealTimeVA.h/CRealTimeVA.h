#ifndef _CREALTIMEVA_H_
#define _CREALTIMEVA_H_

#include "CAudioOperation.h"
#include "CVideoOperation.h"
#define _CRTDBG_MAP_ALLOC 
#include<stdlib.h> 
#include<crtdbg.h> 

struct IoTRobot_RTP_Param
{
	BYTE	remoteIp[4];
	USHORT	localAudioRtpPort;
	USHORT	remoteAudioRtpPort;
	USHORT	localVideoPort;
	USHORT	remoteVideoPort;
};

struct IoTRobot_RealTimeVA_VideoParam
{
	bool	doesUseDefaultCamera;
	int		videoWidth;
	int		videoHeight;
	int		jpegQulity;
	int		frameRate;
};

struct IoTRobot_RealTimeVA_AudioParam
{
	HWND hWnd;
};



class CRealTimeVA
{

public:
	// Methods

	CRealTimeVA();
	~CRealTimeVA();

	int RealTimeVAInit(	IoTRobot_RTP_Param rtpParam,
						IoTRobot_RealTimeVA_VideoParam videoParam,
						IoTRobot_RealTimeVA_AudioParam audioParam);

	int RealTimeVARun();

	int RealTimeVAStop();

	int RealTimeVAUninit();

	bool GetRemoteVideoFrame(BYTE* pOutVideoBuffer, UINT* sizeOfData);
	bool SendLocalVideoFrame(BYTE* pInVideoBuffer, UINT sizeOfData);
	void SetLocalAudioPlayingStatus(bool isAudioPlaying);
	bool GetLocalLocalAudioPlayStatus();

	//Variables


private:
	// Methods

	CAudioOperation* pAudioOperation;
	CVideoOperation* pVideoOperation;

	IoTRobot_RTP_Param rtpParamVA;
	IoTRobot_RealTimeVA_VideoParam videoParamVA;
	IoTRobot_RealTimeVA_AudioParam audioParamVA;

	//Variables

	bool isWorkingNormal;
	bool isRunning;
	bool isInputValid;
};


#endif //#ifndef _CREALTIMEVA_H_