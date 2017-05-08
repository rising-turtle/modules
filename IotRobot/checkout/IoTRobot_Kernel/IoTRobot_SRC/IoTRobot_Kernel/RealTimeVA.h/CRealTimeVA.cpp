#include "stdafx.h"
#include "CRealTimeVA.h"


/*--------------------------------------------
// 
--------------------------------------------*/
CRealTimeVA::CRealTimeVA()
{
	pAudioOperation = NULL;
	pVideoOperation = NULL;

	isWorkingNormal = false;
	isRunning		= false;
	isInputValid	= false;

}
/*--------------------------------------------
// 
--------------------------------------------*/
CRealTimeVA::~CRealTimeVA()
{

}
/*--------------------------------------------
// 
--------------------------------------------*/
int CRealTimeVA::RealTimeVAInit(IoTRobot_RTP_Param rtpParam,
								IoTRobot_RealTimeVA_VideoParam videoParam,
								IoTRobot_RealTimeVA_AudioParam audioParam)
{
	if (!(rtpParam.localAudioRtpPort
		  && rtpParam.localVideoPort
		  && rtpParam.remoteAudioRtpPort
		  && rtpParam.remoteIp
		  && rtpParam.remoteVideoPort))
	{
		goto exitEntry;
	}

	if (!(videoParam.frameRate
		&& videoParam.jpegQulity
		&& videoParam.videoHeight
		&& videoParam.videoWidth))
	{
		goto exitEntry;
	}

	if (!audioParam.hWnd)
	{
		goto exitEntry;
	}



	this->rtpParamVA = rtpParam;
	this->videoParamVA = videoParam;
	this->audioParamVA = audioParam;

	this->pAudioOperation	= new CAudioOperation();
	this->pVideoOperation	= new CVideoOperation();

	isInputValid = true;
	return 1;
exitEntry:
	isInputValid = false;
	return 0;
}
/*--------------------------------------------
// 
--------------------------------------------*/
int CRealTimeVA::RealTimeVARun()
{
	if (!isInputValid)
	{
		return 0;
	}

	if (isRunning)
	{
		return 1;
	}

	bool result;
	result = pAudioOperation->Open(	audioParamVA.hWnd, 
									rtpParamVA.remoteIp, 
									rtpParamVA.localAudioRtpPort, 
									rtpParamVA.remoteAudioRtpPort);
	if (!result)
	{
		goto exitEntry;
	}
	result = pVideoOperation->Open(	videoParamVA.doesUseDefaultCamera,
									videoParamVA.videoWidth,
									videoParamVA.videoHeight,
									rtpParamVA.remoteIp,
									rtpParamVA.localVideoPort,
									rtpParamVA.remoteVideoPort,
									videoParamVA.jpegQulity,
									videoParamVA.frameRate);
	isRunning = true;
	return 1;
exitEntry:
	isRunning = false;
	return 0;

}
/*--------------------------------------------
// 
--------------------------------------------*/
int CRealTimeVA::RealTimeVAStop()
{
	if (!isRunning)
	{
		return 1;
	}
	pAudioOperation->Close();
	pVideoOperation->Close();

	isRunning = false;
	return 1;
}
/*--------------------------------------------
// 
--------------------------------------------*/
int CRealTimeVA::RealTimeVAUninit()
{

	isWorkingNormal = false;
	isRunning		= false;
	isInputValid	= false;

	if (this->pAudioOperation)
	{
		delete this->pAudioOperation;
		this->pAudioOperation = NULL;
	}

	if (this->pVideoOperation)
	{
		delete this->pVideoOperation;
		this->pVideoOperation = NULL;
	}


	return 1;
}
/*--------------------------------------------
// 
--------------------------------------------*/
bool CRealTimeVA::GetRemoteVideoFrame(BYTE* pOutVideoBuffer, UINT* pSizeOfData)
{
	if (!pOutVideoBuffer || !pSizeOfData)
	{
		return false;
	}

	return this->pVideoOperation->GetRemoteVideoFrame(pOutVideoBuffer, pSizeOfData);

}
/*--------------------------------------------
// 
--------------------------------------------*/
bool CRealTimeVA::SendLocalVideoFrame(BYTE* pInVideoBuffer, UINT sizeOfData)
{
	if (!pInVideoBuffer)
	{
		return false;
	}
	return this->pVideoOperation->SendLocalVideoFrame(pInVideoBuffer, sizeOfData);

}
/*--------------------------------------------
// 
--------------------------------------------*/
void CRealTimeVA::SetLocalAudioPlayingStatus(bool doesAudioPlay)
{
	this->pAudioOperation->SetPlayLocalAudio(doesAudioPlay);
}
/*--------------------------------------------
// 
--------------------------------------------*/
bool CRealTimeVA::GetLocalLocalAudioPlayStatus()
{
	return this->pAudioOperation->GetPlayLocalAudioStatus();
}