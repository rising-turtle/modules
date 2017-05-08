#ifndef _CAUDIOOPERRATION_H_
#define _CAUDIOOPERRATION_H_

#ifndef SAFE_DELETE
	#define SAFE_DELETE(p) { if(p) { delete (p); (p)=NULL; } }
#endif

#define MAX_RTP_POCKET_SIZE		20000	//this value used by RTP self-defined class

//audio processing
#include "rtpTransfer.h"	//this header file must be put on the top of other headers,
							//or some conflict will happen.
#include "CDirectSoundRecorder.h"
#include "CDirectSoundPlayer.h"
#include "CVideoAudioBuffer.h"
#include "AudioGlobal.h"

//AMR
extern "C"
{
	#include "AMRNB/typedef.h"
	#include "AMRNB/interf_enc.h"
	#include "AMRNB/interf_dec.h"
}


class CAudioOperation: public CAudioCaptureHandler, public CAudioPlayerHandler
{
public:	//methods
	CAudioOperation();
	~CAudioOperation();

	bool Open(HWND hWnd, BYTE* remoteIp, USHORT localAudioRtpPort, USHORT remoteAudioRtpPort);
	bool Close();
	void SetPlayLocalAudio(bool status);
	bool GetPlayLocalAudioStatus();

public:	//variables

	bool isWorkingNormal;	//Represent whether this class work well

	//If work wrong, this indicate what's wrong happened
	//0: narmal or unknown;
	//1: rtp err; 
	//2: direct sound err 
	int  errorCode;			

private: //methods

	// overload class CAudioCaptureHandler,
	// then this callback routine will be called
	// repeatedly when direct sound capture one 
	// block of PCM data
	void AudioCaptureCallback(unsigned char* pBuffer, long lBufferSize);	

	// overload class CAudioPlayerHandler,
	// then this callback routine will be called
	// repeatedly when direct sound want one 
	// block of PCM data
	void AudioPlayerCallback(unsigned char* pBuffer, long lBufferSize);	

	bool Initialization();
	bool Release();

	//Thread for sending and receiving audio data
	//to and from remote
	static DWORD WINAPI Thread_RTP_AudioTransfer(LPVOID pvoid);	


private: //variables
	//----------- RTP -----------
	BOOL	isAudioRTPThreadCanQuit;
	BYTE	remoteIp[4];
	USHORT	localAudioRtpPort;
	USHORT	remoteAudioRtpPort;

	//------------- AMR -----------------
	void*		encoderState;
	void*		decoderState;
	enum Mode	AMR_mode;
	int			dtx;

	CDirectSoundRecorder*	pAudioCapture;
	CDirectSoundPlayer*		pAudioPlayer;

	CVideoAudioBuffer*	pAudioRcvBuffer;	//audio buffer for receiving
	CVideoAudioBuffer*	pAudioSendBuffer;	//audio buffer for sending

	HANDLE hTrdAudioRTP;

	bool isAudioCaptureRunning;
	bool isAudioPlayerRunning;

};

#endif //#ifndef _CAUDIOOPERRATION_H_
