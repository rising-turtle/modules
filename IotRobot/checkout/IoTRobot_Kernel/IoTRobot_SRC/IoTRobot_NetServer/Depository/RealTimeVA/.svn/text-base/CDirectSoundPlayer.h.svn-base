#pragma once
//#include <mmsystem.h>
#include <dsound.h>
#include "AudioGlobal.h"

class CAudioPlayerHandler {
public:
	virtual void AudioPlayerCallback(unsigned char * pBuffer, long nBufferLen) = 0 ; 
};
class CDirectSoundPlayer
{
protected:
	//DirectSound设备对象表示一个播放设备，它被用来管理设备和创建声音缓冲区。
	IDirectSound8 *   m_pDirSound;		// DirectSound component
	IDirectSoundBuffer8 * m_pDSBuf;		// Sound Buffer object
	IDirectSoundNotify8 * m_pDSNotify;  // Notification object
	WAVEFORMATEX   m_wfxOutput ;		// Wave format of output 

	// some codes from capture audio 
	DSBPOSITIONNOTIFY     m_aPosNotify[NUM_REC_NOTIFICATIONS + 1]; //notify flag array 
	DWORD        m_dwPlayBufSize;		//play loop buffer size 
	DWORD        m_dwNextPlayOffset;	//offset in loop buffer 
	DWORD        m_dwNotifySize;		//notify pos when loop buffer need to emit the event
	CAudioPlayerHandler* m_stream_handler ; // caller stream buffer filler
public:
	BOOL		m_bPlaying ; 
	HANDLE		m_hNotifyEvent;   //notify event 
	BOOL		LoadStreamData() ; 
public:
	static UINT notify_stream_thd(LPVOID data) ; 
protected:
	HRESULT InitDirectSound(HWND hWnd) ; 
	HRESULT FreeDirectSound() ; 
	IDirectSoundBuffer8 *CreateStreamBuffer(IDirectSound8* pDS, WAVEFORMATEX* wfx) ; 
	BOOL SetWavFormat(WAVEFORMATEX * wfx) ; 
public:
	CDirectSoundPlayer(void);
	~CDirectSoundPlayer(void);
	BOOL Open(HWND hWnd, CAudioPlayerHandler * stream_handler) ; 
	BOOL Close() ; 
	BOOL BeginPlay(BOOL bPlaying) ; 
};
