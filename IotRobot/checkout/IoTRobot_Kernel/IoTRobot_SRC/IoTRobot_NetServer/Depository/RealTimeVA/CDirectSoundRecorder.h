#pragma once
#ifndef _CAPTURE_SOUND_H_
#define _CAPTURE_SOUND_H_
//#include <mmsystem.h>
#include <dsound.h>
#include "AudioGlobal.h"

class CAudioCaptureHandler {
public:
	virtual void AudioCaptureCallback(BYTE* pBuffer, long lBufferSize) = 0 ; 
};
/*
利用DirectSound录音的主要思路，就是先根据选择的录音设备创建设备对象，
然后通过设备对象创建辅助缓冲区对象，开始录音的时候，设备将数据写入
缓冲区，应用程序主动的从缓冲区将数据读出来写文件即可，就实现了录音
功能。这里简单介绍一下dsound的通知功能，应用程序会创建一个通知对象，
然后将通知对象邦定，然后设定通知位置（position），什么是通知位置呢，
比如缓冲区的大小为4000字节，如果你想当数据达到缓冲区一半的时候能得到
通知开始copy数据，那么此时你就可以将通知位置设定为2000，通知位置可以
任意的设定，当缓冲区的数据达到你设定的位置时，就会通知应用程序将
缓冲区的数据copy到文件中，缓冲区是循环利用的，当缓冲区填充满了以后，
就会从头开始充填数据，所以，缓冲区就是一边读，一边写的过程。

　　下面我讲一下录音的主要步骤，可以使大家的思路更清晰一些

　　1、枚举录音的设备

　　2、根据选择的设备创建设备对象

　　3、利用设备对象创建缓冲区对象

　　4、设置通知机制

　　5、创建工作线程，用来将缓冲区的数据写入文件。
*/
class CDirectSoundRecorder
{
public:
	BOOL        m_bRecording ;  //recording now ? also used by event recv thread 
protected:
	//DirectSound录音用到的三个非常重要的对象：

	//IDirectSoundCapture8 ，设备对象，根据你录音的设备创建
	//的设备对象，利用该对象可以获取设备的属性。
	LPDIRECTSOUNDCAPTURE8    m_pCapDev ;   //capture device ptr

	//IDirectSoundCaptureBuffer8，缓冲区对象，
	//该对象由设备对象创建，主要用来操作音频数据
	LPDIRECTSOUNDCAPTUREBUFFER m_pCapBuf ;   //capture loop buffer ptr

	//IDirectSoundNotify8 ，事件通知对象，该对象用来通知
	//应用程序从缓冲区中将数据取走，写入文件保存起来。
	LPDIRECTSOUNDNOTIFY8    m_pNotify ;   //capture auto-notify event callback handler ptr

	GUID        m_guidCapDevId ;  //capture device id 设备id
	WAVEFORMATEX      m_wfxInput;   //input wave format description struct 输入的音频格式
	DSBPOSITIONNOTIFY     m_aPosNotify[NUM_REC_NOTIFICATIONS + 1]; //notify flag array设置通知标志的数组 
	HANDLE		m_hNotifyEvent;   //notify event 通知事件
	DWORD		m_dwCapBufSize;  //capture loop buffer size 录音用缓冲区的大小
	DWORD		m_dwNextCapOffset;//offset in loop buffer 偏移位置
	DWORD		m_dwNotifySize;  //notify pos when loop buffer need to emit the event 通知位置
	CAudioCaptureHandler*     m_frame_handler ; // outer frame data dealer ptr 
public: // callback func to add enum devices string name 
	static BOOL CALLBACK enum_dev_proc(LPGUID lpGUID, LPCTSTR lpszDesc, 
		LPCTSTR lpszDrvName, LPVOID lpContext ) ; 
	static UINT notify_capture_thd(LPVOID data) ; 
protected:
	HRESULT InitDirectSound(GUID dev_id = GUID_NULL) ; 
	HRESULT FreeDirectSound() ; 
	HRESULT InitNotifications() ; 
	HRESULT CreateCaptureBuffer(WAVEFORMATEX * wfx) ; 
	HRESULT StartOrStopRecord(BOOL bStartRec) ;
	HRESULT RecordCapturedData() ; 
	void    SetWavFormat(WAVEFORMATEX * wfx) ; 
public:
	CDirectSoundRecorder(void);
	~CDirectSoundRecorder(void);
	BOOL EnumDevices(HWND hList) ;
	BOOL Open(void) ; 
	BOOL Close() ; 
	bool BeginCapture(BOOL bGrabAudioFrames, CAudioCaptureHandler* frame_handler) ; 
}; 
#endif