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
����DirectSound¼������Ҫ˼·�������ȸ���ѡ���¼���豸�����豸����
Ȼ��ͨ���豸���󴴽��������������󣬿�ʼ¼����ʱ���豸������д��
��������Ӧ�ó��������Ĵӻ����������ݶ�����д�ļ����ɣ���ʵ����¼��
���ܡ�����򵥽���һ��dsound��֪ͨ���ܣ�Ӧ�ó���ᴴ��һ��֪ͨ����
Ȼ��֪ͨ������Ȼ���趨֪ͨλ�ã�position����ʲô��֪ͨλ���أ�
���绺�����Ĵ�СΪ4000�ֽڣ�������뵱���ݴﵽ������һ���ʱ���ܵõ�
֪ͨ��ʼcopy���ݣ���ô��ʱ��Ϳ��Խ�֪ͨλ���趨Ϊ2000��֪ͨλ�ÿ���
������趨���������������ݴﵽ���趨��λ��ʱ���ͻ�֪ͨӦ�ó���
������������copy���ļ��У���������ѭ�����õģ�����������������Ժ�
�ͻ��ͷ��ʼ�������ݣ����ԣ�����������һ�߶���һ��д�Ĺ��̡�

���������ҽ�һ��¼������Ҫ���裬����ʹ��ҵ�˼·������һЩ

����1��ö��¼�����豸

����2������ѡ����豸�����豸����

����3�������豸���󴴽�����������

����4������֪ͨ����

����5�����������̣߳�������������������д���ļ���
*/
class CDirectSoundRecorder
{
public:
	BOOL        m_bRecording ;  //recording now ? also used by event recv thread 
protected:
	//DirectSound¼���õ��������ǳ���Ҫ�Ķ���

	//IDirectSoundCapture8 ���豸���󣬸�����¼�����豸����
	//���豸�������øö�����Ի�ȡ�豸�����ԡ�
	LPDIRECTSOUNDCAPTURE8    m_pCapDev ;   //capture device ptr

	//IDirectSoundCaptureBuffer8������������
	//�ö������豸���󴴽�����Ҫ����������Ƶ����
	LPDIRECTSOUNDCAPTUREBUFFER m_pCapBuf ;   //capture loop buffer ptr

	//IDirectSoundNotify8 ���¼�֪ͨ���󣬸ö�������֪ͨ
	//Ӧ�ó���ӻ������н�����ȡ�ߣ�д���ļ�����������
	LPDIRECTSOUNDNOTIFY8    m_pNotify ;   //capture auto-notify event callback handler ptr

	GUID        m_guidCapDevId ;  //capture device id �豸id
	WAVEFORMATEX      m_wfxInput;   //input wave format description struct �������Ƶ��ʽ
	DSBPOSITIONNOTIFY     m_aPosNotify[NUM_REC_NOTIFICATIONS + 1]; //notify flag array����֪ͨ��־������ 
	HANDLE		m_hNotifyEvent;   //notify event ֪ͨ�¼�
	DWORD		m_dwCapBufSize;  //capture loop buffer size ¼���û������Ĵ�С
	DWORD		m_dwNextCapOffset;//offset in loop buffer ƫ��λ��
	DWORD		m_dwNotifySize;  //notify pos when loop buffer need to emit the event ֪ͨλ��
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