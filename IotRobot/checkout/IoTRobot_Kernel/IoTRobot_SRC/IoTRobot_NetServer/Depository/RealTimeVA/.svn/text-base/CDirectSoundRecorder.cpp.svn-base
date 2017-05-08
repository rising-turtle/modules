//#include "StdAfx.h"
#include "CDirectSoundRecorder.h"
#ifndef SAFE_DELETE
#define SAFE_DELETE(p)  { if(p) { delete (p);     (p)=NULL; } }
#endif 
#ifndef SAFE_RELEASE
#define SAFE_RELEASE(p) { if(p) { (p)->Release(); (p)=NULL; } }
#endif 
#ifndef MAX
#define MAX(a,b)        ( (a) > (b) ? (a) : (b) )
#endif 
CDirectSoundRecorder::CDirectSoundRecorder(void)
{
	if(FAILED(CoInitialize(NULL))) /*, COINIT_APARTMENTTHREADED)))*/
	{
		//AfxMessageBox(_T("CDirectSoundRecorder CoInitialize Failed!\r\n")); 
		return;
	}
	m_pCapDev = NULL ;
	m_pCapBuf = NULL ;
	m_pNotify = NULL ; 
	// set default wave format PCM
	ZeroMemory( &m_wfxInput, sizeof(m_wfxInput));
	m_wfxInput.wFormatTag = WAVE_FORMAT_PCM; 
	m_guidCapDevId = GUID_NULL ; 
	m_bRecording = FALSE ; 
	m_hNotifyEvent = NULL ; 
} 
CDirectSoundRecorder::~CDirectSoundRecorder(void)
{
	CoUninitialize() ; 
} 
/*----------------------------------------------------------------------------------
// 为枚举设备，你必须首先创建一个回调函数，
// 它将在枚举每个系统设备时被调用。
// 你能够在这个函数中进行任何操作，
// 并且你可以给它任意的函数名，
// 但是你必须根据 DSEnumCallback 的原型声明它。
// 如果要继续进行枚举，那么回调函数必须返回 TRUE，否则返回FALSE
// NOTE：第一个被枚举到的设备通常称为主要声音设备，这时回调函数的
// 参数lpGUID 为NULL。这个设备代表了用户在控制面板中设置的首选播放设备。
----------------------------------------------------------------------------------*/
BOOL CALLBACK CDirectSoundRecorder::enum_dev_proc(LPGUID lpGUID, LPCTSTR lpszDesc, 
										   LPCTSTR lpszDrvName, LPVOID lpContext) 
{
	HWND hList = (HWND)lpContext;
	if(!hList) return FALSE ; 
	LPGUID lpTemp = NULL; 
	if (lpGUID != NULL) {
		// NULL only for "Primary Sound Driver".
		if ((lpTemp = (LPGUID)malloc(sizeof(GUID))) == NULL) return(TRUE);
		memcpy(lpTemp, lpGUID, sizeof(GUID));
	}
	::SendMessage(hList, CB_ADDSTRING, 0,(LPARAM)lpszDesc);
	::SendMessage(hList, LB_SETITEMDATA, 0, (LPARAM)lpTemp) ; 
	free(lpTemp);
	return(TRUE);
} 
/*--------------------------------------------
// notify event recv thread
--------------------------------------------*/
UINT CDirectSoundRecorder::notify_capture_thd(LPVOID data)
{
	CDirectSoundRecorder * pado = static_cast<CDirectSoundRecorder *>(data) ; 
	MSG   msg;
	HRESULT hr ; 
	DWORD dwResult ; 
	while(pado->m_bRecording) {
		dwResult = MsgWaitForMultipleObjects( 1, &(pado->m_hNotifyEvent), FALSE, INFINITE, QS_ALLEVENTS );
		switch( dwResult ) {
			case WAIT_OBJECT_0 + 0:
				// g_hNotificationEvents[0] is signaled 
				if( FAILED( hr = pado->RecordCapturedData() ) ) {
				//	AfxMessageBox(_T("Error handling DirectSound notifications.")) ; 
					pado->m_bRecording = FALSE ; 
			  }
			  break;
			case WAIT_OBJECT_0 + 1:
				// Windows messages are available
				while( PeekMessage( &msg, NULL, 0, 0, PM_REMOVE ) ) { 
					TranslateMessage( &msg ); 
					DispatchMessage( &msg ); 
					if( msg.message == WM_QUIT ) pado->m_bRecording = FALSE ; 
				}
			break;
		}//switch( dwResult )
	}
	//AfxEndThread(0, TRUE) ; 
	return 0 ; 
} 
/*--------------------------------------------
//
--------------------------------------------*/
BOOL CDirectSoundRecorder::EnumDevices(HWND hList) 
{
	//The DirectSoundCaptureEnumerate function enumerates
	//the DirectSoundCapture objects installed in the system.
	if (FAILED(DirectSoundCaptureEnumerate (	
		(LPDSENUMCALLBACK)(CDirectSoundRecorder::enum_dev_proc),
		(VOID*)&hList)))//Address of the user-defined context passed to
						//the enumeration callback function every time that function is called. 
	{
		return(FALSE);
	}
	return (TRUE) ; 
} 
/*--------------------------------------------
// Open函数是此类中第一个被调用的函数
// 然后才可调用StartOrStopRecord函数
--------------------------------------------*/
BOOL CDirectSoundRecorder::Open(void)
{
	HRESULT hr ; 
	if(!m_bRecording) 
	{
		hr = InitDirectSound() ; 
	}
	return (FAILED(hr)) ? FALSE : TRUE ; 
} 
/*--------------------------------------------
// 在析构此类时，一定要先调用此Close()函数
--------------------------------------------*/
BOOL CDirectSoundRecorder::Close() 
{
	HRESULT hr ; 
	hr = FreeDirectSound() ; 
	CloseHandle(m_hNotifyEvent) ; 
	return (FAILED(hr)) ? FALSE : TRUE ; 
} 
/*--------------------------------------------
//
--------------------------------------------*/
HRESULT CDirectSoundRecorder::InitDirectSound(GUID dev_id)
{
	HRESULT hr ; 
	m_guidCapDevId = dev_id ; //dev_id默认为GUID_NULL
	ZeroMemory( &m_aPosNotify, sizeof(DSBPOSITIONNOTIFY) * (NUM_REC_NOTIFICATIONS + 1) ) ;
	m_dwCapBufSize = 0 ;
	m_dwNotifySize = 0 ; 
	// Create IDirectSoundCapture using the preferred capture device
	hr = DirectSoundCaptureCreate(&m_guidCapDevId, &m_pCapDev, NULL ) ; //打开音频捕获设备
	// init wave format 
	SetWavFormat(&m_wfxInput) ; 
	return (FAILED(hr)) ? S_FALSE : S_OK ; 
} 
/*--------------------------------------------
//
--------------------------------------------*/
HRESULT CDirectSoundRecorder::FreeDirectSound()
{
	// Release DirectSound interfaces
	SAFE_RELEASE( m_pNotify ) ;
	SAFE_RELEASE( m_pCapBuf ) ;
	SAFE_RELEASE( m_pCapDev ) ; 
	return S_OK;
} 
/*--------------------------------------------
// 
--------------------------------------------*/
HRESULT CDirectSoundRecorder::CreateCaptureBuffer(WAVEFORMATEX * wfx) 
{
	HRESULT hr;
	DSCBUFFERDESC dscbd; 
	SAFE_RELEASE( m_pNotify );
	SAFE_RELEASE( m_pCapBuf ); 
	// Set the notification size
	m_dwNotifySize = BUF_SIZE;//MAX( 1024, wfx->nAvgBytesPerSec / 8 ) ; 
	m_dwNotifySize -= m_dwNotifySize % wfx->nBlockAlign ; 
	// Set the buffer sizes 
	m_dwCapBufSize = m_dwNotifySize * NUM_REC_NOTIFICATIONS; 
	SAFE_RELEASE( m_pNotify );
	SAFE_RELEASE( m_pCapBuf ); 
	// Create the capture buffer
	ZeroMemory( &dscbd, sizeof(dscbd) );
	dscbd.dwSize        = sizeof(dscbd);
	dscbd.dwBufferBytes = m_dwCapBufSize;
	dscbd.lpwfxFormat   = wfx ; // Set the format during creatation
	//创建一个录音的buffer对象，这个函数的一个参数采用
	//DSCBUFFERDESC类型的结构来说明buffer的一些特性
	if( FAILED( hr = m_pCapDev->CreateCaptureBuffer( &dscbd, &m_pCapBuf, NULL ) ) )
		return S_FALSE ; 
	/*说明一下，如果你的应用程序一边播放的同时进行录制，
	如果你录制的buffer格式和你的主缓冲buffer不一样，
	那么你创建录制buffer对象就会失败，原因在于，有些声卡只支持一种时钟，
	不能同时支持录音和播放同时以两种不同的格式进行。*/

	m_dwNextCapOffset = 0; 
	if( FAILED( hr = InitNotifications() ) ) //设置通知点
		return S_FALSE ; 
	return S_OK;
} 
/*--------------------------------------------
// 设置通知点
--------------------------------------------*/
HRESULT CDirectSoundRecorder::InitNotifications() 
{
	HRESULT hr; 
	int i ; 
	if( NULL == m_pCapBuf )
		return S_FALSE; 
	// create auto notify event 
	m_hNotifyEvent = CreateEvent( NULL, FALSE, FALSE,NULL ); 
	// Create a notification event, for when the sound stops playing
	if( FAILED( hr = m_pCapBuf->QueryInterface( IID_IDirectSoundNotify, (VOID**)&m_pNotify ) ) )
		return S_FALSE ; 
	// Setup the notification positions
	for( i = 0; i < NUM_REC_NOTIFICATIONS; i++ ) {
		m_aPosNotify[i].dwOffset = (m_dwNotifySize * i) + m_dwNotifySize - 1;
		m_aPosNotify[i].hEventNotify = m_hNotifyEvent;             
	} 
	// Tell DirectSound when to notify us. the notification will come in the from 
	// of signaled events that are handled in WinMain()
	if( FAILED( hr = m_pNotify->SetNotificationPositions( NUM_REC_NOTIFICATIONS, m_aPosNotify ) ) )
		return S_FALSE ; 
	return S_OK;
} 
/*--------------------------------------------
// 开始函数
--------------------------------------------*/
HRESULT CDirectSoundRecorder::StartOrStopRecord(BOOL bStartRec)
{
	//在这之前调用了Open函数，对m_wfxInput进行了设置
	HRESULT hr; 
	if( bStartRec ) {
		// Create a capture buffer, and tell the capture 
		// buffer to start recording   
		// 在CreateCaptureBuffer函数中对m_pCapBuf进行了赋值
		// 在InitNotifications函数中对m_pNotify进行了赋值
		if( FAILED( hr = CreateCaptureBuffer( &m_wfxInput ) ) )
			return S_FALSE ;

		// 在CreateCaptureBuffer函数中对m_pCapBuf和m_pNotify
		// 进行了赋值
		if( FAILED( hr = m_pCapBuf->Start( DSCBSTART_LOOPING ) ) )
			return S_FALSE ; 

		// create notify event recv thread
		// 对于录制来说，使用通知机制是必须的

		LPDWORD ID=0;
		CreateThread(NULL,0,(LPTHREAD_START_ROUTINE)CDirectSoundRecorder::notify_capture_thd,this,0,ID);
		//AfxBeginThread(CDirectSoundRecorder::notify_capture_thd, (LPVOID)(this)) ;
	} else { 
		// Stop the capture and read any data that 
		// was not caught by a notification
		if( NULL == m_pCapBuf )
			return S_OK;
		// wait until the notify_event_thd thread exit and release the resources.
		
		// Stop the buffer, and read any data that was not 
		// caught by a notification
		if( FAILED( hr = m_pCapBuf->Stop() ) )
			return S_OK ; 
		if( FAILED( hr = RecordCapturedData() ) )
			return S_FALSE ; 
		Sleep(100) ; 
	}
	return S_OK;
} 
/*--------------------------------------------
//
--------------------------------------------*/
HRESULT CDirectSoundRecorder::RecordCapturedData() 
{
	HRESULT hr;
	VOID*   pbCaptureData    = NULL;
	DWORD   dwCaptureLength;
	VOID*   pbCaptureData2   = NULL;
	DWORD   dwCaptureLength2;
	DWORD   dwReadPos;
	DWORD   dwCapturePos;
	LONG lLockSize; 
	if( NULL == m_pCapBuf )
		return S_FALSE; 
	/*
	IDirectSoundCaptureBuffer8::GetCurrentPosition方法可以获取buffer中
	read指针和录制指针的偏差。Read指针指向填充到该buffer中的数据的
	最末端，capture指针则指向复制到硬件的数据的末端，你read指针指向
	的前段数据都是安全数据，你都可以安全的复制。
	*/
	if( FAILED( hr = m_pCapBuf->GetCurrentPosition( &dwCapturePos, &dwReadPos ) ) )
		return S_FALSE; 
	lLockSize = dwReadPos - m_dwNextCapOffset;
	if( lLockSize < 0 )
		lLockSize += m_dwCapBufSize; 
	// Block align lock size so that we are always write on a boundary
	lLockSize -= (lLockSize % m_dwNotifySize); 
	if( lLockSize == 0 )
		return S_FALSE; 
	// Lock the capture buffer down
	if( FAILED( hr = m_pCapBuf->Lock( m_dwNextCapOffset, lLockSize,
		&pbCaptureData, &dwCaptureLength, 
		&pbCaptureData2, &dwCaptureLength2, 0L ) ) )
		return S_FALSE ; 
	/*
	这里解释一下，IDirectSoundBuffer8：：Lock可能返回两个地址的原因
	在于你锁定内存的数量是随机的，有时你锁定的区域正好包含buffer的
	起始点，这时，就会给你返回两个地址，举个例子：

　　假设你锁定了30,000字节，偏移位置为20，000字节，也就是开始位置，
	如果你的缓冲区的大小为40，000字节，此时就会给你返回四个数据：

　　·内存地址的偏移位置20，000，

　　·从偏移位置到buffer的最末端的字节数，也是20，000，你要在第一个地址读取20，000个字节的内容

　　·偏移量为0的地址

　　·从起始点开始的字节数，也就是10，000字节，你要从第二个地址，也就是从0点开始读取10，000字节。

　　如果不包含零点，最后两个数值返回为NULL和0，
	*/

	// call the outer data handler
	// $$$$$$$$$$$$ 调用回调函数，将采集到音频数据取出去 $$$$$$$$$$$$
	if(m_frame_handler) {
		m_frame_handler->AudioCaptureCallback((BYTE*)pbCaptureData, dwCaptureLength) ; 
	}

	// Move the capture offset along
	m_dwNextCapOffset += dwCaptureLength; 
	m_dwNextCapOffset %= m_dwCapBufSize; // Circular buffer 
	if( pbCaptureData2 != NULL ) {
		// call the outer data handler 
		if(m_frame_handler) {
			m_frame_handler->AudioCaptureCallback((BYTE*)pbCaptureData2, dwCaptureLength2) ; 
		} 
		// Move the capture offset along
		m_dwNextCapOffset += dwCaptureLength2; 
		m_dwNextCapOffset %= m_dwCapBufSize; // Circular buffer
	} 
	// Unlock the capture buffer
	m_pCapBuf->Unlock( pbCaptureData,  dwCaptureLength, pbCaptureData2, dwCaptureLength2 ); 
	return S_OK;
} 
/*--------------------------------------------
// 
--------------------------------------------*/
void CDirectSoundRecorder::SetWavFormat(WAVEFORMATEX * wfx)
{
	// get the default capture wave formate 
	ZeroMemory(wfx, sizeof(WAVEFORMATEX)) ; 
	//wfx->wFormatTag = WAVE_FORMAT_PCM; 
	//// 8KHz, 16 bits PCM, Mono
	wfx->wFormatTag = WAVE_FORMAT_PCM;	//设置波形声音的格式
	wfx->nChannels = 1;					//设置音频文件的通道数量，对于单声道的声音，此此值为1。
	wfx->nSamplesPerSec = 8000;			//设置每个声道播放和记录时的样本频率。
										//如果wFormatTag = WAVE_FORMAT_PCM，
										//那么nSamplesPerSec通常为8.0 kHz，11.025 kHz，22.05 kHz和44.1 kHz。
										//例如对于采样率为11.025 kHz的音频，nSamplesPerSec将被设为11025。
	wfx->wBitsPerSample = 16;	//每次采样样本的大小，以bit为单位

	//设置请求的平均数据传输率，单位byte/s。
	wfx->nAvgBytesPerSec = wfx->nChannels * wfx->nSamplesPerSec * wfx->wBitsPerSample / 8;
											 //这个值对于创建缓冲大小是很有用的。
	//以字节为单位设置块对齐。块对齐是指最小数据的原子大小。
	wfx->nBlockAlign = wfx->nChannels * wfx->wBitsPerSample / 8;	
									//如果wFormatTag = WAVE_FORMAT_PCM，
									//nBlockAlign 为(nChannels*wBitsPerSample)/8。
	wfx->cbSize = 0;	//额外信息的大小，以字节为单位，
} 
/*--------------------------------------------
// 
--------------------------------------------*/
bool CDirectSoundRecorder::BeginCapture(BOOL bGrabAudioFrames, CAudioCaptureHandler* frame_handler) 
{
	m_frame_handler = frame_handler ; //设置回调函数
	m_bRecording = bGrabAudioFrames ; 
	HRESULT result = StartOrStopRecord(m_bRecording) ; 
	return !FAILED(result);
}