//#include "StdAfx.h"
#include "CDirectSoundPlayer.h"
#ifndef SAFE_RELEASE
#define SAFE_RELEASE(p) { if(p) { (p)->Release(); (p)=NULL; } }
#endif
#ifndef MAX
#define MAX(a,b)        ( (a) > (b) ? (a) : (b) )
#endif 
/*--------------------------------------------
//  构造函数
//	在此构造函数中没有做太多工作
--------------------------------------------*/
CDirectSoundPlayer::CDirectSoundPlayer(void)
{
	if(FAILED(CoInitialize(NULL))) /*, COINIT_APARTMENTTHREADED)))*/
	{
//		AfxMessageBox(_T("CDirectSoundPlayer CoInitialize Failed!\r\n")); 
		return;
	}
	m_pDirSound = NULL ;        // DirectSound component
	m_pDSBuf = NULL ;   // Sound Buffer object
	m_pDSNotify = NULL ;  // Notification object
	m_hNotifyEvent = NULL ; 
	ZeroMemory(&m_wfxOutput, sizeof(m_wfxOutput)) ; // Wave format of output 
	m_wfxOutput.wFormatTag = WAVE_FORMAT_PCM ; 
	m_dwPlayBufSize = 0 ;  //play loop buffer size 
	m_dwNextPlayOffset = 0 ; //offset in loop buffer 
	m_dwNotifySize = 0 ;  //notify pos when loop buffer need to emit the event
	m_bPlaying = FALSE ; 
}
/*--------------------------------------------
//
--------------------------------------------*/
CDirectSoundPlayer::~CDirectSoundPlayer(void)
{
	FreeDirectSound() ; 
	CoUninitialize() ; 
}
/*--------------------------------------------
//
--------------------------------------------*/
HRESULT CDirectSoundPlayer::InitDirectSound(HWND hWnd)
{
	/*当你通过调用 DirectSoundCreate8 函数创建设备对象时，你能够指定默认设备
	注意：DirectSoundCreate8 不要求调用CoInitialize 或CoInitializeEx。
	如果你的应用程序采集声音，同时播放它们，你可以通过
	DirectSoundFullDuplexCreate8方便地创建播放和采集设备，以及播放和采集缓冲区。
	*/
	if(FAILED(DirectSoundCreate8(NULL,	//第一个参数指定了与对象关联的设备的GUID
										//传入一个空指针来指定默认系统音频设备
								&m_pDirSound,	//第二个参数是远程指针LPDIRECTSOUND的地址，
												//也就是创造的DirectSound对象放置的地址
								NULL)))			//必须为NULL，暂时没有用
	{
//		MessageBox(NULL, _T("Unable to create DirectSound object"), _T("Error"), MB_OK);
		return S_FALSE;
	}

	/*在调用DirectSoundCreate8 函数创建一个设备对象后，
	你的应用程序能够通过调用IDirectSound8::GetCaps方法获取声音设备的性能。
	在一般情况下，这样做是不必要的，因为DirectSound可以自动有效的使用
	硬件加速，我们完全可以不用去管硬件是否具有某些能力*/


	// sets the cooperative level of the application for this sound device
	/*每个DirectSound应用程序都有一个协作级别，用于决定它被允许访问设备的程度。
	通过合作级别，DirectX可以保证所有的程序在使用同一设备时不会发生冲突。
	在创建一个设备对象后，你必须使用 IDirectSound8::SetCooperativeLevel 方法
	为这个设备设置协作级别。如果你不这么做，将无法听到声音。
	*/
	m_pDirSound->SetCooperativeLevel(hWnd, DSSCL_PRIORITY);
	/*当使用一个出于优先协作级别（DSSCL_PRIORITY）的DirectSound设备时，
	应用程序享有对硬件资源的优先权，如硬件混频，设置主缓冲区的格式，
	以及压缩设备的On-board内存。游戏程序在几乎所有环境下都应该使用
	优先协作级别。这个级别给予了应用程序最强大的行为能力，使它能够
	控制采样率和位深度。优先协作级别也允许来自其他应用程序（如IP电话）
	的声音与游戏中的声音一同被听到。
	*/

	// use preset output wave format
	SetWavFormat(&m_wfxOutput) ; 
	return S_OK ; 
}
/*--------------------------------------------
//
--------------------------------------------*/
HRESULT CDirectSoundPlayer::FreeDirectSound()
{
	// make sure the thread gone 
	m_bPlaying = FALSE ; 
	// stop sound play 
	if(m_pDSBuf) m_pDSBuf->Stop();

	// Release the notify event handles
	if(m_hNotifyEvent) {
		CloseHandle(m_hNotifyEvent) ; 
		m_hNotifyEvent = NULL ; 
	}

	Sleep(100) ; 

	// Release DirectSound objects
	SAFE_RELEASE(m_pDSBuf) ; 
	SAFE_RELEASE(m_pDirSound) ; 
	return S_OK ; 
}
/*--------------------------------------------
//  使用此类前，要先调用此open函数
--------------------------------------------*/
BOOL CDirectSoundPlayer::Open(HWND hWnd, CAudioPlayerHandler * stream_handler)
{
	HRESULT hr ; 
	m_stream_handler = stream_handler ; 
	hr = InitDirectSound(hWnd) ; 
	return (FAILED(hr)) ? FALSE : TRUE ; 
}
/*--------------------------------------------
//
--------------------------------------------*/
BOOL CDirectSoundPlayer::Close()
{
	HRESULT hr ; 
	hr = FreeDirectSound() ; 
	return (FAILED(hr)) ? FALSE : TRUE ; 
}
/*--------------------------------------------
// 通知处理线程
--------------------------------------------*/
UINT CDirectSoundPlayer::notify_stream_thd(LPVOID data) 
{
	CDirectSoundPlayer * psmado = static_cast<CDirectSoundPlayer *>(data) ; 
	DWORD dwResult = 0 ; 
	DWORD Num = 0 ;
	while(psmado->m_bPlaying) {
		// Wait for a message
		dwResult = MsgWaitForMultipleObjects(1, &psmado->m_hNotifyEvent, 
			FALSE, INFINITE, QS_ALLEVENTS);
		// Get notification
		switch(dwResult) {
   case WAIT_OBJECT_0:
	   {
		   psmado->LoadStreamData();
	   }
	   break ; 
   default:
	   break ; 
		}
	}
	//AfxEndThread(0, TRUE) ; 
	return 0 ; 
}
/*--------------------------------------------
//
--------------------------------------------*/
IDirectSoundBuffer8 * CDirectSoundPlayer::CreateStreamBuffer(IDirectSound8* pDS, WAVEFORMATEX * wfx)
{
	/*在初始化DirectSound时，它会自动地为你的程序创建一主缓冲，
	这个主缓冲的作用就是混音并送到输出设备。除了主缓冲外，
	程序至少还应该创建一个辅助缓冲，辅助缓冲的作用是储存将要
	使用的声音，它可以在不使用的时候释放掉。
	*/

	/*通常情况下，你并不需要和主缓冲打交道，DirectSound会自己来管理它的，
	除非你要使用自己写的混音部分，这时，DirectSound就会让你自行管理主缓冲*/

	/*在应用程序里，辅助缓冲可以有两种——静态缓冲（一段内存空间一段完整的声音；
	好处在于可以一次将全部的声音存入缓冲）和流缓冲（并不将全部的数据一次读入缓冲，
	而是在播放声音时动态的读入；其好处在于占用空间较小），它们可以分别适应不同的
	程序需求。一般的说，如果声音是需要再三播放的，而且容量有限（好比游戏音效），
	那么使用静态缓冲就更有助于提高程序的效率，相反，如果是很冗长的音乐，还是使用流缓冲的好。
	*/

	IDirectSoundBuffer *  pDSB = NULL ;
	IDirectSoundBuffer8 * pDSBuffer = NULL ;
	DSBUFFERDESC dsbd;
	// calculate play buffer size 
	// Set the notification size
	m_dwNotifySize = BUF_SIZE; //MAX( 1024, wfx->nAvgBytesPerSec / 8 );
	m_dwNotifySize -= m_dwNotifySize % wfx->nBlockAlign; 
	// Set the buffer sizes 
	m_dwPlayBufSize = m_dwNotifySize * NUM_REC_NOTIFICATIONS;

	// create the sound buffer using the header data
	ZeroMemory(&dsbd, sizeof(DSBUFFERDESC));
	dsbd.dwSize = sizeof(DSBUFFERDESC);
	// set DSBCAPS_GLOBALFOCUS to make sure event if the software lose focus could still
	// play sound as well
	/*若设置DSBCAPS_STATIC标志，则缓冲类型是静态缓冲，默认值是流缓冲。
	关于DSBCAPS_CTRLPOSITIONNOTIFY，如果你采用的流（stream）buffer的话，
	就需要边播放，边向buffer中填充数据，就需要设置这个标志，
	这样，在directsound播放到指定位置时，就会触发事件。*/
	dsbd.dwFlags = DSBCAPS_CTRLVOLUME | DSBCAPS_CTRLPOSITIONNOTIFY | DSBCAPS_LOCSOFTWARE | DSBCAPS_GLOBALFOCUS;
	dsbd.dwBufferBytes = m_dwPlayBufSize ;//(m_dwPlayBufSize / wfx->nAvgBytesPerSec)秒钟长度的缓冲
	dsbd.lpwfxFormat = wfx ;	//PCM 格式
	if(FAILED(pDS->CreateSoundBuffer(&dsbd, &pDSB, NULL))) return NULL;
	// get newer interface
	if(FAILED(pDSB->QueryInterface(IID_IDirectSoundBuffer8, (void**)(&pDSBuffer)))) {
		SAFE_RELEASE(pDSB) ; 
		return NULL;
	}
	// return the interface
	return pDSBuffer;
}
/*--------------------------------------------
//
--------------------------------------------*/
BOOL CDirectSoundPlayer::LoadStreamData() 
{
	///////////////////////
	HRESULT hr;
	VOID*   pvStreamData1    = NULL;
	DWORD   dwStreamLength1 = 0 ;
	VOID*   pvStreamData2   = NULL;
	DWORD   dwStreamLength2 = 0 ;
	DWORD   dwWritePos = 0 ;
	DWORD   dwPlayPos = 0 ;
	LONG lLockSize = 0 ;
	/*
	DirectSound通常都保证缓冲里有两个指针，一个是当前的播放位置，
	即当前的播放进度，一个是当前的可以写数据的位置。
	这两个指针都只是相对缓冲而言的偏移而已。我们可以将缓冲想象成
	一个时钟的钟面，而这两个指针则可以作为是钟面上的两个指针。
	如果数据是顺时针的写上去的，那么可以被写数据的位置始终在当前
	的播放进度的前面。
	*/
	/*The write cursor is the point in the buffer ahead of which it is safe to
	write data to the buffer. Data should not be written to the part of the 
	buffer after the play cursor and before the write cursor.*/
	if( FAILED( hr = m_pDSBuf->GetCurrentPosition( &dwPlayPos, &dwWritePos ) ) )
		return S_FALSE; 
	lLockSize = dwWritePos - m_dwNextPlayOffset;
	if( lLockSize < 0 )
		lLockSize += m_dwPlayBufSize;
	// Block align lock size so that we are always write on a boundary
	lLockSize -= (lLockSize % m_dwNotifySize);
	if( lLockSize == 0 ) return S_FALSE;
	// lock the sound buffer at position specified
	/*将整个缓冲区锁定。你指定缓冲区中你打算开始写的偏移位置，并返回该点的内存地址。
	通常你每次只更新缓冲区的一小部分。例如，在播放指针到达缓冲区的四分之二时，
	你能够锁定缓冲区的四分之一部分并写入数据。你永远不能对位于播放指针和写指针
	之间的缓冲区部分进行写操作。
	*/
	if(FAILED(m_pDSBuf->Lock( m_dwNextPlayOffset,	// Offset at which to start lock.
							lLockSize,				// Size of lock
							&pvStreamData1,			// Gets address of first part of lock
							&dwStreamLength1,		// Gets size of first part of lock
							&pvStreamData2,			// Address of wraparound
							&dwStreamLength2,		// Size of wraparound
							0L)))					// Flag
		return FALSE;
	// read in the data
	// $$$$$$$$$$$$$$ 在这个回调函数中要把数据拷进来 $$$$$$$$$$$$$$
	if(m_stream_handler) m_stream_handler->AudioPlayerCallback((BYTE *)pvStreamData1, dwStreamLength1) ;
	// Move the capture offset along
	m_dwNextPlayOffset += dwStreamLength1; 
	m_dwNextPlayOffset %= m_dwPlayBufSize; // Circular buffer
	/*
	因为流缓冲存储通常是循环的（就像循环队列），所以当你锁定缓冲时
	DirectSound会返回2个指针。譬如你从一个只有4，000字节的缓冲中点
	开始锁定3，000字节长的数据，那么DirectSound返回的第一个指针是从
	中点开始的那2，000字节，而第二个指针则是缓冲最前面的那1，000字节。
	当然如果没有发生这种情况第二个指针是NULL。
	*/
	if(pvStreamData2 != NULL) {
		if(m_stream_handler) m_stream_handler->AudioPlayerCallback((BYTE *)pvStreamData2, dwStreamLength2) ; 
		// Move the capture offset along
		m_dwNextPlayOffset += dwStreamLength2; 
		m_dwNextPlayOffset %= m_dwPlayBufSize; // Circular buffer
	}
	// unlock it
	m_pDSBuf->Unlock(pvStreamData1,		// Address of lock start
					dwStreamLength1,	// Size of lock
					pvStreamData2,		// Wraparound portion
					dwStreamLength2) ;	// Wraparound size
	// return a success
	return TRUE;
}
/*--------------------------------------------
//	开始函数
--------------------------------------------*/
BOOL CDirectSoundPlayer::BeginPlay(BOOL bPlaying)
{
	HRESULT hr ; 
	int i;
	m_bPlaying = bPlaying ; 
	if(m_bPlaying) {
		// Create a 2 second buffer to stream in wave
		m_pDSBuf = CreateStreamBuffer(m_pDirSound, &m_wfxOutput) ; 
		if(m_pDSBuf == NULL) return FALSE ; 



		// Create the notification interface
		if(FAILED(m_pDSBuf->QueryInterface(IID_IDirectSoundNotify8, (void**)(&m_pDSNotify)))) 
			return FALSE ;
		// create auto notify event 
		m_hNotifyEvent = CreateEvent( NULL, FALSE, FALSE, NULL );
		// Setup the notification positions
		for( i = 0; i < NUM_REC_NOTIFICATIONS; i++ ) {
			m_aPosNotify[i].dwOffset = (m_dwNotifySize * i) + m_dwNotifySize - 1;
			m_aPosNotify[i].hEventNotify = m_hNotifyEvent;             
		}
		// Tell DirectSound when to notify us. the notification will come in the from 
		// of signaled events that are handled in WinMain()
		/*在你使用流缓冲时，很可能需要知道播放进度已经到什么位置了，
		或者重放被停止没有。你可以通过IDirectSoundNotify::SetNotificationPositions方法
		来在缓冲里设置若干个通知点，当相应的事件在这些点发生时DirectSound会给予通知。
		但是如果音乐已经在播放了，是不允许做这些事的。
		如果你需要设置更多的通知位置，你可以通过结构数组来实现*/
		if( FAILED( hr = m_pDSNotify->SetNotificationPositions( NUM_REC_NOTIFICATIONS, m_aPosNotify ) ) )
			return FALSE;



		m_dwNextPlayOffset = 0 ; 

		// Fill buffer with some sound
		LoadStreamData() ; 
		// Play sound looping
		m_pDSBuf->SetCurrentPosition(0);
		m_pDSBuf->SetVolume(DSBVOLUME_MAX);
		m_pDSBuf->Play(0,
						0,		// Priority for voice management
						DSBPLAY_LOOPING);// DSBPLAY_LOOPING标志是指整个buffer播放到end处然后重新从头的播放
		// create notify event recv thread 
		LPDWORD ID=0;
		CreateThread(NULL,0,(LPTHREAD_START_ROUTINE)CDirectSoundPlayer::notify_stream_thd,this,0,ID);
		//AfxBeginThread(CDirectSoundPlayer::notify_stream_thd, (LPVOID)(this)) ;
	} else {
		// stop play 
		// make sure the thread gone 
		
		// stop sound play 
		if(m_pDSBuf) m_pDSBuf->Stop();
		// Release the notify event handles
		if(m_hNotifyEvent) {
			CloseHandle(m_hNotifyEvent) ; 
			m_hNotifyEvent = NULL ; 
		}
		Sleep(100) ; 
		// Release DirectSound objects
		SAFE_RELEASE(m_pDSBuf) ;  
	}
	return TRUE ; 
}
/*--------------------------------------------
//
--------------------------------------------*/
BOOL CDirectSoundPlayer::SetWavFormat(WAVEFORMATEX * wfx)
{
	// get the default capture wave formate 
	ZeroMemory(wfx, sizeof(WAVEFORMATEX)) ; 
	wfx->wFormatTag = WAVE_FORMAT_PCM;
	// 8KHz, 16 bits PCM, Mono
	wfx->nSamplesPerSec = 8000 ; 
	wfx->wBitsPerSample = 16 ; 
	wfx->nChannels  = 1 ;
	wfx->nBlockAlign = wfx->nChannels * ( wfx->wBitsPerSample / 8 ) ; 
	wfx->nAvgBytesPerSec = wfx->nBlockAlign * wfx->nSamplesPerSec;
	return TRUE ; 
}

