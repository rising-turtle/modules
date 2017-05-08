// VideoAudiotransferDlg.cpp : implementation file
//
#include "stdafx.h"
#include "VideoAudiotransfer.h"
#include "VideoAudiotransferDlg.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#endif


// CVideoAudiotransferDlg dialog
/*--------------------------------------------
// 
--------------------------------------------*/
CVideoAudiotransferDlg::CVideoAudiotransferDlg(CWnd* pParent /*=NULL*/)
	: CDialog(CVideoAudiotransferDlg::IDD, pParent)
{
	m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);

	//global varialbles initialization
	
	pRealTimeVA = NULL;
	isWhileQuit = false;
	//_CrtSetBreakAlloc(302);

}
/*--------------------------------------------
// 
--------------------------------------------*/
CVideoAudiotransferDlg::~CVideoAudiotransferDlg()
{
	if (pRealTimeVA)
	{
		delete pRealTimeVA;
		pRealTimeVA = NULL;
	}

	_CrtDumpMemoryLeaks();
}

void CVideoAudiotransferDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
	DDX_Control(pDX, IDC_CHECK_RECORD_AND_SEND, m_CheckAudioRecording);
	DDX_Control(pDX, IDC_CHECK_RECEIVE_REMOTE_AUDIO, m_CheckAudioReception);
	DDX_Control(pDX, IDC_CHECK_SEND_VIDEO, m_CheckVideoSending);
	DDX_Control(pDX, IDC_CHECK_RECEIVE_REMOTE_VIDEO, m_CheckReceiveVideo);
}

BEGIN_MESSAGE_MAP(CVideoAudiotransferDlg, CDialog)
	ON_WM_PAINT()
	ON_WM_QUERYDRAGICON()
	//}}AFX_MSG_MAP

	ON_BN_CLICKED(IDC_CHECK_RECORD_AND_SEND, &CVideoAudiotransferDlg::OnBnClickedCheckRecordAndSend)
	ON_BN_CLICKED(IDC_CHECK_RECEIVE_REMOTE_AUDIO, &CVideoAudiotransferDlg::OnBnClickedCheckReceiveRemoteAudio)
	ON_BN_CLICKED(ID_QUIT, &CVideoAudiotransferDlg::OnBnClickedQuit)
	ON_BN_CLICKED(IDC_CHECK_SEND_VIDEO, &CVideoAudiotransferDlg::OnBnClickedCheckSendVideo)
	ON_BN_CLICKED(IDC_CHECK_RECEIVE_REMOTE_VIDEO, &CVideoAudiotransferDlg::OnBnClickedCheckReceiveRemoteVideo)
END_MESSAGE_MAP()


// CVideoAudiotransferDlg message handlers
/*--------------------------------------------
// 
--------------------------------------------*/
BOOL CVideoAudiotransferDlg::OnInitDialog()
{
	CDialog::OnInitDialog();

	// Set the icon for this dialog.  The framework does this automatically
	//  when the application's main window is not a dialog
	SetIcon(m_hIcon, TRUE);			// Set big icon
	SetIcon(m_hIcon, FALSE);		// Set small icon

	//make instances of every module


	this->pRealTimeVA	= new CRealTimeVA();


	IoTRobot_RTP_Param rtpParamVA;
	rtpParamVA.remoteIp[0] = 192;
	rtpParamVA.remoteIp[1] = 168;
	rtpParamVA.remoteIp[2] = 1;
	rtpParamVA.remoteIp[3] = 101;
	rtpParamVA.localAudioRtpPort	= LOCAL_AUDIO_RTP_PORT;
	rtpParamVA.localVideoPort		= LOCAL_VIDEO_RTP_PORT;
	rtpParamVA.remoteAudioRtpPort	= REMOTE_AUDIO_RTP_PORT;
	rtpParamVA.remoteVideoPort		= REMOTE_VIDEO_RTP_PORT;

	IoTRobot_RealTimeVA_VideoParam videoParamVA;
	videoParamVA.doesUseDefaultCamera = false;
	videoParamVA.frameRate		= 20;
	videoParamVA.jpegQulity		= 30;
	videoParamVA.videoHeight	= 240;
	videoParamVA.videoWidth		= 320;

	IoTRobot_RealTimeVA_AudioParam audioParamVA;
	audioParamVA.hWnd = AfxGetMainWnd()->m_hWnd;


	this->pRealTimeVA->RealTimeVAInit(rtpParamVA, videoParamVA, audioParamVA);



	this->pRealTimeVA->RealTimeVARun();

	CreateThread(NULL, 0, Thread_Circulate_Video, (LPVOID)this, 0, NULL);
	


	return TRUE;  // return TRUE  unless you set the focus to a control
}

// If you add a minimize button to your dialog, you will need the code below
//  to draw the icon.  For MFC applications using the document/view model,
//  this is automatically done for you by the framework.

void CVideoAudiotransferDlg::OnPaint()
{
	if (IsIconic())
	{
		CPaintDC dc(this); // device context for painting

		SendMessage(WM_ICONERASEBKGND, reinterpret_cast<WPARAM>(dc.GetSafeHdc()), 0);

		// Center icon in client rectangle
		int cxIcon = GetSystemMetrics(SM_CXICON);
		int cyIcon = GetSystemMetrics(SM_CYICON);
		CRect rect;
		GetClientRect(&rect);
		int x = (rect.Width() - cxIcon + 1) / 2;
		int y = (rect.Height() - cyIcon + 1) / 2;

		// Draw the icon
		dc.DrawIcon(x, y, m_hIcon);
	}
	else
	{
		CDialog::OnPaint();
	}
}

// The system calls this function to obtain the cursor to display while the user drags
//  the minimized window.
HCURSOR CVideoAudiotransferDlg::OnQueryDragIcon()
{
	return static_cast<HCURSOR>(m_hIcon);
}

/*-----------------------------------------------------
// 
-----------------------------------------------------*/
void CVideoAudiotransferDlg::OnBnClickedCheckRecordAndSend()
{
	bool result;

	if (this->m_CheckAudioRecording.GetCheck())
	{
		this->pRealTimeVA->RealTimeVARun();
	}
	else
	{
		this->pRealTimeVA->RealTimeVAStop();
	}

}
/*-----------------------------------------------------
// 
-----------------------------------------------------*/
void CVideoAudiotransferDlg::OnBnClickedCheckReceiveRemoteAudio()
{
	if (this->m_CheckAudioReception.GetCheck())
	{
		this->pRealTimeVA->SetLocalAudioPlayingStatus(true);
	}
	else
	{
		this->pRealTimeVA->SetLocalAudioPlayingStatus(false);
	}

}
/*-----------------------------------------------------
// 
-----------------------------------------------------*/
void CVideoAudiotransferDlg::OnBnClickedQuit()
{
	if (this->pRealTimeVA)
	{
		this->pRealTimeVA->RealTimeVAStop();
		Sleep(200);
		this->pRealTimeVA->RealTimeVAUninit();

		Sleep(100);
	}

	::PostMessage(AfxGetMainWnd()->m_hWnd, WM_CLOSE, 0, 0); 
}
/*-----------------------------------------------------
// 
-----------------------------------------------------*/
void CVideoAudiotransferDlg::OnBnClickedCheckSendVideo()
{


	if (this->m_CheckVideoSending.GetCheck())
	{
		
	}
	else
	{
		
	}
}

void CVideoAudiotransferDlg::OnBnClickedCheckReceiveRemoteVideo()
{
	if (this->m_CheckReceiveVideo.GetCheck())
	{

		isWhileQuit = true;
	}
	else
	{
		isWhileQuit = false;
	}
}

/*--------------------------------------------
// Video Transfer thread. Encode, send and receive 
// video data via RTP
--------------------------------------------*/
DWORD WINAPI CVideoAudiotransferDlg::Thread_Circulate_Video(LPVOID pvoid)
{
	CVideoAudiotransferDlg * mainClass = (CVideoAudiotransferDlg *)pvoid;

	BYTE rgbArr[320 * 240 * 3];
	UINT rgbSize; 

	while (!mainClass->isWhileQuit)
	{
		if (mainClass->pRealTimeVA->GetRemoteVideoFrame(rgbArr, &rgbSize))
		{
			mainClass->pRealTimeVA->SendLocalVideoFrame(rgbArr, rgbSize);
		}
		Sleep(20);
	}

	return 0;
}

