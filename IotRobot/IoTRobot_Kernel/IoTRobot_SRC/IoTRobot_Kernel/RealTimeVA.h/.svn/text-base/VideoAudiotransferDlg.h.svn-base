// VideoAudiotransferDlg.h : header file
//

#pragma once

#include "Global.h"
#include "afxwin.h"


// CVideoAudiotransferDlg dialog
class CVideoAudiotransferDlg : public CDialog//, public CAudioCaptureHandler, public CAudioPlayerHandler
{
// Construction
public:
	CVideoAudiotransferDlg(CWnd* pParent = NULL);	// standard constructor
	~CVideoAudiotransferDlg();

	static DWORD WINAPI CVideoAudiotransferDlg::Thread_Circulate_Video(LPVOID pvoid);

// Dialog Data
	enum { IDD = IDD_VIDEOAUDIOTRANSFER_DIALOG };

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);	// DDX/DDV support

// Implementation
protected:
	HICON m_hIcon;

	// Generated message map functions
	virtual BOOL OnInitDialog();
	afx_msg void OnPaint();
	afx_msg HCURSOR OnQueryDragIcon();
	DECLARE_MESSAGE_MAP()

private:
	CRealTimeVA* pRealTimeVA;
	bool isWhileQuit;

public:
	afx_msg void OnBnClickedCheckRecordAndSend();
	afx_msg void OnBnClickedCheckReceiveRemoteAudio();
	CButton m_CheckAudioRecording;
	CButton m_CheckAudioReception;
	afx_msg void OnBnClickedQuit();
	CButton m_CheckVideoSending;
	CButton m_CheckReceiveVideo;
	afx_msg void OnBnClickedCheckSendVideo();
	afx_msg void OnBnClickedCheckReceiveRemoteVideo();
};
