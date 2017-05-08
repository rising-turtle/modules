#pragma once


// CDlgControlStick dialog

class CDlgControlStick : public CDialog
{
	DECLARE_DYNAMIC(CDlgControlStick)

public:
	CDlgControlStick(CWnd* pParent = NULL);   // standard constructor
	virtual ~CDlgControlStick();

// Dialog Data
	enum { IDD = IDD_DIALOG_CONTROL_STICK };

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support

	DECLARE_MESSAGE_MAP()
public:
	virtual BOOL OnInitDialog();
	afx_msg void OnPaint();
	afx_msg void OnLButtonDown(UINT nFlags, CPoint point);
	afx_msg void OnLButtonUp(UINT nFlags, CPoint point);
	afx_msg void OnMouseMove(UINT nFlags, CPoint point);

	int m_bStart2MoveStick;
	int m_nStickX,m_nStickY;
	int m_nMiddleX,m_nMiddleY;
	int m_nRadius1,m_nRadius2,m_nRadius3;
	int m_nGear,m_nAngle;
	afx_msg BOOL OnEraseBkgnd(CDC* pDC);

	bool m_bStopSendCtrlCmd;
	static UINT ThreadSendCtrlCmd(LPVOID lpParam);
	void *m_pApp;
	HANDLE m_hThreadSendCtrlCmd;
	afx_msg void OnBnClickedOk();
	afx_msg void OnBnClickedCancel();
	afx_msg void OnLButtonDblClk(UINT nFlags, CPoint point);
};
