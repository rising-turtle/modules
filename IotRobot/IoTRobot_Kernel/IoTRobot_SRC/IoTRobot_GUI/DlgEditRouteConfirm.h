#pragma once


// DlgEditRouteConfirm dialog

class CDlgEditRouteConfirm : public CDialog
{
	DECLARE_DYNAMIC(CDlgEditRouteConfirm)

public:
	CDlgEditRouteConfirm(CWnd* pParent = NULL);   // standard constructor
	virtual ~CDlgEditRouteConfirm();

// Dialog Data
	enum { IDD = IDD_DIALOG_SPECIFIED_ROUTE_CONFIRM };

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support

	DECLARE_MESSAGE_MAP()
public:

	int m_nConfirmRslt;
	afx_msg void OnBnClickedOk();
	afx_msg void OnBnClickedOk2();
	afx_msg void OnBnClickedCancel();
};
