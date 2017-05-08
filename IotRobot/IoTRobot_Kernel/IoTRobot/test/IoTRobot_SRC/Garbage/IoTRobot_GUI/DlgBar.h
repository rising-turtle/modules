#pragma once


// DlgBar dialog

class DlgBar : public CDialog
{
	DECLARE_DYNAMIC(DlgBar)

public:
	DlgBar(CWnd* pParent = NULL);   // standard constructor
	virtual ~DlgBar();

// Dialog Data
	enum { IDD = IDD_DIALOGBAR };

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support

	DECLARE_MESSAGE_MAP()
public:
	afx_msg void OnBnClickedButton2();
	afx_msg void OnBnClickedButton1();
};
