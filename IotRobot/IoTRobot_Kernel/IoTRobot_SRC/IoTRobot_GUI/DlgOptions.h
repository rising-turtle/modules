#pragma once


// CDlgOptions dialog

class CDlgOptions : public CDialog
{
	DECLARE_DYNAMIC(CDlgOptions)

public:
	CDlgOptions(CWnd* pParent = NULL);   // standard constructor
	virtual ~CDlgOptions();

// Dialog Data
	enum { IDD = IDD_DLGOPTIONS };

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support

	DECLARE_MESSAGE_MAP()
public:
	afx_msg void OnBnClickedButtonAuto();
	afx_msg void OnBnClickedButtonIdrive();
};
