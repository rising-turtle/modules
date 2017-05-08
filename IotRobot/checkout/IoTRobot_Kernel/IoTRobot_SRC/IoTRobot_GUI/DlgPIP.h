#pragma once


// CDlgPIP dialog

class CDlgPIP : public CDialog
{
	DECLARE_DYNAMIC(CDlgPIP)

public:
	CDlgPIP(CWnd* pParent = NULL);   // standard constructor
	virtual ~CDlgPIP();

// Dialog Data
	enum { IDD = IDD_DIALOG_PIP };

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support

	DECLARE_MESSAGE_MAP()
};
