#pragma once


// CDlgPathSetting dialog

class CDlgPathSetting : public CDialog
{
	DECLARE_DYNAMIC(CDlgPathSetting)

public:
	CDlgPathSetting(CWnd* pParent = NULL);   // standard constructor
	virtual ~CDlgPathSetting();

// Dialog Data
	enum { IDD = IDD_DIALOG_PATH_LENGTH_SETTING };

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support

	DECLARE_MESSAGE_MAP()
public:
	afx_msg void OnBnClickedRadioFullPath();
	afx_msg void OnBnClickedRadioCustomPath();
	afx_msg void OnEnChangeEditPathLength();


	bool m_bInfinitePath;
	int m_nMaxPathLen;

	CButton *m_pFullPathRadio;
	CButton *m_pCustomPathRadio;
	CEdit *m_pEditPathLength;
//	virtual BOOL Create(LPCTSTR lpszTemplateName, CWnd* pParentWnd = NULL);
	virtual BOOL OnInitDialog();
};
