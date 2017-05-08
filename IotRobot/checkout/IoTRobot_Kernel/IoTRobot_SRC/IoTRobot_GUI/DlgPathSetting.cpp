// DlgPathSetting.cpp : implementation file
//

#include "stdafx.h"
#include "IoTRobot_GUI.h"
#include "DlgPathSetting.h"


// CDlgPathSetting dialog

IMPLEMENT_DYNAMIC(CDlgPathSetting, CDialog)

CDlgPathSetting::CDlgPathSetting(CWnd* pParent /*=NULL*/)
	: CDialog(CDlgPathSetting::IDD, pParent)
{

}

CDlgPathSetting::~CDlgPathSetting()
{
}

void CDlgPathSetting::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
}


BEGIN_MESSAGE_MAP(CDlgPathSetting, CDialog)
	ON_BN_CLICKED(IDC_RADIO_FULL_PATH, &CDlgPathSetting::OnBnClickedRadioFullPath)
	ON_BN_CLICKED(IDC_RADIO_CUSTOM_PATH, &CDlgPathSetting::OnBnClickedRadioCustomPath)
	ON_EN_CHANGE(IDC_EDIT_PATH_LENGTH, &CDlgPathSetting::OnEnChangeEditPathLength)
END_MESSAGE_MAP()


// CDlgPathSetting message handlers

void CDlgPathSetting::OnBnClickedRadioFullPath()
{

	m_bInfinitePath=true;
	m_pEditPathLength->EnableWindow(false);

	// TODO: Add your control notification handler code here
}

void CDlgPathSetting::OnBnClickedRadioCustomPath()
{
	m_bInfinitePath=false;
	m_pEditPathLength->EnableWindow(true);
	char string[25]; 
	itoa(m_nMaxPathLen,string,10);
	m_pEditPathLength->SetWindowText(string);
	// TODO: Add your control notification handler code here
}

void CDlgPathSetting::OnEnChangeEditPathLength()
{
	// TODO:  If this is a RICHEDIT control, the control will not
	// send this notification unless you override the CDialog::OnInitDialog()
	// function and call CRichEditCtrl().SetEventMask()
	// with the ENM_CHANGE flag ORed into the mask.

	// TODO:  Add your control notification handler code here





	CString str;
	int i;
	m_pEditPathLength->GetWindowText(str);
	int len=str.GetLength();
	char cTmp[100];
	memcpy(cTmp,str.GetBuffer(len),len);
	m_nMaxPathLen=atoi(cTmp);
}



BOOL CDlgPathSetting::OnInitDialog()
{
	CDialog::OnInitDialog();

	m_pFullPathRadio=(CButton*)GetDlgItem(IDC_RADIO_FULL_PATH);
	m_pCustomPathRadio=(CButton*)GetDlgItem(IDC_RADIO_CUSTOM_PATH);
	m_pEditPathLength=(CEdit *)GetDlgItem(IDC_EDIT_PATH_LENGTH);

	char string[25]; 
	itoa(m_nMaxPathLen,string,10);
	m_pEditPathLength->SetWindowText(string);


	if (m_bInfinitePath)
	{
		m_pFullPathRadio->SetCheck(true);
		m_pCustomPathRadio->SetCheck(false);
		m_pEditPathLength->EnableWindow(false);
	}
	else
	{
		m_pFullPathRadio->SetCheck(false);
		m_pCustomPathRadio->SetCheck(true);
		m_pEditPathLength->EnableWindow(true);

	}
	// TODO:  Add extra initialization here

	return TRUE;  // return TRUE unless you set the focus to a control
	// EXCEPTION: OCX Property Pages should return FALSE
}
