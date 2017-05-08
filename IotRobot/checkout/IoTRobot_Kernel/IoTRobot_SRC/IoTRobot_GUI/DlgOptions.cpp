// DlgOptions.cpp : implementation file
//

#include "stdafx.h"
#include "IoTRobot_GUI.h"
#include "DlgOptions.h"


// CDlgOptions dialog

IMPLEMENT_DYNAMIC(CDlgOptions, CDialog)

CDlgOptions::CDlgOptions(CWnd* pParent /*=NULL*/)
	: CDialog(CDlgOptions::IDD, pParent)
{

}

CDlgOptions::~CDlgOptions()
{
}

void CDlgOptions::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
}


BEGIN_MESSAGE_MAP(CDlgOptions, CDialog)
	ON_BN_CLICKED(IDC_BUTTON_AUTO, &CDlgOptions::OnBnClickedButtonAuto)
	ON_BN_CLICKED(IDC_BUTTON_IDRIVE, &CDlgOptions::OnBnClickedButtonIdrive)
END_MESSAGE_MAP()


// CDlgOptions message handlers

void CDlgOptions::OnBnClickedButtonAuto()
{
	// TODO: Add your control notification handler code here
	CIoTRobot_GUIApp *pApp=(CIoTRobot_GUIApp *)::AfxGetApp();
	pApp->OnAutoDrive();
}

void CDlgOptions::OnBnClickedButtonIdrive()
{	
	CIoTRobot_GUIApp *pApp=(CIoTRobot_GUIApp *)::AfxGetApp();
	pApp->OnIDrive();
	// TODO: Add your control notification handler code here
}
