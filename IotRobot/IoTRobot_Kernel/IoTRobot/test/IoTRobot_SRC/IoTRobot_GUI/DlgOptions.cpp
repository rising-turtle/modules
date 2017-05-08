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
	ON_BN_CLICKED(IDC_BUTTON2, &CDlgOptions::OnBnClickedButton2)
	ON_BN_CLICKED(IDC_BUTTON1, &CDlgOptions::OnBnClickedButton1)
	ON_BN_CLICKED(IDC_BUTTON3, &CDlgOptions::OnBnClickedButton3)
	ON_BN_CLICKED(IDC_BUTTON4, &CDlgOptions::OnBnClickedButton4)
	ON_LBN_SELCHANGE(IDC_LIST1, &CDlgOptions::OnLbnSelchangeList1)
END_MESSAGE_MAP()


// CDlgOptions message handlers

void CDlgOptions::OnBnClickedButton2()
{
	// TODO: Add your control notification handler code here
}

void CDlgOptions::OnBnClickedButton1()
{
	// TODO: Add your control notification handler code here
}

void CDlgOptions::OnBnClickedButton3()
{
	// TODO: Add your control notification handler code here
}

void CDlgOptions::OnBnClickedButton4()
{
	// TODO: Add your control notification handler code here
}

void CDlgOptions::OnLbnSelchangeList1()
{
	// TODO: Add your control notification handler code here
}
