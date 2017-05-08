// DlgBar.cpp : implementation file
//

#include "stdafx.h"
#include "IoTRobot_GUI.h"
#include "DlgBar.h"


// DlgBar dialog

IMPLEMENT_DYNAMIC(DlgBar, CDialog)

DlgBar::DlgBar(CWnd* pParent /*=NULL*/)
	: CDialog(DlgBar::IDD, pParent)
{

}

DlgBar::~DlgBar()
{
}

void DlgBar::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
}


BEGIN_MESSAGE_MAP(DlgBar, CDialog)
	ON_BN_CLICKED(IDC_BUTTON2, &DlgBar::OnBnClickedButton2)
	ON_BN_CLICKED(IDC_BUTTON1, &DlgBar::OnBnClickedButton1)
END_MESSAGE_MAP()


// DlgBar message handlers

void DlgBar::OnBnClickedButton2()
{
	// TODO: Add your control notification handler code here
}

void DlgBar::OnBnClickedButton1()
{
	// TODO: Add your control notification handler code here
}
