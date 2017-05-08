// DlgEditRouteConfirm.cpp : implementation file
//

#include "stdafx.h"
#include "IoTRobot_GUI.h"
#include "DlgEditRouteConfirm.h"


// DlgEditRouteConfirm dialog

IMPLEMENT_DYNAMIC(CDlgEditRouteConfirm, CDialog)

CDlgEditRouteConfirm::CDlgEditRouteConfirm(CWnd* pParent /*=NULL*/)
	: CDialog(CDlgEditRouteConfirm::IDD, pParent)
{

}

CDlgEditRouteConfirm::~CDlgEditRouteConfirm()
{
}

void CDlgEditRouteConfirm::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
}


BEGIN_MESSAGE_MAP(CDlgEditRouteConfirm, CDialog)
	ON_BN_CLICKED(IDOK, &CDlgEditRouteConfirm::OnBnClickedOk)
	ON_BN_CLICKED(IDOK2, &CDlgEditRouteConfirm::OnBnClickedOk2)
	ON_BN_CLICKED(IDCANCEL, &CDlgEditRouteConfirm::OnBnClickedCancel)
END_MESSAGE_MAP()


// DlgEditRouteConfirm message handlers

void CDlgEditRouteConfirm::OnBnClickedOk()
{
	// TODO: Add your control notification handler code here
	OnOK();
	m_nConfirmRslt=1;
}

void CDlgEditRouteConfirm::OnBnClickedOk2()
{
	m_nConfirmRslt=2;
	OnBnClickedCancel();
	// TODO: Add your control notification handler code here
}

void CDlgEditRouteConfirm::OnBnClickedCancel()
{
	// TODO: Add your control notification handler code here
	OnCancel();
}
