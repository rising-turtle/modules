// DlgControlStick.cpp : implementation file
//

#include "stdafx.h"
#include "IoTRobot_GUI.h"
#include "DlgControlStick.h"
#include <math.h>
#include "IoTRobot_GUI.h"

// CDlgControlStick dialog

IMPLEMENT_DYNAMIC(CDlgControlStick, CDialog)

CDlgControlStick::CDlgControlStick(CWnd* pParent /*=NULL*/)
	: CDialog(CDlgControlStick::IDD, pParent)
{
	m_hThreadSendCtrlCmd=0;
}

CDlgControlStick::~CDlgControlStick()
{
}

void CDlgControlStick::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
}


BEGIN_MESSAGE_MAP(CDlgControlStick, CDialog)
	ON_WM_PAINT()
	ON_WM_LBUTTONDOWN()
	ON_WM_LBUTTONUP()
	ON_WM_MOUSEMOVE()
	ON_WM_ERASEBKGND()
	ON_BN_CLICKED(IDOK, &CDlgControlStick::OnBnClickedOk)
	ON_BN_CLICKED(IDCANCEL, &CDlgControlStick::OnBnClickedCancel)
	ON_WM_LBUTTONDBLCLK()
END_MESSAGE_MAP()


// CDlgControlStick message handlers

BOOL CDlgControlStick::OnInitDialog()
{
	CDialog::OnInitDialog();

	// TODO:  Add extra initialization here
	RECT rectStickArea;
	this->GetClientRect(&rectStickArea);

	if (rectStickArea.right>=rectStickArea.bottom)
	{
		m_nRadius1=rectStickArea.bottom/2;
	}
	else
	{
		m_nRadius1=rectStickArea.right/2;
	}

	m_nMiddleX=rectStickArea.right/2;
	m_nMiddleY=rectStickArea.bottom/2;
	m_nRadius2=m_nRadius1/7*4;
	m_nRadius3=m_nRadius1/14*3;
	m_bStart2MoveStick=false;

	m_nStickX=m_nMiddleX;
	m_nStickY=m_nMiddleY;


	m_pApp=(void *)::AfxGetApp();
	m_bStopSendCtrlCmd=false;
	LPDWORD ID=0;
	//m_hThreadSendCtrlCmd=CreateThread(NULL,0,(LPTHREAD_START_ROUTINE)ThreadSendCtrlCmd,this,0,ID);
	return TRUE;  // return TRUE unless you set the focus to a control
	// EXCEPTION: OCX Property Pages should return FALSE
}

void CDlgControlStick::OnPaint()
{
	CRect   rc;    // 定义一个矩形区域变量
	GetClientRect(rc); 
	int nWidth   =   rc.Width();   
	int nHeight   =   rc.Height();   

	CDC   *pDC = GetDC();    // 定义设备上下文
	CDC MemDC; // 定义一个内存显示设备对象 
	CBitmap MemBitmap; // 定义一个位图对象   

	CBrush GreenBrush(RGB(0,255,0));
	CBrush RedBrush(RGB(255,0,0));
	CBrush BlueBrush(RGB(0,0,255));
	//建立与屏幕显示兼容的内存显示设备     
	MemDC.CreateCompatibleDC(pDC);       
	//建立一个与屏幕显示兼容的位图，位图的大小可选用窗口客户区的大小 
	MemBitmap.CreateCompatibleBitmap(pDC,nWidth,nHeight);   
	//将位图选入到内存显示设备中，只有选入了位图的内存显示设备才有地方绘图，画到指定的位图上    
	CBitmap *pOldBit = MemDC.SelectObject(&MemBitmap);       
	//先用背景色将位图清除干净，否则是黑色。这里用的是白色作为背景   
	MemDC.FillSolidRect(0,0,nWidth,nHeight,RGB(255,255,255)); 

	//绘图操作等在这里实现  
	//	MemDC.MoveTo(……); 
	//	MemDC.LineTo(……);   
	CBrush Gear2Brush(RGB(130,130,130));
	MemDC.SelectObject(Gear2Brush);
	MemDC.Ellipse(m_nMiddleX-m_nRadius1,m_nMiddleY-m_nRadius1,m_nMiddleX+m_nRadius1,m_nMiddleY+m_nRadius1);//一个圆

	CBrush Gear1Brush(RGB(170,170,170));
	MemDC.SelectObject(Gear1Brush);
	MemDC.Ellipse(m_nMiddleX-m_nRadius2,m_nMiddleY-m_nRadius2,m_nMiddleX+m_nRadius2,m_nMiddleY+m_nRadius2);//一个圆

	MemDC.SelectObject(GreenBrush);
	{
		if (m_nGear==1)
		{
			
			MemDC.SelectObject(GreenBrush);
		}
		else if (m_nGear==2)
		{
			
			MemDC.SelectObject(BlueBrush);
		}
		else if (m_nGear==0)
		{
			MemDC.SelectObject(RedBrush);
		}
	
		MemDC.Ellipse(m_nStickX-m_nRadius3,m_nStickY-m_nRadius3,m_nStickX+m_nRadius3,m_nStickY+m_nRadius3);//一个圆
	}
	pDC->BitBlt(0,0,nWidth,nHeight,&MemDC,0,0,SRCCOPY); 
	MemBitmap.DeleteObject();

	this->ReleaseDC(pDC);


	// Do not call CDialog::OnPaint() for painting messages
}

void CDlgControlStick::OnLButtonDown(UINT nFlags, CPoint point)
{
	// TODO: Add your message handler code here and/or call default
	/*if (abs(point.x-m_nMiddleX)<m_nRadius3
		&&abs(point.y-m_nMiddleY)<m_nRadius3)
	{
		m_bStart2MoveStick=true;
		m_nStickX=point.x;
		m_nStickY=point.y;
	}*/

	if (abs(point.x-m_nStickX)<m_nRadius3
		&&abs(point.y-m_nStickY)<m_nRadius3)
	{
		m_bStart2MoveStick=true;
		m_nStickX=point.x;
		m_nStickY=point.y;
	}
	Invalidate(false);

	CDialog::OnLButtonDown(nFlags, point);
}

void CDlgControlStick::OnLButtonUp(UINT nFlags, CPoint point)
{
	// TODO: Add your message handler code here and/or call default
	m_bStart2MoveStick=false;
	CIoTRobot_GUIApp *pApp=(CIoTRobot_GUIApp *)m_pApp;
	pApp->TransferCtrlCmd(m_nAngle,m_nGear,0,1);
	Invalidate(false);
	CDialog::OnLButtonUp(nFlags, point);
}

void CDlgControlStick::OnMouseMove(UINT nFlags, CPoint point)
{
	// TODO: Add your message handler code here and/or call default
	// TODO: Add your message handler code here and/or call default
	int nAngle;
	if (m_bStart2MoveStick)
	{
		float fXdiff=point.x-m_nMiddleX,fYDiff=point.y-m_nMiddleY;
		float fNewRadius=sqrt((float)fXdiff*fXdiff+(float)fYDiff*fYDiff);
		float fThres=m_nRadius1-m_nRadius3-1;
		float fSinVal=(float)fYDiff/fNewRadius;

		//printf("nYDiff: %f \n",fYDiff);
		//printf("nNewRadius: %f \n",fNewRadius);
		//printf("fSinVal: %f \n",fSinVal);
		nAngle=-asin(fSinVal)*180/3.1415926;

		if (fXdiff>=0&&fYDiff>=0)
		{
			nAngle=360+nAngle;
		}
		else if (fXdiff<0&&fYDiff>=0)
		{
			nAngle=180-nAngle;
		}
		else if (fXdiff<0&&fYDiff<0)
		{
			nAngle=180-nAngle;
		}
		else
		{
			
		}
		if (nAngle==360)
		{
			m_nAngle=0;
		}
		else
		{
			m_nAngle=nAngle;
		}
		//printf("x :%d ,y :%d\n",fXdiff,fYDiff);
		//printf("angle :%d \n",nAngle);
		if (fNewRadius<m_nRadius2)
		{
			m_nGear=1;
		}
		else
		{
			m_nGear=2;
		}


		if (fNewRadius>fThres)
		{
			m_nStickX=(float)fThres/fNewRadius*(point.x-m_nMiddleX)+m_nMiddleX;
			m_nStickY=(float)fThres/fNewRadius*(point.y-m_nMiddleY)+m_nMiddleY;
		}
		else
		{
			m_nStickX=point.x;
			m_nStickY=point.y;
		}
		Invalidate(false);

	}
	CDialog::OnMouseMove(nFlags, point);
}

BOOL CDlgControlStick::OnEraseBkgnd(CDC* pDC)
{
	// TODO: Add your message handler code here and/or call default

	return true;
}


UINT CDlgControlStick::ThreadSendCtrlCmd(LPVOID lpParam)
{
	CDlgControlStick *pControlStick=(CDlgControlStick *)lpParam;
	CIoTRobot_GUIApp *pApp=(CIoTRobot_GUIApp *)pControlStick->m_pApp;
	while (!pControlStick->m_bStopSendCtrlCmd)
	{
		if (pControlStick->m_bStart2MoveStick)
		{
			pApp->TransferCtrlCmd(pControlStick->m_nAngle,pControlStick->m_nGear,0,1);
		}
		Sleep(10);
	}
	return 0;
}
void CDlgControlStick::OnBnClickedOk()
{
	// TODO: Add your control notification handler code here
	m_bStopSendCtrlCmd=true;
	if (m_hThreadSendCtrlCmd!=0)
	{
		WaitForSingleObject(m_hThreadSendCtrlCmd,INFINITE);
		m_hThreadSendCtrlCmd=0;
	}
	OnOK();
}

void CDlgControlStick::OnBnClickedCancel()
{
	// TODO: Add your control notification handler code here
	m_bStopSendCtrlCmd=true;
	if (m_hThreadSendCtrlCmd!=0)
	{
		WaitForSingleObject(m_hThreadSendCtrlCmd,INFINITE);
		m_hThreadSendCtrlCmd=0;
	}
	OnCancel();
}


//Stop Robot
void CDlgControlStick::OnLButtonDblClk(UINT nFlags, CPoint point)
{
	// TODO: Add your message handler code here and/or call default
	if (abs(point.x-m_nStickX)<m_nRadius3
		&&abs(point.y-m_nStickY)<m_nRadius3)
	{
		m_bStart2MoveStick=false;
		m_nStickX=m_nMiddleX;
		m_nStickY=m_nMiddleY;
		m_nGear=0;
		m_nAngle=0;
		CIoTRobot_GUIApp *pApp=(CIoTRobot_GUIApp *)m_pApp;
		pApp->TransferCtrlCmd(0,0,0,1);
	}
	Invalidate(false);
	CDialog::OnLButtonDblClk(nFlags, point);
}
