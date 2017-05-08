// DlgRobotView.cpp : implementation file
//

#include "stdafx.h"
#include "IoTRobot_GUI.h"
#include "DlgRobotView.h"
#include <math.h>

// CDlgRobotView dialog

IMPLEMENT_DYNAMIC(CDlgRobotView, CDialog)

CDlgRobotView::CDlgRobotView(CWnd* pParent /*=NULL*/)
: CDialog(CDlgRobotView::IDD, pParent)
{
	m_bRobotViewRun=false;
	m_pucRobotView=new unsigned char[352*288*3];
}

CDlgRobotView::~CDlgRobotView()
{
	if (m_pucRobotView!=NULL)
	{
		delete [] m_pucRobotView;
		m_pucRobotView=NULL;
	}
}

void CDlgRobotView::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
}


BEGIN_MESSAGE_MAP(CDlgRobotView, CDialog)
	//	ON_WM_CLOSE()
	ON_WM_DESTROY()
	ON_STN_CLICKED(IDC_STATIC_ROBOT_VIEW_AREA, &CDlgRobotView::OnStnClickedStaticRobotViewArea)
	ON_WM_PAINT()
	ON_WM_MOVE()
	ON_WM_LBUTTONDBLCLK()
	ON_WM_MOUSEMOVE()
	ON_WM_LBUTTONDOWN()
	ON_WM_LBUTTONUP()
END_MESSAGE_MAP()


// CDlgRobotView message handlers

BOOL CDlgRobotView::OnInitDialog()
{
	CDialog::OnInitDialog();

	// TODO:  Add extra initialization here

	m_bRobotViewRun=true;
	m_hRobotView=0;
#ifdef RGB24
	m_RobotViewImgBMPInfo.bmiHeader.biBitCount=24;
	m_RobotViewImgBMPInfo.bmiHeader.biSize=sizeof(BITMAPINFO);
	m_RobotViewImgBMPInfo.bmiHeader.biSizeImage=352*288*4;
	m_RobotViewImgBMPInfo.bmiHeader.biWidth=352;
	m_RobotViewImgBMPInfo.bmiHeader.biHeight=-288;
	m_RobotViewImgBMPInfo.bmiHeader.biPlanes=1;
	m_RobotViewImgBMPInfo.bmiHeader.biCompression=BI_RGB;
#endif

#ifdef JPEG

	m_nImgHeight=QVGA_HEIGHT;
	m_nImgWidth=QVGA_WIDTH;
	m_RobotViewImgBMPInfo.bmiHeader.biBitCount=24;
	m_RobotViewImgBMPInfo.bmiHeader.biSize=sizeof(BITMAPINFO);
	m_RobotViewImgBMPInfo.bmiHeader.biWidth=m_nImgWidth;
	m_RobotViewImgBMPInfo.bmiHeader.biHeight=-m_nImgHeight;
	m_RobotViewImgBMPInfo.bmiHeader.biPlanes=1;
	m_RobotViewImgBMPInfo.bmiHeader.biCompression=BI_RGB;
	m_RobotViewImgBMPInfo.bmiHeader.biSizeImage=m_nImgWidth*m_nImgHeight*4;
#endif

	this->SetWindowPos(&wndTop,100,100,320,240,SWP_SHOWWINDOW);
	CStatic *pStaticPic=(CStatic *)GetDlgItem(IDC_STATIC_ROBOT_VIEW_AREA);
	pStaticPic->SetWindowPos(&wndTop,0,0,320,240,SWP_SHOWWINDOW);

	LPDWORD ID=0;
	m_hRobotView=CreateThread(NULL,0,(LPTHREAD_START_ROUTINE)ThreadDrawRobotView,this,0,ID);



	RECT rectStickArea;
	this->GetClientRect(&rectStickArea);


	m_nRadius1=100;
	m_nMiddleX=rectStickArea.right/2;
	m_nMiddleY=110;
	m_nRadius2=50;
	m_nRadius3=20;
	m_nRadius4=10;
	m_bStart2MoveStick=false;

	m_nStickX=m_nMiddleX;
	m_nStickY=m_nMiddleY;


	m_pApp=(void *)::AfxGetApp();


	m_TLCenter.x=m_nMiddleX-50;
	m_TLCenter.y=m_nMiddleY;

	m_TRCenter.x=m_nMiddleX+50;
	m_TRCenter.y=m_nMiddleY;


	m_MLCenter.x=m_nMiddleX-105;
	m_MLCenter.y=m_nMiddleY;


	m_MRCenter.x=m_nMiddleX+105;
	m_MRCenter.y=m_nMiddleY;


	m_FWCenter.x=m_nMiddleX;
	m_FWCenter.y=m_nMiddleY-50;

	m_BKCenter.x=m_nMiddleX;
	m_BKCenter.y=m_nMiddleY+50;



	//	m_bStopSendCtrlCmd=false;
	//	LPDWORD ID=0;
	return TRUE;  // return TRUE unless you set the focus to a control
	// EXCEPTION: OCX Property Pages should return FALSE
}


UINT CDlgRobotView::ThreadDrawRobotView(LPVOID lpParam)
{
	CDlgRobotView *pRobotView=(CDlgRobotView *)lpParam;
	CRect rect;
	char *pcTmp=new char[320*240*4];
	for (int i=0;i<320*240;i++)
	{
		pcTmp[i*4]=255;
	}
	CStatic *pStaticPic=(CStatic *)pRobotView->GetDlgItem(IDC_STATIC_ROBOT_VIEW_AREA);
	pStaticPic->GetClientRect(&rect);
	CDC *pDC = pStaticPic->GetDC();
	CBitmap Bitmap1;
	Bitmap1.CreateBitmap(320,240,1,24,NULL);
	pDC->SelectObject(&Bitmap1);

	while (pRobotView->m_bRobotViewRun)
	{
		int nWidth   =   320;   
		int nHeight   =   240;   

		int nCenterX=pRobotView->m_nMiddleX;
		int nCenterY=pRobotView->m_nMiddleY;

		CDC MemDC; // 定义一个内存显示设备对象 
		CBitmap MemBitmap; // 定义一个位图对象   

		CBrush GreenBrush(RGB(0,255,0));
		CBrush RedBrush(RGB(255,0,0));
		CBrush BlueBrush(RGB(0,0,255));
		//建立与屏幕显示兼容的内存显示设备     
		//pDC->SelectObject();
		MemDC.CreateCompatibleDC(pDC);       
		//建立一个与屏幕显示兼容的位图，位图的大小可选用窗口客户区的大小 
		MemBitmap.CreateCompatibleBitmap(pDC,nWidth,nHeight);  
		BITMAP bm;

		MemBitmap.GetObject(sizeof(bm), &bm);

		for (int i=0;i<320*240;i++)
		{
			pcTmp[i*4]=pRobotView->m_pucRobotView[i*3];
			pcTmp[i*4+1]=pRobotView->m_pucRobotView[i*3+1];
			pcTmp[i*4+2]=pRobotView->m_pucRobotView[i*3+2];
		}
		MemBitmap.SetBitmapBits(320*240*4,pcTmp);
		//将位图选入到内存显示设备中，只有选入了位图的内存显示设备才有地方绘图，画到指定的位图上    
		CBitmap *pOldBit = MemDC.SelectObject(&MemBitmap);       

		//turn left
		MemDC.SelectObject(GreenBrush);
		if (pRobotView->m_nCtrlCmd==1)
		{
			MemDC.SelectObject(RedBrush);
		}
		CPoint pt1[3] = {CPoint(nCenterX-65,nCenterY),CPoint(nCenterX-35,nCenterY-20), CPoint(nCenterX-35,nCenterY+20)};
		MemDC.Polygon(pt1,3);

		//turn left
		MemDC.SelectObject(GreenBrush);
		if (pRobotView->m_nCtrlCmd==2)
		{
			MemDC.SelectObject(RedBrush);
		}
		CPoint pt[3] = {CPoint(nCenterX+65,nCenterY),CPoint(nCenterX+35,nCenterY-20), CPoint(nCenterX+35,nCenterY+20)};
		MemDC.Polygon(pt,3);

		//move left

		MemDC.SelectObject(GreenBrush);
		if (pRobotView->m_nCtrlCmd==3)
		{
			MemDC.SelectObject(RedBrush);
		}
			CPoint pt2[3] = {CPoint(nCenterX-115,nCenterY),CPoint(nCenterX-100,nCenterY-20), CPoint(nCenterX-100,nCenterY+20)};
			MemDC.Polygon(pt2,3);

			CPoint pt3[3] = {CPoint(nCenterX-100,nCenterY),CPoint(nCenterX-85,nCenterY-20), CPoint(nCenterX-85,nCenterY+20)};
			MemDC.Polygon(pt3,3);
		

		//move right
		MemDC.SelectObject(GreenBrush);
		if (pRobotView->m_nCtrlCmd==4)
		{
			MemDC.SelectObject(RedBrush);
		}
		CPoint pt4[3] = {CPoint(nCenterX+115,nCenterY),CPoint(nCenterX+100,nCenterY-20), CPoint(nCenterX+100,nCenterY+20)};
		MemDC.Polygon(pt4,3);

		CPoint pt5[3] = {CPoint(nCenterX+100,nCenterY),CPoint(nCenterX+85,nCenterY-20), CPoint(nCenterX+85,nCenterY+20)};
		MemDC.Polygon(pt5,3);

		//back
		MemDC.SelectObject(GreenBrush);
		if (pRobotView->m_nCtrlCmd==6)
		{
			MemDC.SelectObject(RedBrush);
		}
		CPoint pt6[3] = {CPoint(nCenterX,nCenterY+65),CPoint(nCenterX-20,nCenterY+50), CPoint(nCenterX+20,nCenterY+50)};
		MemDC.Polygon(pt6,3);

		CPoint pt7[3] = {CPoint(nCenterX,nCenterY+50),CPoint(nCenterX-20,nCenterY+35), CPoint(nCenterX+20,nCenterY+35)};
		MemDC.Polygon(pt7,3);

		//forward
		MemDC.SelectObject(GreenBrush);
		if (pRobotView->m_nCtrlCmd==5)
		{
			MemDC.SelectObject(RedBrush);
		}

		CPoint pt8[3] = {CPoint(nCenterX,nCenterY-65),CPoint(nCenterX-20,nCenterY-50), CPoint(nCenterX+20,nCenterY-50)};
		MemDC.Polygon(pt8,3);

		CPoint pt9[3] = {CPoint(nCenterX,nCenterY-50),CPoint(nCenterX-20,nCenterY-35), CPoint(nCenterX+20,nCenterY-35)};
		MemDC.Polygon(pt9,3);

		//random in Gear 2
		if (pRobotView->m_nCtrlCmd==7)
		{
			MemDC.SelectObject(RedBrush);
			MemDC.Ellipse(pRobotView->m_nStickX-pRobotView->m_nRadius4,
				pRobotView->m_nStickY-pRobotView->m_nRadius4,
				pRobotView->m_nStickX+pRobotView->m_nRadius4,
				pRobotView->m_nStickY+pRobotView->m_nRadius4);//一个圆
		}

		if (pRobotView->m_nCtrlCmd==8)
		{
			MemDC.SelectObject(GreenBrush);
			MemDC.Ellipse(pRobotView->m_nStickX-pRobotView->m_nRadius4,
				pRobotView->m_nStickY-pRobotView->m_nRadius4,
				pRobotView->m_nStickX+pRobotView->m_nRadius4,
				pRobotView->m_nStickY+pRobotView->m_nRadius4);//一个圆
		}


		CBrush Gear2Brush(RGB(130,130,130));
		CPen Gear2Pen(PS_SOLID,1,RGB(0,0,0)); 

		CBrush *pBrush=CBrush::FromHandle((HBRUSH)GetStockObject(NULL_BRUSH));
		CBrush *pOldBrush=MemDC.SelectObject(pBrush);

		MemDC.MoveTo(40,pRobotView->m_nMiddleY);
		MemDC.LineTo(280,pRobotView->m_nMiddleY);

		MemDC.MoveTo(pRobotView->m_nMiddleX,5);
		MemDC.LineTo(pRobotView->m_nMiddleX,230);

		MemDC.SelectObject(Gear2Pen);
		MemDC.Ellipse(pRobotView->m_nMiddleX-pRobotView->m_nRadius1,pRobotView->m_nMiddleY-pRobotView->m_nRadius1,pRobotView->m_nMiddleX+pRobotView->m_nRadius1,pRobotView->m_nMiddleY+pRobotView->m_nRadius1);//一个圆
		MemDC.Ellipse(pRobotView->m_nMiddleX-pRobotView->m_nRadius2,pRobotView->m_nMiddleY-pRobotView->m_nRadius2,pRobotView->m_nMiddleX+pRobotView->m_nRadius2,pRobotView->m_nMiddleY+pRobotView->m_nRadius2);//一个圆



		pDC->BitBlt(0,0,nWidth,nHeight,&MemDC,0,0,SRCCOPY); 
		MemBitmap.DeleteObject();

		DeleteDC(MemDC);
		Sleep(25);
	}
	pStaticPic->ReleaseDC(pDC);
	return 0;
}
void CDlgRobotView::OnDestroy()
{

	CDialog::OnDestroy();
	m_bRobotViewRun=false;
	WaitForSingleObject(m_hRobotView,INFINITE);
	// TODO: Add your message handler code here
}

void CDlgRobotView::OnStnClickedStaticRobotViewArea()
{
	// TODO: Add your control notification handler code here
}

void CDlgRobotView::OnPaint()
{
	CPaintDC dc(this); // device context for painting

	/*	CRect   rc;    // 定义一个矩形区域变量
	GetClientRect(rc); 
	//	int nWidth   =   rc.Width();   
	//	int nHeight   =   rc.Height();   
	int nWidth   =   320;   
	int nHeight   =   240;   

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



	CBrush Gear2Brush(RGB(130,130,130));
	CPen Gear2Pen(PS_SOLID,1,RGB(0,0,0)); 

	CBrush *pBrush=CBrush::FromHandle((HBRUSH)GetStockObject(NULL_BRUSH));
	CBrush *pOldBrush=MemDC.SelectObject(pBrush);

	MemDC.MoveTo(40,m_nMiddleY);
	MemDC.LineTo(280,m_nMiddleY);

	MemDC.MoveTo(m_nMiddleX,5);
	MemDC.LineTo(m_nMiddleX,230);

	MemDC.SelectObject(Gear2Pen);
	MemDC.Ellipse(m_nMiddleX-m_nRadius1,m_nMiddleY-m_nRadius1,m_nMiddleX+m_nRadius1,m_nMiddleY+m_nRadius1);//一个圆
	MemDC.Ellipse(m_nMiddleX-m_nRadius2,m_nMiddleY-m_nRadius2,m_nMiddleX+m_nRadius2,m_nMiddleY+m_nRadius2);//一个圆

	MemDC.SelectObject(GreenBrush);
	//turn right
	if (m_nCtrlCmd==2)
	{
	MemDC.SelectObject(RedBrush);
	}
	CPoint pt[3] = {CPoint(m_nMiddleX+60,m_nMiddleY),CPoint(m_nMiddleX+40,m_nMiddleY-15), CPoint(m_nMiddleX+40,m_nMiddleY+15)};
	MemDC.Polygon(pt,3);

	MemDC.SelectObject(GreenBrush);
	if (m_nCtrlCmd==1)
	{
	MemDC.SelectObject(RedBrush);
	}
	//turn left
	CPoint pt1[3] = {CPoint(m_nMiddleX-60,m_nMiddleY),CPoint(m_nMiddleX-40,m_nMiddleY-15), CPoint(m_nMiddleX-40,m_nMiddleY+15)};
	MemDC.Polygon(pt1,3);


	//move left

	MemDC.SelectObject(GreenBrush);
	if (m_nCtrlCmd==3)
	{
	MemDC.SelectObject(RedBrush);
	}
	CPoint pt2[3] = {CPoint(m_nMiddleX-110,m_nMiddleY),CPoint(m_nMiddleX-100,m_nMiddleY-15), CPoint(m_nMiddleX-100,m_nMiddleY+15)};
	MemDC.Polygon(pt2,3);

	CPoint pt3[3] = {CPoint(m_nMiddleX-100,m_nMiddleY),CPoint(m_nMiddleX-90,m_nMiddleY-15), CPoint(m_nMiddleX-90,m_nMiddleY+15)};
	MemDC.Polygon(pt3,3);

	//move right
	MemDC.SelectObject(GreenBrush);
	if (m_nCtrlCmd==4)
	{
	MemDC.SelectObject(RedBrush);
	}
	CPoint pt4[3] = {CPoint(m_nMiddleX+110,m_nMiddleY),CPoint(m_nMiddleX+100,m_nMiddleY-15), CPoint(m_nMiddleX+100,m_nMiddleY+15)};
	MemDC.Polygon(pt4,3);

	CPoint pt5[3] = {CPoint(m_nMiddleX+100,m_nMiddleY),CPoint(m_nMiddleX+90,m_nMiddleY-15), CPoint(m_nMiddleX+90,m_nMiddleY+15)};
	MemDC.Polygon(pt5,3);

	//back
	MemDC.SelectObject(GreenBrush);
	if (m_nCtrlCmd==6)
	{
	MemDC.SelectObject(RedBrush);
	}
	CPoint pt6[3] = {CPoint(m_nMiddleX,m_nMiddleY+60),CPoint(m_nMiddleX-15,m_nMiddleY+50), CPoint(m_nMiddleX+15,m_nMiddleY+50)};
	MemDC.Polygon(pt6,3);

	CPoint pt7[3] = {CPoint(m_nMiddleX,m_nMiddleY+50),CPoint(m_nMiddleX-15,m_nMiddleY+40), CPoint(m_nMiddleX+15,m_nMiddleY+40)};
	MemDC.Polygon(pt7,3);

	//forward
	MemDC.SelectObject(GreenBrush);
	if (m_nCtrlCmd==5)
	{
	MemDC.SelectObject(RedBrush);
	}

	CPoint pt8[3] = {CPoint(m_nMiddleX,m_nMiddleY-60),CPoint(m_nMiddleX-15,m_nMiddleY-50), CPoint(m_nMiddleX+15,m_nMiddleY-50)};
	MemDC.Polygon(pt8,3);

	CPoint pt9[3] = {CPoint(m_nMiddleX,m_nMiddleY-50),CPoint(m_nMiddleX-15,m_nMiddleY-40), CPoint(m_nMiddleX+15,m_nMiddleY-40)};
	MemDC.Polygon(pt9,3);

	//random in Gear 2
	if (m_nCtrlCmd==7)
	{
	MemDC.SelectObject(RedBrush);
	MemDC.Ellipse(m_nStickX-m_nRadius3,m_nStickY-m_nRadius3,m_nStickX+m_nRadius3,m_nStickY+m_nRadius3);//一个圆
	}

	if (m_nCtrlCmd==8)
	{
	MemDC.SelectObject(GreenBrush);
	MemDC.Ellipse(m_nStickX-m_nRadius3,m_nStickY-m_nRadius3,m_nStickX+m_nRadius3,m_nStickY+m_nRadius3);//一个圆
	}

	MemDC.SelectObject(pBrush);*/



	/*MemDC.SelectObject(GreenBrush);
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
	}*/


	/*pDC->BitBlt(0,240,nWidth,nHeight,&MemDC,0,0,SRCCOPY); 
	MemBitmap.DeleteObject();

	this->ReleaseDC(pDC);*/
	// TODO: Add your message handler code here
	// Do not call CDialog::OnPaint() for painting messages
}

void CDlgRobotView::OnMove(int x, int y)
{
	CDialog::OnMove(x, y);


	// TODO: Add your message handler code here
}

void CDlgRobotView::OnLButtonDblClk(UINT nFlags, CPoint point)
{
	// TODO: Add your message handler code here and/or call default
	// TODO: Add your message handler code here and/or call default
	if (abs(point.x-m_nStickX)<m_nRadius3
		&&abs(point.y-240-m_nStickY)<m_nRadius3)
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

void CDlgRobotView::OnMouseMove(UINT nFlags, CPoint point)
{
	// TODO: Add your message handler code here and/or call default

	CDialog::OnMouseMove(nFlags, point);

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
		//Invalidate(false);

	}
}

void CDlgRobotView::OnLButtonDown(UINT nFlags, CPoint point)
{
	// TODO: Add your message handler code here and/or call default
	int nX=point.x;
	int nY=point.y;

	CIoTRobot_GUIApp *pApp=(CIoTRobot_GUIApp *)m_pApp;


	
	float fCurRa=(nX-m_nMiddleX)*(nX-m_nMiddleX)+(nY-m_nMiddleY)*(nY-m_nMiddleY);
	float fTmp;
	m_bStart2MoveStick=1;


	if ((nX-m_MLCenter.x)*(nX-m_MLCenter.x)+(nY-m_MLCenter.y)*(nY-m_MLCenter.y)<m_nRadius3*m_nRadius3)
	{
		m_nCtrlCmd=3;
		pApp->TransferCtrlCmd(180,1,0,1);
		}
	else if ((nX-m_MRCenter.x)*(nX-m_MRCenter.x)+(nY-m_MRCenter.y)*(nY-m_MRCenter.y)<m_nRadius3*m_nRadius3)
	{
		m_nCtrlCmd=4;
		pApp->TransferCtrlCmd(0,1,0,1);
	}	


	else if (fCurRa<m_nRadius1*m_nRadius1)
	{
		fTmp=(nX-m_TLCenter.x)*(nX-m_TLCenter.x)+(nY-m_TLCenter.y)*(nY-m_TLCenter.y);
		if (fTmp<m_nRadius3*m_nRadius3)
		{
			m_nCtrlCmd=1;
			pApp->TransferCtrlCmd(0,0,-15,1);
		}
		else if ((nX-m_TRCenter.x)*(nX-m_TRCenter.x)+(nY-m_TRCenter.y)*(nY-m_TRCenter.y)<m_nRadius3*m_nRadius3)
		{
			m_nCtrlCmd=2;
			pApp->TransferCtrlCmd(0,0,15,1);
		}
		
		else if ((nX-m_FWCenter.x)*(nX-m_FWCenter.x)+(nY-m_FWCenter.y)*(nY-m_FWCenter.y)<m_nRadius3*m_nRadius3)
		{
			m_nCtrlCmd=5;
			pApp->TransferCtrlCmd(90,2,0,1);
		}
		else if ((nX-m_BKCenter.x)*(nX-m_BKCenter.x)+(nY-m_BKCenter.y)*(nY-m_BKCenter.y)<m_nRadius3*m_nRadius3)
		{
			m_nCtrlCmd=6;
			pApp->TransferCtrlCmd(270,2,0,1);
		}
	/*	else
		{
			float fXdiff=point.x-m_nMiddleX,fYDiff=point.y-m_nMiddleY;
			float fNewRadius=sqrt((float)fXdiff*fXdiff+(float)fYDiff*fYDiff);
			float fThres=m_nRadius1-m_nRadius3-1;
			float fSinVal=(float)fYDiff/fNewRadius;
			int nAngle,nGear;
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

			if (fCurRa>m_nRadius2*m_nRadius2)
			{

				
				m_nCtrlCmd=7;
				m_nStickX=nX;
				m_nStickY=nY;

				float fRotTime;

				int nRotAngle;


				if (m_nAngle>=90&&m_nAngle<=270)
				{
					//fRotTime=fabs((m_nAngle-90)*0.013888*1000);
					nRotAngle=m_nAngle-90;
					pApp->TransferCtrlCmd(0,0,-nRotAngle,2);
					Sleep(6);
					
				}
				else
				{
					if (m_nAngle>=0&&m_nAngle<90)
					{
						//fRotTime=fabs((90-m_nAngle)*0.013888*1000);
						nRotAngle=m_nAngle-90;
						
					}
					else
					{
						//fRotTime=fabs((450-m_nAngle)*0.013888*1000);
						nRotAngle=m_nAngle-450;

						
					}

					
					pApp->TransferCtrlCmd(0,0,-nRotAngle,2);
					Sleep(6);
				}

				printf("rotate angle  :%d   \n",nRotAngle);
				
			//	Sleep(fRotTime);

			//	pApp->TransferCtrlCmd(0,0,0,1);
			//	Sleep(1);
				pApp->TransferCtrlCmd(90,2,0,1);
				Sleep(1);
			}
			else 
			{
				m_nCtrlCmd=8;
				m_nStickX=nX;
				m_nStickY=nY;

				float fRotTime;
				int nRotAngle;

				if (m_nAngle>=90&&m_nAngle<=270)
				{
					//fRotTime=fabs((m_nAngle-90)*0.013888*1000);
					nRotAngle=m_nAngle-90;
					pApp->TransferCtrlCmd(0,0,-nRotAngle,2);
					Sleep(6);
				}
				else
				{
					if (m_nAngle>=0&&m_nAngle<90)
					{
						//fRotTime=fabs((90-m_nAngle)*0.013888*1000);
						nRotAngle=m_nAngle-90;
					}
					else
					{
						//fRotTime=fabs((450-m_nAngle)*0.013888*1000);
						nRotAngle=m_nAngle-450;
					}
					pApp->TransferCtrlCmd(0,0,-nRotAngle,2);
					Sleep(6);
				}
				pApp->TransferCtrlCmd(90,1,0,1);
				Sleep(1);
			}

		}*/

		
	}
	CDialog::OnLButtonDown(nFlags, point);
}

void CDlgRobotView::OnLButtonUp(UINT nFlags, CPoint point)
{
	// TODO: Add your message handler code here and/or call default
	m_bStart2MoveStick=false;
	m_nCtrlCmd=0;
	CIoTRobot_GUIApp *pApp=(CIoTRobot_GUIApp *)m_pApp;
	pApp->TransferCtrlCmd(0,0,0,1);
//	Invalidate(false);
	CDialog::OnLButtonUp(nFlags, point);
}
