#pragma once


// CDlgRobotView dialog
#define JPEG
#define QVGA_WIDTH 320
#define QVGA_HEIGHT 240

//#define RGB24

class CDlgRobotView : public CDialog
{
	DECLARE_DYNAMIC(CDlgRobotView)

public:
	CDlgRobotView(CWnd* pParent = NULL);   // standard constructor
	virtual ~CDlgRobotView();

// Dialog Data
	enum { IDD = IDD_DIALOG_ROBOT_VIEW };

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support

	DECLARE_MESSAGE_MAP()
public:

	bool m_bRobotViewRun;
	HANDLE m_hRobotView;
	virtual BOOL OnInitDialog();
//	afx_msg void OnClose();
	afx_msg void OnDestroy();

	static UINT ThreadDrawRobotView(LPVOID lpParam);
	BITMAPINFO m_RobotViewImgBMPInfo;
	unsigned char *m_pucRobotView;
	afx_msg void OnStnClickedStaticRobotViewArea();

	int m_nImgWidth,m_nImgHeight;
	int m_nDataLen;
	afx_msg void OnPaint();


	int m_bStart2MoveStick;
	int m_nStickX,m_nStickY;
	int m_nMiddleX,m_nMiddleY;
	int m_nRadius1,m_nRadius2,m_nRadius3,m_nRadius4;
	int m_nGear,m_nAngle;

	CPoint  m_TLCenter;
	CPoint  m_TRCenter;
	CPoint  m_MLCenter;
	CPoint  m_MRCenter;
	CPoint  m_FWCenter;
	CPoint  m_BKCenter;

	void *m_pApp;
	afx_msg void OnMove(int x, int y);
	afx_msg void OnLButtonDblClk(UINT nFlags, CPoint point);
	afx_msg void OnMouseMove(UINT nFlags, CPoint point);
	afx_msg void OnLButtonDown(UINT nFlags, CPoint point);
	afx_msg void OnLButtonUp(UINT nFlags, CPoint point);

	int m_nCtrlCmd;
};
