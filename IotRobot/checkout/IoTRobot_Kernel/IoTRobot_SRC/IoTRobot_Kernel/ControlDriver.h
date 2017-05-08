#pragma once
#define MSGARRAY_LEN 100





class CControlDriver
{
private:
	typedef struct CtrlDrv_MSG
	{
		CtrlDrv_MSG *pstNext;
		char cContent;
		char cStateFlag;
	}CtrlDrv_MSG;

	typedef struct CtrlDrv_MSGArray
	{
		CtrlDrv_MSG *pstReadPos;
		CtrlDrv_MSG *pstWritePos;
		CtrlDrv_MSG stMsgArray[MSGARRAY_LEN];
	}CtrlDrv_MSGArray;


public:
	CControlDriver(void);
	CControlDriver(void *p_CCmdCtrl);
	~CControlDriver(void);

public:
	int AddMsg(char *pcContent);
private:

	CtrlDrv_MSGArray m_stMsgArray;
	void CreateMsgArray();
	
	int DrawMsg(char *pcContent);
	void MsgArrayTest();


	void *mp_CCmdCtrl;
	void SendSsrEvt2CC(void *pSsrEvt);
	void SendCtrlInfo2WIFI(void *pCtrlInfo);
};


