#ifndef IOT_IDRIVE_H
#define IOT_IDRIVE_H
#include "InternalDefine.h"
#include "RealTimeVA.h/CRealTimeVA.h"
class CIDrive
{
public:
	CIDrive();
	~CIDrive();

	int IDriveInit();
	int IDriveRun();
	int IDriveStop();
	int IDriveUninit();

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

	int AddMsg(char *pcContent);
	CtrlDrv_MSGArray m_stMsgArray;
	void CreateMsgArray();

	int DrawMsg(char *pcContent);

	void NewCommand(const IoTRobot_Message MSG);
	int SendCtrlCmd2Robot();

	IoTRobot_RTP_Param  m_stRTPParam;
	int m_nRobotCnc;
	int m_nRealTimeHasInit;

	CRealTimeVA m_CRealTimeVA;
	int m_nStopRealTimeVA;
	HANDLE m_hThreadRTVA;
	IoTRobot_RealTimeVA_VideoParam m_stRealTimeVA_VideoParam;
	IoTRobot_RealTimeVA_AudioParam m_stRealTimeVA_AudioParam;
	static UINT ThreadRealTimeVA(LPVOID lpParam);
	CallBack_PIPQVGA m_cbPIPQVGA;


	int m_nRunIDrive;
};
#endif