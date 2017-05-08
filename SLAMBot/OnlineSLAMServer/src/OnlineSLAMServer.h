#include "ExternalCom/ExternalCom.h"
#include "Configuration/ParseXML.h"
#include "Basic_Define.h"
#include <pthread.h>
#include "_3DRender/_3DRender.h"

class OnlineSLAMServer
{
public:
	OnlineSLAMServer();
	~OnlineSLAMServer();




	int OnlineSLAMServerInit();
	int OnlineSLAMServerRun();
	int OnlineSLAMServerUninit();

	ParseXML m_CParseXML;
	ExternalCom m_CExternalCom;
	_3DRender m_C3DRender;

	char m_cExternalComParams[1000];


	ClientInfo m_stClientInfo[SUPPORT_CLEINT_NUM];
	//int m_nRegisteredCLient[SUPPORT_CLEINT_NUM];
	static int RegisterCenter(int nSock,int nID,int nType,char *pcData,int nDataLen);
	static int HandleSLAMSycData(int nSock,int nID,char *pcData,int nDataLen);

	static int HandleSequenceFrame(char *pcData,int nID,int nSock);
	static int HandleSnapFrame(char *pcData,int nID,int nSock,int nType);

	static int RenderSnapFrame();
	static int SaveRecvFile(char *pcData,int nID,int nSock,int nType);
	static OnlineSLAMServer *m_pCOnlineSLAMServer;


	pthread_t m_hThreadRenderSnapFrame;
	bool m_bStopRenderSnapFrame;
	static void* ThreadRenderSnapFrame(void* lpParam);


	pthread_t m_hThread3DRender;
	bool m_bStop3DRender;
	static void* Thread3DRender(void* lpParam);


	int SendSnapCMD(void*pContent);
	int WaitingforSnapFramesSync(void*pContent);

	int m_nCurCncClientNum;
	int LoadConfFile(char *pcFilepath);
	char m_cServerIP[30];
	int m_nServerPort;

	pthread_t m_hThreadExternalComRun;
	static void* ThreadExternalComRun(void* lpParam);


	static bool m_bSnapSync[SUPPORT_CLEINT_NUM];
	static PCDData *m_pstPCDData[SUPPORT_CLEINT_NUM];
	static PosOffset m_stPosOffset[SUPPORT_CLEINT_NUM];

	static int m_nCurStatus[SUPPORT_CLEINT_NUM];
};
