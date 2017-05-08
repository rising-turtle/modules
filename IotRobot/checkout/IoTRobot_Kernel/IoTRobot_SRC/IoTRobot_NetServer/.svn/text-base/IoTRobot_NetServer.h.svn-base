#pragma once
//#define DLL_EXPORT __declspec(dllexport)
#include "IoTRobot_NetServer_Define.h"
#include "IoTRobot_NetProtocol.h"
//#include "./Encode/IoTRobot_Encode_H263.h"
//#include "./Decode/IoTRobot_Decode_H263.h"
//#include "RealTimeVA/IoTRobot_USBCamera.h"
//#include "RealTimeVA/jpegClass.h"
//#include "RealTimeVA/IoTRobot_RTP.h"

//#include "RealTimeVA/IoTRobot_RealTimeVideo.h"

//#include "RealTimeVA/IoTRobot_RealTimeVA.h"



class IoTRobot_NetServer
{

public:
	


public:
	IoTRobot_NetServer(void);
	~IoTRobot_NetServer(void);


	int NetServer_Init(IoT_NetCallBackSet stCallBackSet,IoT_NetSendCMDFuncSet *pstSendCMDFuncSet);
	int NetServer_Run();
	int NetServer_Uninit();
	int NetServer_Stop();
	int NetServer_GetDeviceState(int *pnState);

	int m_nCnCNum;

	//TCP/IP Param Set
	IoTRobot_TCPIPParamSet m_stTCPIPParamSet;
private:
	//handle the connection request from Mobile termination or Robot termination
	int AnwserRequests(int nSockFD,BYTE *pucClientIP);
	int AnwserMobileRequests(int nSockFD,BYTE *pucClientIP);
	int AnwserRobotRequests(int nSockFD,BYTE *pucClientIP);

	//TCP/IP Recv and Send data
	int RcvStream(int nGoalSize,char *pcTmpBuff,int sockfd);
	int SendStream(int nGoalSize,char *pcTmpBuff,int sockfd);

	//Get the Valid TCP/IP Connect Position 
	IoTRobot_TCPIPParam * GetValidCIPos();


	//Memory Pool
	char *m_pcMemoryPool;

	//Data Ptr Set
	IoT_NetDataPtrSet m_stDataPtrSet;

	//Thread function
	static UINT ThreadSLAMSock(LPVOID lpParam);
	static UINT ThreadCmdSock(LPVOID lpParam);
	static UINT ThreadIDriveSock(LPVOID lpParam);
	int CreateThreadForEachSock(IoTRobot_TCPIPParam * pstTCPIPParam);

	//Get Host IP
	int GetHostIP(BYTE *ucMyIP);
	BYTE m_ucHostIP[4];




	//IMU Status
	int m_nIMUState;
	
	//Callback Set
	IoT_NetCallBackSet m_stCallBackSet;

	//Put here temporarily
	static int SendSysCmd(char *pcContent,int nLen,int nIdx,void *pContext);
	static int SendCtlCmd(char *pcContent,int nLen,int nIdx,void *pContext);

	int m_nSockCmd;
	struct sockaddr_in m_MyAddr;

	//Thread Switch
	bool m_bStopListen;
	bool m_bStopThreadSLAMSock;
	bool m_bStopThreadCmdSock;
	bool m_bStopThreadIDriveSock;

	static UINT ThreadAnwserRequests(LPVOID lpParam);

	//NetServer Status
	


	static void *m_pNetServer;

/*	int m_nSockCmd;
	int m_nSockRGB;
	int m_nSockDepth;

	char *m_pcMemoryPool;
	IoT_NetDataPtrSet m_stDataPtrSet;

	struct sockaddr_in m_MyAddr;


	IoT_NetClientInfoSet m_stNetCleintInfoSet;
	bool m_bStopListen;
	int NetServer_Init(IoT_NetCallBackSet stCallBackSet);
	int NetServer_Init(IoT_NetCallBackSet stCallBackSet,IoT_NetSendCMDFuncSet *pstSendCMDFuncSet);



	int NetServer_Run();
	int NetServer_Uninit();
	int NetServer_Stop();
	int NetServer_Conf(int nClientIdx,IoT_NetConf stNetConf);
	int NetServer_GetOneSlamFrame(int nClientIdx);

	int NetServer_GetDeviceState(int *pnState);

	char m_cKinectConf[KINECT_NET_CONF_LEN];
	bool m_bNewKinectConf;
	bool m_bGetOneSlamFrame;


//	IoT_NetConf m_stNetConf;


	int AnwserClientRequests(int nSockFD,BYTE *pucClientIP);
	int AnwserMobileRequests(int nSockFD,BYTE *pucClientIP);
	int AnwserRobotRequests(int nSockFD,BYTE *pucClientIP);
	int AnwserRequests(int nSockFD,BYTE *pucClientIP);




	static int RcvStream(int nGoalSize,char *pcTmpBuff,int sockfd);
	int SendStream(int nGoalSize,char *pcTmpBuff,int sockfd);

	IoT_NetClientInfo * GetValidCIPos();
	int OpenThreadForClient(IoT_NetClientInfo * pstCI);

	static UINT ThreadKinectSock(LPVOID lpParam);
	static UINT ThreadCmdSock(LPVOID lpParam);
	static UINT ThreadIDriveSock(LPVOID lpParam);
	static UINT ThreadRecvRealTimeVideo(LPVOID lpParam);
	static UINT ThreadSendRealTimeVideo(LPVOID lpParam);
	static UINT ThreadKinectSendCmd(LPVOID lpParam);

	int VGA2QVGA(unsigned char *pucVGA,unsigned char *pucQVGA);

	IoT_NetCallBackSet m_stCallBackSet;
	int GetHostIP(BYTE *ucMyIP);
	BYTE m_ucHostIP[4];


	IoTRobot_RTP m_CRTP;

#ifdef H263_COMPRESS
	IoTRobot_Encode_H263 m_CH263Encode;
	static IoTRobot_Decode_H263 m_CH263Decode;
#endif

#ifdef JPEG_COMPRESS
	jpegClass m_CJPEG;
#endif


	bool m_bStopSendRealTimeVideo;
	bool m_bStopRecvRealTimeVideo;

	static void CallBack_RTPData(unsigned char*pucRTPData,unsigned int uiDataLen,void *pContext);
	IoTRobot_USBCamera m_CUSBCam;
	static void CallBack_USBCamera(unsigned char *pucImgData,void *pContext);
	int m_nIMUState;


	static int SendSysCmd(char *pcContent,int nLen,int nIdx,void *pContext);
	static int SendCtlCmd(char *pcContent,int nLen,int nIdx,void *pContext);


	int ChangeRunMode(int nIdx,int nMode);
	int m_nRunMode;
	//IoTRobot_RealTimeVideo m_CRTV;
	IoTRobot_RealTimeVA m_CRTVA;


	NetServerCallBack m_cRegisterCallBack;
	//int NetServer_Init(NetServerCallBack cCallBack);*/
};
