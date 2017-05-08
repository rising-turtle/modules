#pragma once

#include <Winsock2.h>
#include <string.h>
#include <windows.h>
#include <stdio.h>


//#define MY_IP "10.0.0.5"
#define MY_IP "192.168.1.100"
#define MY_PORT 8000
#define BACKLOG 10
#define MAX_CLIENT_NUM 1


#define IOT_TMP_BUFF_SIZE 100
#define IOT_MEMORY_POOL_SIZE 1024*1024*10 //10M
#define IOT_RGBBUFF_SIZE 640*480*3
#define IOT_DEPTHBUFF_SIZE 640*480*2
#define IOT_QVGARGBBUFF_SIZE 320*240*3

#define  KINECT_NET_CONF_LEN 28
#define  RTP_MAX_PACK_SIZE 20000


#define JPEG_COMPRESS


#define MEMORY_ALLOCATION(Ptr,Type,Addr,Len,Count){ Ptr=(Type)Addr; Addr+=Len;Count+=Len;}

typedef void (*NetServerCallBack_SLAMData)(unsigned char * pucSLAMData,void *pContext);
typedef int (*SendCmd)(char *pcContent,int nLen,int nIdx,void *pContext);


typedef void (*NetServerCallBack_SLAM_IMU)(unsigned char*pucRGB,unsigned short *pusDepth,char *pcIMUData,void *pContext);
typedef void (*NetServerCallBack_SLAM)(unsigned char*pucRGB,unsigned short *pusDepth,void *pContext);
typedef void (*NetServerCallBack_Display)(unsigned char*pucRGB,void *pContext);
typedef void (*NetServerCallBack_RecvRTPData)(unsigned char*pucRTPData,unsigned int uiDataLen,void *pContext);

typedef void (*NetServerCallBack_Data2OpenGL)(unsigned char*pucRTPData,unsigned int uiDataLen,void *pContext);
typedef void (*NetServerCallBack_RunModeChange)(unsigned char *pucData,unsigned int uiDataLen,void *pContext);

typedef void (*NetServerCallBack_Data2Kernel)(unsigned char*pucData,unsigned int uiDataLen,void *pContext);

typedef struct IoT_NetCallBackSet
{
	NetServerCallBack_SLAMData cbSlam;
	void *pSlamContext;

	NetServerCallBack_Data2OpenGL cdData2OpenGl;
	void *pData2OpenGlContext;

	NetServerCallBack_Data2Kernel cbData2Kernel;
	void *pData2Kernel;
}IoT_NetCallBackSet;


typedef struct IoT_NetSendCMDFuncSet
{
	void *pSendCtlCMDContext;
	SendCmd scSendCtlCMD;
	void *pSendSysCMDContext;
	SendCmd scSendSysCMD;
}IoT_NetSendCMDFuncSet;


typedef struct IoT_NetDataPtrSet
{
	char *pcTmpBuff;
	char *pcTmpCmdBuff;
	char *pcTmpIDriveBuff;
	char *pcTmpSLAMBuff;
}IoT_NetDataPtrSet;

typedef struct IoTRobot_TCPIPParam
{
	BYTE ucClientIP[4];

	HANDLE hThreadCmd;
	HANDLE hThreadIDrive;
	HANDLE hThreadSLAM;


	struct sockaddr_in ClientAddr;
	struct sockaddr_in CmdAddr;
	struct sockaddr_in SLAMAddr;
	struct sockaddr_in IDriveAddr;


	int nCmdPort;
	int nSLAMPort;
	int nIDrivePort;
	int nClientRTPVideoPort;
	int nServerRTPVideoPort;
	int nClientRTPAudioPort;
	int nServerRTPAudioPort;

	int nSockCmd;
	int nSockSLAM;
	int nSockIDrive;

	int nIDriveNewFd;
	int nCmdNewFd;

	LPDWORD ID_Cmd;
	LPDWORD ID_SLAM;
	LPDWORD ID_IDrive;

}IoTRobot_TCPIPParam;


typedef struct IoTRobot_TCPIPParamSet
{
	IoTRobot_TCPIPParam stTCPIPParam[MAX_CLIENT_NUM];
	int nOccupyFlag[MAX_CLIENT_NUM];
}IoTRobot_TCPIPParamSet;








/*

typedef struct IoT_RobotInfo
{
	char cName[100];
	char cType[100];
	int nIdx;
}IoT_RobotInfo;


typedef struct IoT_NetClientInfo
{
	BYTE ucClientIP[4];
	HANDLE hThreadCmd;
	HANDLE hThreadIDrive;
	HANDLE hThreadSLAM;
	struct sockaddr_in ClientAddr;
	struct sockaddr_in CmdAddr;
	struct sockaddr_in SLAMAddr;
	struct sockaddr_in IDriveAddr;
	int nCmdPort;
	int nKinectPort;
	int nIDrivePort;

	
	HANDLE hThreadKinect;
	HANDLE hThreadIDrive;
	HANDLE hThreadRecvRealTimeVideo;
	HANDLE hThreadSendRealTimeVideo;

	IoT_RobotInfo stRobotInfo;
	struct sockaddr_in ClientAddr;

	int nCmdPort;
	int nKinectPort;
	int nIDrivePort;
	int nClientRTPVideoPort;
	int nServerRTPVideoPort;
	int nClientRTPAudioPort;
	int nServerRTPAudioPort;

	struct sockaddr_in CmdAddr;
	struct sockaddr_in KinectAddr;
	struct sockaddr_in IDriveAddr;

	int nSockCmd;
	int nSockKinect;
	int nSockIDrive;

	LPDWORD ID_Cmd;
	LPDWORD ID_Kinect;
	LPDWORD ID_IDrive;
	LPDWORD ID_RecvRealTimeVideo;
	LPDWORD ID_SendRealTimeVideo;

	int nCmdNewFd;
	int nKinectNewFd;
	int nIDriveNewFd;

	IoT_NetConf stNetConf;

}IoT_NetClientInfo;


typedef struct IoT_NetClientInfoSet
{
	IoT_NetClientInfo stNetClientInfo[MAX_CLIENT_NUM];
	int nOccupyFlag[MAX_CLIENT_NUM];
}IoT_NetClientInfoSet;











typedef struct IoT_KinectNetConf
{
	char cType;
	char cFrequency;
	char cCompression;
	char cResolution;

}IoT_KinectNetConf;


typedef struct IoT_NetConf
{
	IoT_KinectNetConf stSLAMConf;
	IoT_KinectNetConf stVisulizationConf;
}IoT_NetConf;

typedef struct IoT_NetDataPtrSet
{


	char cCurWrtRGBBuff;
	char cCurWrtDepthBuff;

	char cCurReadRGBBuff;
	char cCurReadDepthBuff;

	char *pTmpBuff;
	char *pTmpKinectBuff;
	char *pTmpCmdBuff;
	char *pTmpIDriveBuff;

	unsigned char *pucRGBBuffA;
	unsigned char *pucRGBBuffB;
	unsigned char *pucRGB;
	unsigned char *pucQVGARGB;

	unsigned short *pusDepthBuffA;
	unsigned short *pusDepthBuffB;
	unsigned short *pusDepth;

	unsigned char *pucCIFBuff;

	unsigned char *pucR;
	unsigned char *pucG;
	unsigned char *pucB;
	unsigned char *pucBGB24;

	unsigned char *pucEncodeBuff;

	char *pcSLAMData;
}IoT_NetDataPtrSet;
typedef void (*NetServerCallBack_SLAM_IMU)(unsigned char*pucRGB,unsigned short *pusDepth,char *pcIMUData,void *pContext);
typedef void (*NetServerCallBack_SLAM)(unsigned char*pucRGB,unsigned short *pusDepth,void *pContext);
typedef void (*NetServerCallBack_Display)(unsigned char*pucRGB,void *pContext);
typedef void (*NetServerCallBack_RecvRTPData)(unsigned char*pucRTPData,unsigned int uiDataLen,void *pContext);
*/