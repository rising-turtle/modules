#pragma once

#define RTN_OK 0
#define RTN_ERR_READFILE -1
#define RTN_ADD_PUB_ERR -100
#define RTN_ADD_SUB_ERR -200

#define NET_STATUS_OK_BIT_NUM 5
#define NET_STATUS_NORMAL_BIT_NUM 3
#define NET_STATUS_SLOW_BIT_NUM 1
#define NET_STATUS_UNKOWN_BIT_NUM 0
#define NET_STATUS_INIT_NUM 10

typedef int (*CallBack_StandarProtocal)(int nDataLen,char *pcData);

#define TPC_RTC_W2R "Topic_WebRTC_W2R"
#define TPC_RTC_R2W "Topic_WebRTC_R2W"
#define TPC_CMD_R2W "Topic_CMD_R2W"
#define TPC_CMD_W2R "Topic_CMD_W2R"
#define TPC_DATA_R2W "Topic_Data_R2W"
#define TPC_DATA_W2R "Topic_Data_W2R"

#define MSG_CMD "CMD"
#define MSG_STARTRTC "StartRTC"
#define MSG_CONTENT "Content"
#define MSG_ENDFLAG "EndFlag"
#define MSG_ANSWER "answer"
#define MSG_SDP "sdpMLineIndex"
#define MSG_SDPMID "sdpMid"
#define MSG_WEBRTC "WebRTC"
#define MSG_CANDIDATE "Candidate"

#define CMD_WEBRTC "WebRTC"
#define CMD_SESSIONDESCRIPTION "sessionDescription"
#define CMD_CANDIDATE "Candidate"
#define CMD_CMD2ROBOT "CMD2Robot"
#define CMD_DATA2ROBOT "Data2Robot"
#define CMD_CMD "CMD"
#define CMD_WebGL "WebGL"
#define CMD_WEBRTCRUN "WebRTCRun"
#define CMD_WEBRTCSTOP "WebRTCStop"

#define CMD_LEFT "Left"
#define CMD_RIGHT "Right"
#define CMD_FORWARD "Forward"
#define CMD_BACKWARD "Backward"
#define CMD_STOP "Stop"
#define CMD_IDRIVE "IDrive"
#define CMD_AUTODRIVE "AutoDrive"
#define CMD_PATH "Path"

#define CMD_WEBRTCRDY "WebRTCReady"
#define CMD_WEBRTCLOST "WebRTCLost"
//ROS
#include "std_msgs/String.h"
#include "std_msgs/ByteMultiArray.h"
#include "ros/ros.h"

extern ros::NodeHandle * g_pCNodeHandle;

typedef struct PosOffset
{
	int nID;
	float fOffsetParams[4];
	int nStatus;
}PosOffset;

typedef struct PCDData
{
	unsigned char ucR;
	unsigned char ucG;
	unsigned char ucB;
	float fX;
	float fY;
	float fZ;
}PCDData;

typedef struct RegisterInfo
{
	int nID;
	float fOffsetParams[4];
}RegisterInfo;

typedef struct ClientInfo
{
	int nSock;
	int nCLientID;
	bool bHasCnc;
	RegisterInfo stRegisterInfo;
}ClientInfo;




#define SUPPORT_CLEINT_NUM 6
