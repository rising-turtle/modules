#pragma once
#define IOTGUI_PC_WIDTH 640
#define IOTGUI_PC_HEIGHT 480
#define IOTGUI_PC_SIZE 307200
#define IOTGUI_PC_DOUBLESIZE 614400
#define IOTGUI_PC_TRIBLESIZE 921600



#define IOTGUI_PIP_WIDTH 320
#define IOTGUI_PIP_HEIGHT 240
#define IOTGUI_PIP_SIZE 76800
#define IOTGUI_PIP_DOUBLESIZE 153600
#define IOTGUI_PIP_TRIBLESIZE 230400


#define IOTGUI_PIP_WIDTH_CIF 352
#define IOTGUI_PIP_HEIGHT_CIF 288
#define IOTGUI_PIP_SIZE_CIF 352*288
#define IOTGUI_PIP_DOUBLESIZE_CIF 352*288*2
#define IOTGUI_PIP_TRIBLESIZE_CIF 352*288*3


#define IOTGUI_SLAM_SIZE 19200000  //400*400*120
#define IOTGUI_SLAM_TRIBLESIZE 57600000  //400*400*120
#define IOTGUI_SLAM_SIXTHSIZE 230400000  //400*400*120


#define X_RATE 0.001875
#define Y_RATE 0.001875
#define CIRCLE_POINTS 72
#define CIRCLE_RADIUS 1

#define MAX_PATH_NODE_LEN 50
#define MAX_TWINKLE_TIME 1000
#define MAX_TWINKLE_INTERVAL 4


#define GUI_MODULE -1
#define SLAM_MODULE 0
#define MAPBUILDER_MODULE 1
#define DMAP_MODULE 2
#define HOSTPORTAL_MODULE 3
#define IDRIVE_MSG 4
#define CMDCTRL_MODULE 5


#define QVGA
//#define CIF


enum MouseDragMode {None, Zoom, Translate, Rotate};

typedef struct IoTRobot_GUIParam
{
	void *pPIPCallBack;
	void *pPCCallBack;
	void *pSLAMPCCallBack;
	void *pStateCallBack;
	void *pPathCallBack;
	void *pData2OpenGL;
	void *pPlaneCallBack;
}IoTRobot_GUIParam;

typedef struct IoTRobot_GLPathNode
{
	float fTMat[3];
	float fRAngle[3];
	int nIdx;
	IoTRobot_GLPathNode *pstSonNode;
	IoTRobot_GLPathNode *pstParentNode;
}IoTRobot_GLPathNode;


typedef struct IoTRobot_GLPath
{
	IoTRobot_GLPathNode *pstPathHead;
	IoTRobot_GLPathNode *pstPathTail;
	IoTRobot_GLPathNode *pstPathCur;

	int nCurNodeLen;
}IoTRobot_GLPath;


typedef struct IoTRobot_PathTwinkleParam
{
	DWORD ulStartTime;
	float fLastPos[3];
	float fCurPos[3];
	float fCurColor[3];
	int nColorCount;
}IoTRobot_PathTwinkleParam;



typedef struct IoTRobot_SLAMCMD
{
	char cOpenIMU;
	char cRebuildMap;
	int nSLAMSettingAddr;
}IoTRobot_SLAMCMD;

typedef struct IoTRobot_MBCMD
{
	char cMsg1;
}IoTRobot_MBCMD;


typedef struct IoTRobot_3DMapCMD
{
	char cMsg1;
}IoTRobot_3DMapCMD;

typedef struct IoTRobot_GUICMD
{
	IoTRobot_SLAMCMD stSLAMCMD;
	IoTRobot_MBCMD stMBCMD;
	IoTRobot_3DMapCMD st3DMapCMD;
}IoTRobot_GUICMD;



typedef struct IoTRobot_Message
{
	char cFromModule;
	char cDestModule;
	char cCommand;
	char cPriority;

	int nParam1;
	int nParam2;
	char *pcParam3;
}IoTRobot_Message;

typedef struct IoTRobot_DeviceState
{
	char cIMU;
	char cKinect;
	char cMotor;
	char cKeep;
}IoTRobot_DeviceState;

typedef struct IoTRobot_MobileState
{
	char cType;  //0  init state(config)  1 run state
	IoTRobot_DeviceState stDeviceState;
}IoTRobot_MobileState;



typedef struct IoTRobot_BarButtons
{
	bool bOpenStatusLight;
	bool bOpenPath;
	bool bOpenPIP;
	bool bOpenCompass;
	bool bOpenTriangulation;


	bool bOpenManualMB;
	bool bOpenSLAMSetting;
	bool bOpenIMU;

	bool bOpenMessageBox;
	bool bOpenOptionButtons;
} IoTRobot_BarButtons;


#define CMDCTRL_MSG_GETLASTSLAMPOS 0
#define CMDCTRL_MSG_RESETSESSION 1

#define SLAM_MSG_IMU 0
#define SLAM_MSG_REBUILD_MAP 1
#define SLAM_MSG_SETTINGS 2


#define HOSTPORTAL_MSG_IDRIVE 0
#define HOSTPORTAL_MSG_AUTODRIVE 1

#define HOSTPORTAL_MSG_SPECIFIEDPAH  11


#define HOSTPORTAL_MSG_EMERGENCYBREAK 21
#define HOSTPORTAL_MSG_RESETSESSION 31
typedef int (*SendCMD2Kernel)(void *pMessage);





#define IDRIVE_MODE  1
#define AUTO_DRIVE 2