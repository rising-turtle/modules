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


enum MouseDragMode {None, Zoom, Translate, Rotate};

typedef struct IoTRobot_GUIParam
{
	void *pPIPCallBack;
	void *pPCCallBack;
	void *pSLAMPCCallBack;
	void *pStateCallBack;
	void *pPathCallBack;
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

typedef struct IoTRobot_GUICMD
{
	int nSLAMCMD;
}IoTRobot_GUICMD;