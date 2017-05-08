#ifndef IOT_INTERNAL_DEFINE_H
#define IOT_INTERNAL_DEFINE_H


#pragma once



//include Openni head file
#include "XnCppWrapper.h"

//include PCL head file
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/common/time.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/openni_camera/openni_exception.h>
#include <pcl/pcl_config.h>
#include <pcl/io/openni_camera/openni_image.h>
#include <pcl/pcl_macros.h>


#define LOG_ARRAY_LEN 50
/*
#define RX 1200 // X [-6 6]
#define RY 400	// Y [-2 2]
#define RZ 1200 // Z [-6 6]
*/
#define RX 700 // X [-4 4]
#define RY 200	// Y [-2 2]
#define RZ 700 // Z [-4 4]

#define L_RX RX/200
#define L_RY RY/200
#define L_RZ RZ/200
#define S_RX RX/2
#define S_RY RY/2
#define S_RZ RZ/2

#define CELLSIZE 4
#define X_CELL RX/CELLSIZE // 200
#define Y_CELL RY/CELLSIZE // 100
#define Z_CELL RZ/CELLSIZE // 200

#define X_STEP Y_CELL*Z_CELL // 100*200
#define Y_STEP Z_CELL // 200

#define ALL_CELLS X_CELL*Y_CELL*Z_CELL // 



#define RGBD_DATA_BUFF_LEN 10
#define MAX_STORE_BUFF_LEN 2
#define RGB_BUFF_SRIDE 640*480*3
#define D_BUFF_SRIDE 640*480*2


#define MEMORY_ALLOCATION(Ptr,Type,Addr,Len,Count){ Ptr=(Type)Addr; Addr+=Len;Count+=Len;}

#define IOTGUI_SLAM_SIZE ALL_CELLS//19200000  //400*400*120

#define KERNEL_MSG_LEN 20

//define callback function 
typedef void (*OpenNICallBack_RGB24)(unsigned char *pucRGB24,void *pContext);
typedef void (*OpenNICallBack_XYZRGB)(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud,void *pContext);
typedef void (*OpenNICallBack_RGB)(const boost::shared_ptr<openni_wrapper::Image>& img,void *pContext);
typedef void (*OpenNICallBack_RGB24_Depth)(unsigned char *pucRGB24,unsigned short *pusDepth,void *pContext);
typedef void (*NetServerCallBack_SLAM_IMU)(unsigned char*pucRGB,unsigned short *pusDepth,char *pcIMUData,void *pContext);

typedef void (*CallBack_PIPQVGA)(unsigned char *pucQVGAImg,void *pContext);
typedef void (*CallBack_PointCloud)(unsigned char *pucImg,float *pfVertex,void *pContext);
typedef void (*CallBack_SLAM_PC)(unsigned char *pucImg,float *pfVertex,void *pContext,int nCount);
typedef void (*CallBack_RunState)(char *cState);
typedef void (*CallBack_Path)(float *pfTMat,float *pfRAngle,void *pContext);
typedef void (*CallBack_Session_Planes)(unsigned char* pucImg,float* pfVertex,int nCount);

typedef void (*NetServerCallBack_SLAMData)(unsigned char * pucSLAMData,void *pContext);
typedef void (*NetServerCallBack_Data2OpenGL)(unsigned char*pucRTPData,unsigned int uiDataLen,void *pContext);
typedef int (*SendCMD2Kernel)(void *pMessage);
typedef void (*NetServerCallBack_Data2Kernel)(unsigned char*pucData,unsigned int uiDataLen,void *pContext);
using namespace std;

typedef struct CallBackFuncSet
{
//	OpenNICallBack_RGB24 callBack_RGB24;
//	void *pRGB24Context;

	NetServerCallBack_SLAMData cbSLAM;
	void *pSLAMContext;

	NetServerCallBack_Data2OpenGL cdData2OpenGl;
	void *pData2OpenGlContext;

	NetServerCallBack_Data2Kernel cbGetDataFromNet;
	void *pGetDataFromNet;
	/*OpenNICallBack_XYZRGB callBack_XYZRGB;
	void *pXYZRGBContext;

	OpenNICallBack_RGB callBack_RGB;
	void *pRGBContext;

	OpenNICallBack_RGB24_Depth callBack_RGB24_Depth;
	void *pRGB24_DepthContext;

	NetServerCallBack_SLAM_IMU cbSlam_IMU;
	void *pSlamIMUContext;*/


}IoT_CallBackFuncSet;






typedef struct ClassPtrs
{
	void *pSLAM;
	void *pPointCloud;
	void *pMapBuilder;
	void *p3Dmap;
	void *pHostPotal;
}ClassPtrs;


struct RGBDData
{
	struct RGBDData *pstNext;
	unsigned char *pucRGB;
	unsigned short *pusDepth;
	double dIMUData[6];
	bool bReady;
	bool bUse;
	int nIdx;
};


typedef struct IoTRobot_GUICMD
{
	int nSLAMCMD;
}IoTRobot_GUICMD;


#define MSG_BUFF_SIZE 16

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


typedef struct IoTRobot_MSG
{
	IoTRobot_MSG *pstNext;
	IoTRobot_Message stContent;
	char cStateFlag;
}IoTRobot_MSG;

typedef struct IoTRobot_MSGArray
{
	IoTRobot_MSG *pstReadPos;
	IoTRobot_MSG *pstWritePos;
	IoTRobot_MSG stMsgArray[KERNEL_MSG_LEN];
}IoTRobot_MSGArray;

#define MSGARRAY_LEN 100

#define SLAM_MSG 0
#define MAP_BUILDER_MSG 1
#define DMAP_MSG 2
#define HOSTPOTAL_MSG 3
#define IDRIVE_MSG 4
#define CMDCTRL_MSG 5


#define GUI_MODULE -1
#define SLAM_MODULE 0
#define MAPBUILDER_MODULE 1
#define DMAP_MODULE 2
#define HOSTPORTAL_MODULE 3
#define IDRIVE_MODULE 4
#define CMDCTRL_MODULE 5

#define SLAM_MSG_IMU 0
#define SLAM_MSG_BUILD_MAP_STYLE 1
#define SLAM_MSG_SETTINGS 2



typedef struct IoTRobot_SLAMMSG
{
	int nApplyIMU;
	int nMapRebuild;
	int nMapBuilderStyle;
	int nSLAMSettingAddr;
}IoTRobot_SLAMMSG;



#define MAP_BUILDER_MSG_MSG1 0

typedef struct IoTRobot_MapBuilderMSG
{
	int nMsg1;
}IoTRobot_MapBuilderMSG;



#define DMAP_MSG_MSG1 0
typedef struct IoTRobot_3DMapMSG
{
	int nMsg1;
};

#define HOSTPORTAL_MSG_IDRIVE 0
#define HOSTPORTAL_MSG_AUTODRIVE 1

#define HOSTPORTAL_MSG_SPECIFIEDPAH  11
#define HOSTPORTAL_MSG_CTRL_CMD  12


#define HOSTPORTAL_MSG_EMERGENCYBREAK 21

#define HOSTPORTAL_MSG_RESETSESSION 31


#define IDRIVE_MSG_CTRL 0
#define IDRIVE_IP_INFO 1
#define IDRIVE_REALTIMEVA 2
#define IDRIVE_SWITCH 3


#define CMDCTRL_MSG_GETLASTSLAMPOS 0
#define CMDCTRL_MSG_RESETSESSION 1

#define REALTIME_RUN 1
#define REALTIME_STOP 0
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
#endif