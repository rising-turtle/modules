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



#define RX 1200 // X [-6 6]
#define RY 400	// Y [-2 2]
#define RZ 1200 // Z [-6 6]

#define CELLSIZE 4
#define X_CELL RX/CELLSIZE // 400
#define Y_CELL RY/CELLSIZE // 120
#define Z_CELL RZ/CELLSIZE // 400

#define X_STEP Y_CELL*Z_CELL // 120*400
#define Y_STEP Z_CELL // 400

#define ALL_CELLS X_CELL*Y_CELL*Z_CELL // 



#define RGBD_DATA_BUFF_LEN 10
#define MAX_STORE_BUFF_LEN 2
#define RGB_BUFF_SRIDE 640*480*3
#define D_BUFF_SRIDE 640*480*2


#define MEMORY_ALLOCATION(Ptr,Type,Addr,Len,Count){ Ptr=(Type)Addr; Addr+=Len;Count+=Len;}

#define IOTGUI_SLAM_SIZE 19200000  //400*400*120

//define callback function 
typedef void (*OpenNICallBack_RGB24)(unsigned char *pucRGB24,void *pContext);
typedef void (*OpenNICallBack_XYZRGB)(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud,void *pContext);
typedef void (*OpenNICallBack_RGB)(const boost::shared_ptr<openni_wrapper::Image>& img,void *pContext);
typedef void (*OpenNICallBack_RGB24_Depth)(unsigned char *pucRGB24,unsigned short *pusDepth,void *pContext);
typedef void (*NetServerCallBack_SLAM_IMU)(unsigned char*pucRGB,unsigned short *pusDepth,char *pcIMUData,void *pContext);

typedef void (*CallBack_PIPQVGA)(unsigned char *pucQVGAImg,void *pContext);
typedef void (*CallBack_PointCloud)(unsigned char *pucImg,float *pfVertex,void *pContext);
typedef void (*CallBack_SLAM_PC)(unsigned char *pucImg,float *pfVertex,void *pContext,int nCount);
typedef void (*CallBack_RunState)(char cState);
typedef void (*CallBack_Path)(float *pfTMat,float *pfRAngle,void *pContext);
using namespace std;

typedef struct CallBackFuncSet
{
	OpenNICallBack_RGB24 callBack_RGB24;
	void *pRGB24Context;

	OpenNICallBack_XYZRGB callBack_XYZRGB;
	void *pXYZRGBContext;

	OpenNICallBack_RGB callBack_RGB;
	void *pRGBContext;

	OpenNICallBack_RGB24_Depth callBack_RGB24_Depth;
	void *pRGB24_DepthContext;

	NetServerCallBack_SLAM_IMU cbSlam_IMU;
	void *pSlamIMUContext;
}IoT_CallBackFuncSet;



typedef struct ClassPtrs
{
	void *pSLAM;
	void *pPointCloud;
	void *pMapBuilder;
	void *p3Dmap;
	void *pOpenNi;
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

#endif