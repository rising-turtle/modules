#pragma once
#define ALGORITHM_NUM 4

#include "InternalDefine.h"
#include "IoTRobot_RGBDSLAM_Interface.h"
//#include "OpenNI.h"
//#include "Point_Cloud.h"



class CSLAM
{
private:
	typedef struct SLAM_AlgorithmOpt
	{
		char cNewCmdFlag;
		char cAlgorithmFlag[ALGORITHM_NUM];
	}SLAM_AlgorithmOpt;
public:
	CSLAM();
	CSLAM(void *pCCmdCtrl);
	~CSLAM();

	
	
	static void CallBack_XYZRGB(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud,void *pContext);
	static void CallBack_RGB24_Depth(unsigned char *pucRGB24,unsigned short *pusDepth,void *pContext);
	static void CallBack_SLAM_IMU(unsigned char*pucRGB,unsigned short *pusDepth,
		char *pcIMUData,void *pContext);
	static void CallBack_RGB24_Depth_Dynamic(unsigned char *pucRGB24,unsigned short *pusDepth,void *pContext);

	static CallBack_RunState m_cCallBackState;

	void SLAMInit(IoT_CallBackFuncSet *pstCallBackFuncSet);
	void SLAMRun();
	void SLAMUninit();


	void NewCommand(char *pCmd);
	void GetState(char *pCmd);
	

	static UINT ThreadRGBDSlam(LPVOID lpParam);
	ClassPtrs m_stClassPtrs;


	static UINT ThreadRGBDDynamicManage(LPVOID lpParam);
	static UINT ThreadRGBDDynamicReceive(LPVOID lpParam);


	struct RGBDData *m_pstRGBDDataListHead;
	struct RGBDData *m_pstRGBDDataCurWrite;
	struct RGBDData *m_pstRGBDDataCurRead;
	struct RGBDData *m_pstRGBDDataReady;


	int CreateRGBDDataBuffList(int nLen);
	int DeleteRGBDDataBuffList(int nLen);
	bool m_bStart2Get;
	bool m_bIWanaOneFrame;
	int m_nLeftBuffLen;
	int m_nFirstFrame;

	LPDWORD ID_RGBDSlam,ID_RGBDDynamicManage,ID_RGBDDynamicReceive;

	CallBack_PointCloud m_cbPointCould;

	//static unsigned char *m_pucImg;
	float *m_pfVertex;
	int TestPC(unsigned short *psDepth,float *pfPC);

	static int m_nApplyIMU;
	static int m_nApplyIMU2;
private:
	void *m_pCCmdCtrl;
	SLAM_AlgorithmOpt m_stAlgOpt;
	IoTRobot_RGBDSLAM_Interfaace m_cRGBDSlam;
	static bool m_bFrameReady;
	static bool m_bFinishFrame;
	static unsigned char *m_pucRGBBuff;
	static unsigned short *m_pusDepth;
	static double *m_pdIMUData;
	
	static  int m_nCount;
	static unsigned int m_uiFrameNum;


	static int m_nCheckIMU;
	
	
};