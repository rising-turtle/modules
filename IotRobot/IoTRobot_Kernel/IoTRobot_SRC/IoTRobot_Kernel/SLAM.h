












#pragma once

#include "afxmt.h"
#include "InternalDefine.h"
#include "CPose3D.h"

class CSLAM
{
public:
	CSLAM();
	CSLAM(void *pCCmdCtrl);
	~CSLAM();
	void SLAMInit(IoT_CallBackFuncSet *pstCallBackFuncSet);
	void SLAMRun();
	void SLAMStop();
	void SLAMUninit();




	static void Callback_SLAM(unsigned char* pucSLAMData,void *pContext);
	static UINT ThreadRGBDSlam(LPVOID lpParam);


	void NewCommand(const IoTRobot_Message MSG);
	
	ClassPtrs m_stClassPtrs;

	CallBack_RunState m_cbState;

	bool m_bManualMap;
	bool m_bUpdateManually;

	//boost::mutex lock_to_write;
	bool rebuild_cells;
	vector<CPose3D> robotpath_update; // NOT USED
	//vector<int> id_of_node;			  
	 boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > m_SLAMPC;
	 vector<int> m_id_of_node2;
	 vector<vector<float> >m_matrix_of_nodeF;
	bool m_bReadyforMB;
	IoTRobot_SLAMMSG m_stSLAMSG;
	LPDWORD ID_RGBDSlam;
	HANDLE m_hThreadRGBDSlam;
	bool m_bStopSlam;

	// For Reset
	void Reset();
};




/*#pragma once
#define ALGORITHM_NUM 4
#include "afxmt.h"
#include "InternalDefine.h"
#include "IoTRobot_RGBDSLAM_Interface.h"
//#include "afxmt.h"

//#include "CommandAndControl.h"
//#include "Point_Cloud.h"
//#include "OpenNI.h"
//#include "Point_Cloud.h"

class CPose3D;

class CSLAM
{
private:
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


	void getImagesandDepthMetaData(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> >&  point_cloud,
		unsigned char* rgbbuf,unsigned short* depthbuf);
	void NewCommand(const IoTRobot_Message MSG);
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


	// determine whether to rebuild the whole map
	bool rebuild_cells;
	vector<CPose3D> robotpath_update;
	vector<int> id_of_node;

public:
	void *m_pCCmdCtrl;
	IoTRobot_RGBDSLAM_Interfaace m_cRGBDSlam;
	static bool m_bFrameReady;
	static bool m_bFinishFrame;
	static unsigned char *m_pucRGBBuff;
	static unsigned short *m_pusDepth;
	static double *m_pdIMUData;
	
	static  int m_nCount;
	static unsigned int m_uiFrameNum;


	static int m_nCheckIMU;
	IoTRobot_SLAMMSG m_stSLAMSG;

	// decide whether to manually update Map
	bool m_bManualMap;
	bool m_bUpdateManually;

	//boost::mutex lock_to_write;
	static boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > m_SLAMPC;
	static vector<int> m_id_of_node2;
	static vector<vector<float> >m_matrix_of_nodeF;
	static bool m_bReadyforMB;
};*/

//static void CallBack_SLAM_IMU(unsigned char*pucRGB,unsigned short *pusDepth,
//	char *pcIMUData,void *pContext);