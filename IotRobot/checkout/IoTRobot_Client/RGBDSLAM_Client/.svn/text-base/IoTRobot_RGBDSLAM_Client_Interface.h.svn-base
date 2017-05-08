//#ifndef IOT_ROBOT_RGBDSLAM_CLIENT_INTERFACE_H
//#define IOT_ROBOT_RGBDSLAM_CLIENT_INTERFACE_H
#pragma once
#define DLL_EXPORT __declspec(dllexport)

typedef void (*SLAMClientCallBack_OK)(unsigned char*pucData,int nDataLen,void *pContext);
typedef void (*SLAMClientCallBack_RGB24_Depth)(unsigned char*pucData,unsigned short *pusDepth,void *pContext);

typedef struct IoTRobot_SLAMClient_CBSet
{
	SLAMClientCallBack_OK cbSlam;
	void *pSLAMOKContext;
	SLAMClientCallBack_RGB24_Depth cbRGBD;
	void *pRGBDDataContext;
}IoTRobot_SLAMClient_CBSet;


#define WRONG_VALUE -999999
namespace IoTRobot_Client
{
	class DLL_EXPORT RGBDSLAM_Interface
	{
	public:
		RGBDSLAM_Interface();
		~RGBDSLAM_Interface();

		int RGBDSLAMInit(IoTRobot_SLAMClient_CBSet stCBset);
		int RGBDSLAMRun();
		int RGBDSLAMStop();
		int RGBDSLAMUnint();

		int RGBSLAMParamsSetting(void *pParam);

		int RGBDSLAMGetCurPos(double *pdPos);
	protected:
	private:
	};
}
//#endif