/*
*@file      CommandAndControl.h
*@brief     master the intercommunication of command and control data between GUI module and SLAM/ControlDriver module. 
*           At the same time, take the responsibility for the storage of sensor/svent data and GUI settings 
*@auther    IoT songshuang2@lenovo.com
*@version 1.0
*@data 2011.9
*@Modification
*/

#pragma once
#include "SLAM.h"
#include "Storage.h"
#include "ControlDriver.h"
#include "MapBuilder.h"
#include "InternalDefine.h"
//#include "OpenNI.h"
#include "IoTRobot_GUI_Interface.h"
#include "Point_Cloud.h"
#include "3DMap.h"
#include "Log.h"
#include "HostPotal.h"
#include "IDrive.h"





class CCommandAndControl
{

public:
	///@brief 
	/// struct for Slam thread parameter
	typedef struct SLAMPARAM
	{
		CSLAM m_CSLAM;/// instance variable used in Slam thread
	}SLAMPARAM;
public:
	CCommandAndControl(void);
	~CCommandAndControl(void);


public:
	///@brief
	///CControlDriver module instantiation
	CControlDriver m_CCmdCtrl;

public:

	int CCInit();
	int CCRun();
	int CCUninit();
	

	
	///@brief
	///function used for getting the stored settings
	void GetSettings(int nPos,void *pSettings);

	///@brief
	///function used for store the new settings
	void StoreSettings(int nPos,void *pSettings);

	///@brief
	///function used for getting the stored sensor/event data
	void GetSensorAndEvent(int nPos,void *pSsrEvt);

	///@brief
	///function used for store the new stored sensor/event data
	void StoreSensorAndEvent(int nPos,void *pSsrEvt);


	///@brief
	/// external interface function used for pushing the control data into the massage array in control driver module
	void PushCtrlCmd(void *pMove);

	///@brief
	///function used for send the command to SLAM Module
	void SendCmd2SLAM(const IoTRobot_Message MSG);

	///@brief
	///function used for send the command to Map Builder Module
	void SendCmd2MB(const IoTRobot_Message MSG);

	///@brief
	///function used for send the command to 3DMap Module
	void SendCmd23Map(const IoTRobot_Message MSG);


	void SendCmd2HostPotal(const IoTRobot_Message MSG);

	void SendCmd2IDrive(const IoTRobot_Message MSG);


	void HandleCmdFromOtherModule(const IoTRobot_Message MSG);


	///@brief
	/// run SLAM
	void StartSLAM();


	///@brief
	/// 
	static UINT ThreadSLAM(LPVOID lpParam);


	///@brief
	///run CmdCtrl
	void CmdCtrlRun();


	///@brief
	//////////////////////////////////////////////////////////////////////////
	CMapBuilder m_CMapBuilder;
	CPoint_Cloud m_cPointCloud;
	C3DMap m_c3DMap;
	CIDrive m_CIDrive;


public:

	///@brief
	///function used for pushing the control data into the massage array in control driver module
	void PushCtrlData2CtrlDrv();


	///@brief
	///function used for sending necessary data back to GUI
	void SendData2GUI();


public:
	///@brief
	///CSLAM module instantiation
	CSLAM m_CSLAM;

	///@brief
	///CSettings storage module instantiation
	CSettings m_CSettings;

	///@brief
	///CSensorAndEvent storage module instantiation
	CSensorAndEvent m_CSsrEvt;

	///@brief
	///define the SLAMPARAM as a member variable
	SLAMPARAM m_stSlamParam;


	IoT_CallBackFuncSet m_stCallBackSet;
	//COpenNI m_cOpenNI;
	CHostPotal  m_CHostPotal;
	IoTRobot_GUI_Interface m_cGUI;


	ClassPtrs m_stClassPtrs;
	
	static UINT ThreadLog(LPVOID lpParam);
	static UINT ThreadListenCmdFromGUI(LPVOID lpParam);
	static UINT ThreadSendDeviceState2GUI(LPVOID lpParam);


	static UINT ThreadRunGUI(LPVOID lpParam);

	CLogfile m_cLog;


//Handle Message
	
	int InitMSGArray();
	int PumpMSG(IoTRobot_Message *pstMessage);

	static IoTRobot_MSGArray m_stMSGArray;
	static int AddMSG(const IoTRobot_Message *pstMessage);


	bool m_bSystemQuit;

	IoTRobot_MobileState m_stMobileState;

	static void CallBack_GetDataFromNet(unsigned char*pucData,unsigned int uiDataLen,void *pContext);

	 double m_sfLastPose[6];
	// get last CPose3D from CSLAM
	void GetLastPoseFromSlam();
	void ResetSession();
};
