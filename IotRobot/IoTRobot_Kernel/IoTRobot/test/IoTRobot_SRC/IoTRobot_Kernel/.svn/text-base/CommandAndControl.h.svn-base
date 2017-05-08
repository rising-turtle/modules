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
#include "OpenNI.h"
#include "IoTRobot_GUI_Interface.h"
#include "Point_Cloud.h"
#include "3DMap.h"
#include "Log.h"





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
	void SendCmd2SLAM(char *pCmd);


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
	COpenNI m_cOpenNI;
	IoTRobot_GUI_Interface m_cGUI;


	ClassPtrs m_stClassPtrs;
	
	static UINT ThreadLog(LPVOID lpParam);
	static UINT ThreadListenCmdFromGUI(LPVOID lpParam);

	CLogfile m_cLog;
};
