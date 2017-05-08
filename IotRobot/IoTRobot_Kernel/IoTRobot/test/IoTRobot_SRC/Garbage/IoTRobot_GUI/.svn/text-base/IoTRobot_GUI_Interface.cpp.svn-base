#include "StdAfx.h"
#include "IoTRobot_GUI_Interface.h"
#include "IoTRobot_GUI.h"





IoTRobot_GUI_Interface::IoTRobot_GUI_Interface(void)
{
}

IoTRobot_GUI_Interface::~IoTRobot_GUI_Interface(void)
{
}

int IoTRobot_GUI_Interface::IoTRobot_GUI_Init(void *pCallback, void *pContext)
{

	//pCallback=(void*)CIoTRobot_GUIApp::CallBack_RGB;
	//pCallback=(void*)CIoTRobot_GUIApp::CallBack_RGB24;
	//pCallback=(void*)CIoTRobot_GUIApp::CallBack_XYZRGB;
	theApp.AppRun();
	return 1;
}

int IoTRobot_GUI_Interface::IoTRobot_GUI_Run()
{
	
	
	//theApp.m_bRun=true;
	theApp.AppMessageLoop();

	return 1;
}

void *IoTRobot_GUI_Interface::IoTRobot_GUI_Init()
{
	//return (void*)CIoTRobot_GUIApp::CallBack_RGB24;
	
	theApp.AppRun();
	theApp.AppGetCB();
	return (void*)&theApp.m_stInterfaceParam;
}

int IoTRobot_GUI_Interface::IoTRobot_GUI_Uninit()
{
	return 1;
}