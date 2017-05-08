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
	theApp.AppRun(&m_stGUICMD);
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
	theApp.AppRun(&m_stGUICMD);
	theApp.AppGetCB();
	return (void*)&theApp.m_stInterfaceParam;
}


void *IoTRobot_GUI_Interface::IoTRobot_GUI_Init(void *pCMDFunc)
{

	theApp.AppRun(&m_stGUICMD);
	theApp.AppGetCB();
	theApp.m_pCSendCMD2Kernel=(SendCMD2Kernel)pCMDFunc;
	return (void*)&theApp.m_stInterfaceParam;
}


int IoTRobot_GUI_Interface::IoTRobot_GUI_UpdateMobileState(void *pMobileState)
{
	theApp.UpdateMobileState((IoTRobot_MobileState*)pMobileState);
	return 0;
}
int IoTRobot_GUI_Interface::IoTRobot_GUI_Uninit()
{
	return 1;
}