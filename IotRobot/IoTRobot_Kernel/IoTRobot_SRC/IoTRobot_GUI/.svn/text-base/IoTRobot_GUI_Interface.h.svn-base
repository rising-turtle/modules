#pragma once
#define DLL_EXPORT __declspec(dllexport)




class DLL_EXPORT IoTRobot_GUI_Interface
{
public:
	IoTRobot_GUI_Interface(void);
	~IoTRobot_GUI_Interface(void);

	IoTRobot_GUICMD m_stGUICMD;

	int IoTRobot_GUI_Init(void *pCallback,void *pContext);
	int IoTRobot_GUI_Run();
	int IoTRobot_GUI_Uninit();
	void * IoTRobot_GUI_Init();
	void * IoTRobot_GUI_Init(void *pCMDFunc);

	int IoTRobot_GUI_UpdateMobileState(void *pMobileState);

};
