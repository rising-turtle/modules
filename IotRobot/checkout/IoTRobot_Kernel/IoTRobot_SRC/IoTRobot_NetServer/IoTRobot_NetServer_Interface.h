#pragma once
#define DLL_EXPORT __declspec(dllexport)
class DLL_EXPORT IoTRobot_NetServer_Interface
{
public:
	IoTRobot_NetServer_Interface(void);
	~IoTRobot_NetServer_Interface(void);

	int NetServer_Init(void * pCallBackSet,void *pSendCmdFuncSet);
	int NetServer_Run();
	int NetServer_Stop(); 
	int NetServer_Uninit();
	int NetServer_GetDeviceState(int *pnState);
	int NetServer_GetNSState(char *pData);
};
