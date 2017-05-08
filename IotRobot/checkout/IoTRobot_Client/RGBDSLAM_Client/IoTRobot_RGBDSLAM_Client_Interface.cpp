#include "IoTRobot_RGBDSLAM_Client_Interface.h"
#include "IoTRobot_RGBDSLAM_Client.h"

using namespace IoTRobot_Client;

IoTRobot_RGBDSLAM_Client *g_pCRGBDSLAMClient=NULL;
RGBDSLAM_Interface::RGBDSLAM_Interface()
{

}

RGBDSLAM_Interface::~RGBDSLAM_Interface()
{

}

int RGBDSLAM_Interface::RGBDSLAMInit(IoTRobot_SLAMClient_CBSet stCBset)
{
	g_pCRGBDSLAMClient=new IoTRobot_RGBDSLAM_Client;
	g_pCRGBDSLAMClient->SLAMInit();
	g_pCRGBDSLAMClient->m_cbSLAMOK=stCBset.cbSlam;
	g_pCRGBDSLAMClient->m_pSLAMOKContext=stCBset.pSLAMOKContext;

	g_pCRGBDSLAMClient->m_cbRGBDData=stCBset.cbRGBD;
	g_pCRGBDSLAMClient->m_pRGBDDataContext=stCBset.pRGBDDataContext;
	return 0;
}

int RGBDSLAM_Interface::RGBDSLAMRun()
{
	g_pCRGBDSLAMClient->SLAMRun();
	return 0;
}

int RGBDSLAM_Interface::RGBDSLAMStop()
{
	g_pCRGBDSLAMClient->SLAMStop();
	return 0;
}

int RGBDSLAM_Interface::RGBDSLAMUnint()
{
	if (g_pCRGBDSLAMClient!=NULL)
	{
		delete g_pCRGBDSLAMClient;
		g_pCRGBDSLAMClient=NULL;
	}
	return 0;
}

int RGBDSLAM_Interface::RGBSLAMParamsSetting(void *pParam)
{
	return g_pCRGBDSLAMClient->SLAMParamsSetting(pParam);
}

int RGBDSLAM_Interface::RGBDSLAMGetCurPos(double *pdPos)
{
	memcpy(pdPos,g_pCRGBDSLAMClient->m_dCurPos,6*sizeof(double));
	return 0;
}