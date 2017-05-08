#include "IoTRobot_USBCamera.h"

IoTRobot_USBCamera::IoTRobot_USBCamera()
{
	m_pucQVGABuff=new unsigned char [CAMERA_WIDTH*CAMERA_HEIGHT*3];
	m_pCCameraDS=NULL;
}

IoTRobot_USBCamera::~IoTRobot_USBCamera()
{
	if (m_pucQVGABuff!=NULL)
	{
		delete [] m_pucQVGABuff;
		m_pucQVGABuff=NULL;
	}
}

int IoTRobot_USBCamera::USBCameraInit(int nIdx,USBCameraCallBack_ImgData cbImgData,void *pContext)
{
#ifdef CAMERA_OPENCV
	if (!m_cvCamCapture.isOpened())
		m_cvCamCapture.open(1);

	if (!m_cvCamCapture.isOpened())
	{
		return -1;
	}
#endif

#ifdef CAMERA_DIRECTSHOW
	m_pCCameraDS = new CCameraDS();
	int camCount = m_pCCameraDS->CameraCount();
	char camName[30];
	m_pCCameraDS->CameraName(0, camName, 30);			//获取指定摄像头的名字
	bool openResult = m_pCCameraDS->OpenCamera(0, false, CAMERA_WIDTH, CAMERA_HEIGHT);	//0为USB视频设备
	if (!openResult){
		return -1;
	}
#endif

	m_cbImgData=cbImgData;
	m_pContext=pContext;

	return 0;
}


int IoTRobot_USBCamera::USBCameraRun()
{
	LPDWORD ID=0;
	m_bStopGetUSBCameraData=false;
	
	m_hThreadGetUSBCameraData=CreateThread(NULL,0,(LPTHREAD_START_ROUTINE)ThreadGetUSBCameraData,this,0,ID);
	return 0;
}

int IoTRobot_USBCamera::USBCameraUninit()
{
	if (m_pCCameraDS!=NULL)
	{
		delete m_pCCameraDS;
		m_pCCameraDS=NULL;
	}
	return 0;
}


int IoTRobot_USBCamera::USBCameraStop()
{
	m_bStopGetUSBCameraData=true;
	WaitForSingleObject(m_hThreadGetUSBCameraData, INFINITE);
	return 0;
}
UINT IoTRobot_USBCamera::ThreadGetUSBCameraData(LPVOID lpParam)
{

	IoTRobot_USBCamera *pUSBCam=(IoTRobot_USBCamera *)lpParam;
	while (!pUSBCam->m_bStopGetUSBCameraData)
	{
#ifdef CAMERA_OPENCV
		pUSBCam->m_cvCamCapture>>pUSBCam->m_cvOneFrame;
#endif


#ifdef CAMERA_DIRECTSHOW
		pUSBCam->m_Frame=pUSBCam->m_pCCameraDS->QueryFrame();
		memcpy(pUSBCam->m_pucQVGABuff,pUSBCam->m_Frame->data,CAMERA_WIDTH*CAMERA_HEIGHT*3);
#endif

		pUSBCam->m_cbImgData(pUSBCam->m_pucQVGABuff,pUSBCam->m_pContext);
		Sleep(40);
	}






	return 0;
}