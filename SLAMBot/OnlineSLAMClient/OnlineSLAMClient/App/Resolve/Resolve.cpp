#include "Resolve.h"
#include "../../Com/Net/ClientNet.h"
UploadInfo2UI Resolve::m_cbUploadInfo2UI;
SendData2RobotServer Resolve::m_cbSendData2RobotSrv;
Resolve::Resolve()
{

}

Resolve::~Resolve()
{

}
int Resolve::AppInit(void *pcParams)
{

	return 0;
}
int Resolve::AppRun(void *pcParams)
{
	m_bStop=false;
	m_pCClientNet->SendData("ClientIn",8);




	while (!m_bStop)
	{
	//	m_cbSendData2RobotSrv(fTest);
		Sleep(1000);
	}
	return 0;
}
int Resolve::AppStop(void *pcParams)
{
	m_bStop=true;
	return 0;
}
int Resolve::AppUninit(void *pcParams)
{
	return 0;
}

int Resolve::RecvDataFromSvr(char *pcData,int nDataLen)
{
	char cUploadInfo[100];
	float fPos[3];
	memset(cUploadInfo,0,100);
	ResolveRslt stResolveRslt;
	memcpy(&stResolveRslt,pcData,nDataLen);
	cUploadInfo[0]=100;
	memcpy(&cUploadInfo[1],&stResolveRslt.loc_x[0],4);
	memcpy(&cUploadInfo[5],&stResolveRslt.loc_y[0],4);
	memcpy(&cUploadInfo[9],&stResolveRslt.loc_z[0],4);
	memcpy(&cUploadInfo[13],&stResolveRslt.loc_x[1],4);

	fPos[0]=stResolveRslt.loc_x[0];
	fPos[1]=stResolveRslt.loc_y[0];
	fPos[2]=stResolveRslt.loc_z[0];
	if (stResolveRslt.loc_x[1]>0.0001)
	{
		printf("confidence:%f  ,x:%f  ,y:%f,  z:%f  \n",stResolveRslt.loc_x[1],fPos[0],fPos[1],fPos[2]);
		m_cbUploadInfo2UI(cUploadInfo);
		m_cbSendData2RobotSrv(fPos);
	}



	
	//printf("RESOLVE Result: X: %f, Y: %f, Z: %f \n", stResolveRslt.loc_x[0],stResolveRslt.loc_y[0],stResolveRslt.loc_z[0]);
	return 0;
}