#pragma once
#include "../App.h"
#include "../../Com/Net/ClientNet.h"

class Resolve:public App
{
public:
	Resolve();
	~Resolve();
	virtual int AppInit(void *pcParams);
	virtual int AppRun(void *pcParams);
	virtual int AppStop(void *pcParams);
	virtual int AppUninit(void *pcParams);

	ClientNet *m_pCClientNet;
	static int RecvDataFromSvr(char *pcData,int nDataLen);

	static UploadInfo2UI m_cbUploadInfo2UI;
	static SendData2RobotServer m_cbSendData2RobotSrv;

protected:
private:
	bool m_bStop;
	typedef struct ResolveRslt
	{
		int img_index[3];
		float confidence[3];
		float loc_x[3];
		float loc_y[3];
		float loc_z[3];
	}ResolveRslt;
};