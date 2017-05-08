#pragma once
#include "../App.h"


#include "../../Sensor/IMU/INSProcessor.h"

#include "PlanarDetector.h"
class PlanDtct: public App
{
public:
	PlanDtct();
	~PlanDtct();
	virtual int AppInit(void *pcParams);
	virtual int AppRun(void *pcParams);
	virtual int AppStop(void *pcParams);
	virtual int AppUninit(void *pcParams);

	int ShowRslt();
	int StopShowRslt();

	int NewDataIn(unsigned char* pucRGB,unsigned short *pusDepth);
	unsigned char *m_pucRGBView;
	ImgView m_cbImgView;

protected:
private:
	union 
	{
		unsigned int (WINAPI   *Run)(void *);
		unsigned int (WINAPI   INSMonitor::*MemberPorc)();
	}ProcINSMonitor;

	unsigned char *m_pucRGB;
	unsigned short *m_pusDepth;

	unsigned short * m_pusDepthDownSample;
	unsigned char* m_pucRGBDownSample;
	bool m_bNewDataIn;

	bool m_bStopPlanDtct;
	bool m_bShowRslt;
};