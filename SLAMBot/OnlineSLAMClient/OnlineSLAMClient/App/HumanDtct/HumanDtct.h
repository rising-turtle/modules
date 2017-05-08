#pragma once
#include "../App.h"
#include "C:\Program Files\OpenNI\Include\XnCppWrapper.h"

using namespace xn;
extern CRITICAL_SECTION g_cs2rdView;
class HumanDtct: public App
{
public:
	HumanDtct();
	~HumanDtct();
	virtual int AppInit(void *pcParams);
	virtual int AppRun(void *pcParams);
	virtual int AppStop(void *pcParams);
	virtual int AppUninit(void *pcParams);


	int ShowRslt();
	int StopShowRslt();

	UserGenerator *m_pCUserGenerator;
	unsigned char *m_pcRGB;
	unsigned char *m_pucRGB;

	ImgView m_cbImgView;
	
	int NewDataIn(unsigned char* pucRGB,unsigned short *pusDepth);
protected:
private:
	bool m_bStopHumanDtct;
	bool m_bNewDataIn;
	bool m_bShowRslt;
};