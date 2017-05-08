#pragma once
int typedef (*ImgView)(unsigned char *pucImgData);
int typedef (*UploadInfo2UI)(char *pcData);
int typedef (*SendData2RobotServer)(float *fPos);
class App
{
public:
	virtual int AppInit(void *pcParams)=0;
	virtual int AppRun(void *pcParams)=0;
	virtual int AppStop(void *pcParams)=0;
	virtual int AppUninit(void *pcParams)=0;
protected:
private:
};