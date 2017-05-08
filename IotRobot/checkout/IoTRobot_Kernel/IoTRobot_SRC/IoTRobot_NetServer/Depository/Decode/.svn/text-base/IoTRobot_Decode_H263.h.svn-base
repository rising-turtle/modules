#pragma once
#include "IoTRobot_Decode.h"
#include "./decoder/Tmndec.h"
#include "./decoder/convert.h"

class IoTRobot_Decode_H263:public IoTRobot_Decode
{
public:
	IoTRobot_Decode_H263();
	virtual ~IoTRobot_Decode_H263();

	virtual int IoTRobot_DecodeInit();
	virtual int IoTRobot_DecodeRun();
	virtual int IoTRobot_DecodeUnint();

	int IoTRobot_DecodeOneFrame(unsigned char *pucEncodeData,int nEncodeDataLen,
		                        unsigned char *pucDecodeData,int nDecodeDataLen);
	int YUV420Convert2RGB24_CIF(unsigned char *pucRGB24,unsigned char *pucTUV420);

	unsigned char *m_pucR,*m_pucG,*m_pucB;
protected:
private:
};