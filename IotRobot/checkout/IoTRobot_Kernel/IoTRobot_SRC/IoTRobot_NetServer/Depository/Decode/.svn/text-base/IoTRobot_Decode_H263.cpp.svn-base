#include "IoTRobot_Decode_H263.h"


IoTRobot_Decode_H263::IoTRobot_Decode_H263()
{

}


IoTRobot_Decode_H263::~IoTRobot_Decode_H263()
{

}

int IoTRobot_Decode_H263::IoTRobot_DecodeInit()
{
	InitH263Decoder();
	InitConvertTable();


	m_pucR=new unsigned char[352*288];
	m_pucG=new unsigned char[352*288];
	m_pucB=new unsigned char[352*288];
	return 0;
}

int IoTRobot_Decode_H263::IoTRobot_DecodeRun()
{
	return 0;
}

int IoTRobot_Decode_H263::IoTRobot_DecodeUnint()
{
	return 0;
}

int IoTRobot_Decode_H263::IoTRobot_DecodeOneFrame(unsigned char *pucEncodeData,int nEncodeDataLen, unsigned char *pucDecodeData,int nDecodeDataLen)
{
	DecompressFrame(pucEncodeData,nEncodeDataLen,pucDecodeData,nDecodeDataLen);
	return 0;
}

int IoTRobot_Decode_H263::YUV420Convert2RGB24_CIF(unsigned char *pucRGB24,unsigned char *pucTUV420)
{
	int i;
	ConvertYUV2RGB(m_pucR,m_pucG,m_pucB,pucTUV420,352,288);

	for (i=0;i<352*288;i++)
	{
		pucRGB24[i*3]=m_pucR[i];
		pucRGB24[i*3+1]=m_pucG[i];
		pucRGB24[i*3+2]=m_pucB[i];
	}
	return 0;
}