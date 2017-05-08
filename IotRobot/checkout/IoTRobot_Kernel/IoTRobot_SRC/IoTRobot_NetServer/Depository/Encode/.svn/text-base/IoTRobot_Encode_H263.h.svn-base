#pragma once
#include "IoTRobot_Encode.h"
#include "IoTRobot_Encode_Define.h"

#include "opencv/cv.h"
#include "opencv/highgui.h"
#include "./encoder/libr263.h"

//#include "opencv2/core/core_c.h"
//#include <opencv2/core/core.hpp>
//#include "opencv2/highgui/highgui_c.h"
//#include <opencv2/highgui/highgui.hpp>
//#include <opencv2/imgproc/imgproc.hpp>


#ifdef _DEBUG
#pragma comment(lib, "opencv_core230d.lib")
#pragma comment(lib, "opencv_highgui230d.lib")
#pragma comment(lib, "opencv_imgproc230d.lib")
#else
#pragma comment(lib, "opencv_core230.lib")
#pragma comment(lib, "opencv_highgui230.lib")
#pragma comment(lib, "opencv_imgproc230.lib")
#endif


using namespace cv;

class IoTRobot_Encode_H263:public IoTRobot_Encode
{
public:
	IoTRobot_Encode_H263();
	virtual ~IoTRobot_Encode_H263();

	virtual int IoTRobot_EncodeInit();
	int IoTRobot_EncodeInit(int nOriginalType,int nConvertType);
	virtual int IoTRobot_EncodeRun();
	virtual int IoTRobot_EncodeUnint();

	int IoTRobot_EncodeOneFrame(unsigned char *pucData,unsigned char *pucEncodeData,int *pnEncodeDataLen,int nType);
	int IoTRobot_DataTypeChage(int nOriginalType,int nConvertType);
protected:
private:


	int LookUpTable();
	short *m_psHorizonTable;
	short *m_psVerticalTable;
	int *m_nTable;
	int m_nFrameCount;
	static int m_nByteCount;
	static CParam m_263Param;
	static void H263_Encode_CallBack(int nData);
	static unsigned char *m_pucEncodeData;
	Mat  m_cvRGB;
	Mat  m_cvRGB2;
	Mat  m_cvRGB3;

	unsigned int *puiYUV420;


	void InitLookupTable();
	int  ConvertRGB2YUV(int w,int h,unsigned char *rgbdata,unsigned int *yuv);


	// Conversion from YUV420 to RGB24
	void InitConvertTable();
	void ConvertYUV2RGB(unsigned char *src0,unsigned char *src1,unsigned char *src2,unsigned char *dst_ori,
		int width,int height);
};