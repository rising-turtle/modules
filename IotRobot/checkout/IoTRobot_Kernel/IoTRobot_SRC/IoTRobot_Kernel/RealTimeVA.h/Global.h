#ifndef GLOBAL_H_
#define GLOBAL_H_

#define WIDTH					320		//video width
#define HEIGHT					240		//video height
#define STATICTICS_INTERVAL		5000	//time interval of statictics routine
#define JPEG_QUANLITY			40		
//#define REMOTE_IP				{192, 168, 1, 101}
#define LOCAL_AUDIO_RTP_PORT	6000
#define LOCAL_VIDEO_RTP_PORT	8000
#define REMOTE_AUDIO_RTP_PORT	6000
#define REMOTE_VIDEO_RTP_PORT	7000

#include "CRealTimeVA.h"
#include <iostream>
using namespace std;


//openCV
/*#include "opencv2/core/core_c.h"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui_c.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv/cv.h"
#include "opencv/highgui.h"
using namespace cv;

#ifdef _DEBUG
	#pragma comment(lib, "opencv_core220d.lib")
	#pragma comment(lib, "opencv_highgui220d.lib")
	#pragma comment(lib, "opencv_imgproc220d.lib")
#else
	#pragma comment(lib, "opencv_core220.lib")
	#pragma comment(lib, "opencv_highgui220.lib")
	#pragma comment(lib, "opencv_imgproc220.lib")
#endif*/

#define CAMERA

#endif	//#ifndef GLOBAL_H_

