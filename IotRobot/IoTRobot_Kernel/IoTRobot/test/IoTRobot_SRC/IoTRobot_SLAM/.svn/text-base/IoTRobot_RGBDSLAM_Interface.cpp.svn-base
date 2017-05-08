#include "IoTRobot_RGBDSLAM_Interface.h"
#include "IoTRobot_RGBDSLAM.h"

Iot_Robot_rgdbslam test("SURF", "SURF", "BruteForce");


void IoT_ConvertToXYZRGBPointCloud(unsigned char*pucRGB,unsigned short *pusDepth,
												   boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > cloud)
{
	cloud->header.frame_id = "/openni_rgb_optical_frame";;
	cloud->height = 480;
	cloud->width = 640;
	cloud->is_dense = false;

	cloud->points.resize(cloud->height * cloud->width);
	register int centerX = (cloud->width >> 1);
	int centerY = (cloud->height >> 1);
	register const XnDepthPixel* depth_map = pusDepth;
	register int color_idx = 0, depth_idx = 0;


	float bad_point = std::numeric_limits<float>::quiet_NaN();

	for (int v = -centerY; v < centerY; ++v)
	{
		for (register int u = -centerX; u < centerX; ++u, color_idx += 3, ++depth_idx)
		{
			pcl::PointXYZRGB& pt = cloud->points[depth_idx];
			/// @todo Different values for these cases
			// Check for invalid measurements
			if (depth_map[depth_idx] == 0 )
			{
				pt.x = pt.y = pt.z = bad_point;
			}
			else
			{
				pt.z=pusDepth[depth_idx] * 0.001f;
				pt.x=0.001904*u *pt.z;
				pt.y=0.001904*v *pt.z;
				pt.r=pucRGB[color_idx];
				pt.g=pucRGB[color_idx+1];
				pt.b=pucRGB[color_idx+2];

			}
		}
	}
}
IoTRobot_RGBDSLAM_Interfaace::IoTRobot_RGBDSLAM_Interfaace()
{

}

IoTRobot_RGBDSLAM_Interfaace::~IoTRobot_RGBDSLAM_Interfaace()
{

}

int IoTRobot_RGBDSLAM_Interfaace::IoTRobot_RGBDSLAM_Init()
{
	test.init();
	return 0;
}

int IoTRobot_RGBDSLAM_Interfaace::IoTRobot_RGBDSLAM_Run()
{
	
	return 0;
}



int IoTRobot_RGBDSLAM_Interfaace::IoTRobot_RGBDSLAM_RunOneFrame(unsigned char *pucRGB,unsigned short *pusDepth, 
																boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > &cloud, 
																Eigen::Matrix4f &final_transformation,unsigned long *pulRunInfo,
																double* pdIMUdata, int status)
{
	
	IoT_ConvertToXYZRGBPointCloud(pucRGB,pusDepth,cloud);
	return test.RunOneFrameHogman(pucRGB,pusDepth,cloud,final_transformation,pulRunInfo,pdIMUdata,status);
}

int IoTRobot_RGBDSLAM_Interfaace::IoTRobot_RGBDSLAM_Uninit()
{
	return 0;
}