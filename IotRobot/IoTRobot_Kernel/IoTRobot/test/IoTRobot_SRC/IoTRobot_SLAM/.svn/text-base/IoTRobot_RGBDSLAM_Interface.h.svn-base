#ifndef IOT_ROBOT_RGBDSLAM_INTERFACE_H
#define IOT_ROBOT_RGBDSLAM_INTERFACE_H
#define DLL_EXPORT __declspec(dllexport)
#include <boost/shared_ptr.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Core>

class DLL_EXPORT IoTRobot_RGBDSLAM_Interfaace
{
public:
	IoTRobot_RGBDSLAM_Interfaace();
	~IoTRobot_RGBDSLAM_Interfaace();
	int IoTRobot_RGBDSLAM_Init();
	int IoTRobot_RGBDSLAM_Run();
	int IoTRobot_RGBDSLAM_RunOneFrame(unsigned char *pucRGB,unsigned short *pusDepth,
		                              boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > &cloud,
									  Eigen::Matrix4f &final_transformation,unsigned long *pulRunInfo,
									  double* pdIMUdata, int status);
	int IoTRobot_RGBDSLAM_Uninit();
};
#endif