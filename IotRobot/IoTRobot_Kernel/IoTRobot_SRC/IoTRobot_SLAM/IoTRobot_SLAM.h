#ifndef IOT_ROBOT_SLAM
#define IOT_ROBOT_SLAM

#include "pcl/point_types.h"
#include "pcl/point_cloud.h"
#include "Eigen/Core"



typedef pcl::PointXYZRGB point_type;
typedef pcl::PointCloud<point_type> pointcloud_type;


class Iot_Robot_Slam{
public:
	Iot_Robot_Slam(){}
	~Iot_Robot_Slam(){}

	virtual int init() = 0;
	//virtual int runoneframe(pcl::PointCloud<pcl::PointXYZRGB>::Ptr frame) = 0; // This is the actual process of slam
	virtual int runoneframe(unsigned char *pucRGB,unsigned short *pusDepth,
		boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > &cloud,
		Eigen::Matrix4f &final_transformation) = 0;

	/*virtual int RunOneFrameHogman(
		unsigned char *pucRGB,unsigned short *pusDepth,
		boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > &cloud,
		Eigen::Matrix4f &final_transformation)=0;*/
	virtual int uninit() = 0;

};


#endif