#ifndef IOT_ROBOT_RGBDSLAM_INTERFACE_H
#define IOT_ROBOT_RGBDSLAM_INTERFACE_H
#define DLL_EXPORT __declspec(dllexport)
#include <boost/shared_ptr.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Core>
#include <map>
#include <string>
using namespace std;
class CPose3D;
struct _Node3DDesc;

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
									  double* pdIMUdata, int status,bool& rebuild_map, std::vector<CPose3D>&,std::vector<int>& ,int& ,_Node3DDesc**);

	int IoTRobot_RGBDSLAM_RunOneFrame2(unsigned char *pucRGB,unsigned short *pusDepth,
		boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > &cloud,
		Eigen::Matrix4f &final_transformation,unsigned long *pulRunInfo,
		double* pdIMUdata, int status,bool& rebuild_map, std::vector<CPose3D>&,std::vector<int>& ,int& ,_Node3DDesc**);
	int IoTRobot_RGBDSLAM_Uninit();
	// "min_trans"(float, meter), "max_trans"(float, meter), "min_rot"(float, degree), "max_rot"(float, degree)
	// "graph_degree" (int, 3+), "max_comparison" (int, 5+), "min_inlier" (int, 10+),
	// "max_graph_thres" (int, 100-), "min_graph_thres" (int, 4+)
	// "detector_type"(string, SURF|FAST|HARRIS|GFTT|STAR|SIFT|MSER|PyramidFAST|PyramidGFTT)
	int IoTRobot_RGBDSLAM_SetParams(map<string, string>& params);
	int IoTRobot_RGBDSLAM_Reset();
	int IoTRobot_RGBDSLAM_ResetSession(CPose3D pose);
	int IoTRobot_RGBDSLAM_UpdateDetector();
};
#endif