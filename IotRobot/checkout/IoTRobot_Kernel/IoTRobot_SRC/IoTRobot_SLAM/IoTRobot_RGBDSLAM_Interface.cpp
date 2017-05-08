#include "IoTRobot_RGBDSLAM_Interface.h"
#include "IoTRobot_RGBDSLAM.h"

Iot_Robot_rgdbslam test;


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
																double* pdIMUdata, int status,bool& rebuild_map, vector<CPose3D>& robotpath_update,
																vector<int>& id_of_node,int& num_of_node,_Node3DDesc** features)
{
	
	IoT_ConvertToXYZRGBPointCloud(pucRGB,pusDepth,cloud);
	return test.RunOneFrameHogman(pucRGB,pusDepth,cloud,final_transformation,pulRunInfo,pdIMUdata,status,rebuild_map,robotpath_update,id_of_node,num_of_node,features);
	//return test.RunOneFrameHogman(pucRGB,pusDepth,cloud,final_transformation,pulRunInfo,pdIMUdata,status,rebuild_map,robotpath_update,id_of_node);
	//return test.RunOneFrameHogman(pucRGB,pusDepth,cloud,final_transformation,pulRunInfo,pdIMUdata,status,rebuild_map,robotpath_update,id_of_node);
}


int IoTRobot_RGBDSLAM_Interfaace::IoTRobot_RGBDSLAM_RunOneFrame2(unsigned char *pucRGB,unsigned short *pusDepth, 
																 boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > &cloud, 
																 Eigen::Matrix4f &final_transformation,unsigned long *pulRunInfo,
																 double* pdIMUdata, int status,bool& rebuild_map, vector<CPose3D>& robotpath_update,
																 vector<int>& id_of_node,int& num_of_node,_Node3DDesc** features)
{
	return test.RunOneFrameHogman(pucRGB,pusDepth,cloud,final_transformation,pulRunInfo,pdIMUdata,status,rebuild_map,robotpath_update,id_of_node,num_of_node,features);

}
int IoTRobot_RGBDSLAM_Interfaace::IoTRobot_RGBDSLAM_Uninit()
{
	return 0;
}

int IoTRobot_RGBDSLAM_Interfaace::IoTRobot_RGBDSLAM_SetParams(map<string, string>& params){
	string key;
	map<string, string>::iterator it;
	// "min_trans", "max_trans", "min_rot", "max_rot"
	// "graph_degree", "min_inlier"
	// "max_graph_thres", "min_graph_thres"
	// "detector_type"(SURF|FAST)

	key = "graph_degree";
	it = params.find(key);
	if(it != params.end())
		global_connectivity = atoi(it->second.c_str());

	key = "max_comparison";
	it = params.find(key);
	if(it != params.end())
		global_potential_nodes = atoi(it->second.c_str());

	key = "max_graph_thres";
	it = params.find(key);
	if(it != params.end())
		global_graph_size = atoi(it->second.c_str());

	key = "min_graph_thres";
	it = params.find(key);
	if(it != params.end())
		global_bg_graph_threshold = atoi(it->second.c_str());

	key = "min_trans";
	it = params.find(key);
	if(it != params.end())
		global_min_translation_meter = atof(it->second.c_str());

	key = "min_rot";
	it = params.find(key);
	if(it != params.end())
		global_min_rotation_degree = atof(it->second.c_str());

	key = "max_trans";
	it = params.find(key);
	if(it != params.end())
		global_max_translation_meter = atof(it->second.c_str());

	key = "detector_type";
	it = params.find(key);
	if(it != params.end())
		global_feature_detector_type = it->second;

	return 0;
}

int IoTRobot_RGBDSLAM_Interfaace::IoTRobot_RGBDSLAM_Reset(){
	test.reset();
	return 0;
}

int IoTRobot_RGBDSLAM_Interfaace::IoTRobot_RGBDSLAM_ResetSession(CPose3D pose){
	test.resetSession(pose);
	return 0;
}

int IoTRobot_RGBDSLAM_Interfaace::IoTRobot_RGBDSLAM_UpdateDetector(){
	test.setDetector(global_feature_detector_type);
	return 0;
}