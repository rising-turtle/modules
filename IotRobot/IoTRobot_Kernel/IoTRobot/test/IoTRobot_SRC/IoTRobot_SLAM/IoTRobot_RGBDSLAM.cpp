#include "IoTRobot_RGBDSLAM.h"


#include "Openni.h"
#include <boost/array.hpp>
#include <boost/shared_array.hpp>

#define PI 3.141592654
#define D2R(deg) ((deg*PI)/180.0)

Node* Iot_Robot_rgdbslam::createFeatureNode(const cv::Mat& visual , boost::shared_ptr<pointcloud_type>& point_cloud, 
													  const cv::Mat& depth){
														  Node* node_ptr = new Node(visual, m_detector, m_extractor, m_matcher,
															  point_cloud, depth);
														  return node_ptr;
}

bool Iot_Robot_rgdbslam::computeNodePose(Node* node_ptr, double* xyz, double* rpy){

	if(m_graph_mgr.addNode(node_ptr))
	{
		xyz[0] = m_graph_mgr.latest_pose[0];
		xyz[1] = m_graph_mgr.latest_pose[1];
		xyz[2] = m_graph_mgr.latest_pose[2];

		//m_graph_mgr.curr_pose.getrpy(rpy);// trust IMU
		
		rpy[0] = m_graph_mgr.latest_pose[3];
		rpy[1] = m_graph_mgr.latest_pose[4];
		rpy[2] = m_graph_mgr.latest_pose[5];

		return true;
	}
	else
		return false;
}

void Iot_Robot_rgdbslam::getImagesandDepthMetaData(boost::shared_ptr<pointcloud_type> point_cloud,
															 unsigned char* rgbbuf,
															 unsigned char* depthbuf
															 )
{
	unsigned short * pdepth =(unsigned short*) (depthbuf);
	unsigned char  * pimage = rgbbuf;  
	unsigned int totalnum = point_cloud->width * point_cloud->height;

	float bad_point = std::numeric_limits<float>::quiet_NaN();


	for(size_t i=0;i<totalnum;i++){

		pcl::PointXYZRGB& pt = point_cloud->points[i];
		// get rgb-info 
		*pimage = pt.r;
		pimage++;
		*pimage = pt.g;
		pimage++;
		*pimage = pt.b;
		pimage++;

		// get depth-info
		if(pt.x == bad_point && pt.y == bad_point && pt.z == bad_point){
			*pdepth = 0;
		}
		else
		{
			*pdepth = pt.z * 1000.0f;
		}
		pdepth ++;
	}
}

// whether pose is good
bool Iot_Robot_rgdbslam::IsBigTra(CPose3D & pose){
	if(fabs(pose.m_coords[0]) > 2 || \
		fabs(pose.m_coords[1]) > 2 || \
		fabs(pose.m_coords[2]) > 2 || \
		fabs(R2D(pose.yaw)) > 50 || \
		fabs(R2D(pose.pitch)) > 50 || \
		fabs(R2D(pose.roll)) > 50 )
		return false;
	if(fabs(pose.m_coords[0]) > 0.1 || \
		fabs(pose.m_coords[1]) > 0.1 || \
		fabs(pose.m_coords[2]) > 0.1 || \
		fabs(R2D(pose.yaw)) > 5 || \
		fabs(R2D(pose.pitch)) > 5 || \
		fabs(R2D(pose.roll)) > 5 )
		return true;
	return false;
}

int Iot_Robot_rgdbslam::init()
{
	m_cvRGBImage=cv::Mat(480, 640, CV_8UC3);
	//m_cvRGBImage=cv::Mat(480, 640,CV_8UC1);
	m_cvDepthImage=cv::Mat(480, 640, CV_16UC1);
	m_cvDepth8UC1Image=cv::Mat(480, 640, CV_8UC1);
	return 0;
}


//  0: matched and successful!
// -1: matched but not big tra : normal situation
// -2: it matched with a noisy frame
// -3: it lost: not matched with previous frames!
// -4: it lost: current frame has too little features < 50 !
int Iot_Robot_rgdbslam::RunOneFrameHogman(unsigned char *pucRGB,unsigned short *pusDepth,
										  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > &cloud, 
										  Eigen::Matrix4f &final_transformation,unsigned long *pulRunInfo,
										  double* pdIMUdata, int status)
{
	memcpy(m_cvRGBImage.data,pucRGB, 640*480*3);
	memcpy(m_cvDepthImage.data, pusDepth, 640*480*2);
	m_cvDepthImage.convertTo(m_cvDepth8UC1Image, CV_8UC1);
	Node* node_ptr = createFeatureNode(m_cvRGBImage, cloud, m_cvDepth8UC1Image);
	pulRunInfo[0]=::GetTickCount();
	
	if(node_ptr->feature_locations_2d_.size() < FEATURES_THRESHOLD)
	{
		return -4;
	}
	// this is for initializing IMU
	//if(status)

	//std::cout<<"IMU DATA 2 :"<<(*pdIMUdata)<<" "<<*(pdIMUdata+1)<<" "<<*(pdIMUdata+2)<<" "<<*(pdIMUdata+3)<<" "<<*(pdIMUdata+4)<<" "<<*(pdIMUdata+5)<<endl;

		if(fabs(*pdIMUdata - 1.0)<1e-3 && fabs(*(pdIMUdata+1)-2.0) < 1e-3 && fabs(*(pdIMUdata+2)- 3.0)<1e-3 \
			&& fabs(*(pdIMUdata+3)- 4.0)<1e-3 && fabs(*(pdIMUdata+4)-5.0)<1e-3 && fabs(*(pdIMUdata+5)-6.0)<1e-3)
		{
		//	cout<<"filtering! &&&&&&&&&&&&&&&&&&&"<<endl;
			return -1;
		}
	/*
		IMU		(roll, pitch, yaw)
		KINECT	(yaw, -roll, pitch)
	*/
	if(status !=0 ) // using IMUData
	{
		printf("Use IMU Info!!!!!!!\n");
		//for the first frame
		static bool first_pose = true;
		static CPose3D firstPose;
		if(first_pose){  // relative to absolute coordinate, we have to transfer to local coordinate of first frame
			//firstPose.setrpy(D2R(*(pdIMUdata+3)),D2R(*(pdIMUdata+4)),D2R(*(pdIMUdata+5)));
			firstPose.setrpy(D2R(*(pdIMUdata+4))*(-1),D2R(*(pdIMUdata+5)),D2R(*(pdIMUdata+3)));
			
			//cout<<"FIRST_POSE to absolute coordinate!"<<endl;
			//firstPose.output(std::cout);
			first_pose = false;
		}
		else
		{
			CPose3D abspose(D2R(*(pdIMUdata+3)),D2R(*(pdIMUdata+5)),D2R(*(pdIMUdata+4))*(-1),0,0,0);//(D2R(*(pdIMUdata+5)),D2R(*(pdIMUdata+4)),D2R(*(pdIMUdata+3)),0,0,0);
			m_graph_mgr.curr_pose = abspose - firstPose;
			//cout<<"CURR_POSE IMU Data: "<<endl;
			//m_graph_mgr.curr_pose.output(std::cout);
		}
		// using IMU data when calculate relative pose
		//m_graph_mgr.usingIMU = true;
	}
	
	double xyz[3]={0,0,0};
	double rpy[3]={0,0,0};
	if(computeNodePose(node_ptr,xyz,rpy))
	{
		// Record current pose info
		CPose3D pose(rpy[2],rpy[1],rpy[0],xyz[0],xyz[1],xyz[2]);

		//cout<<"after IMU: "<<endl;
		//pose.output(std::cout);

		if(robotpath.size()==0 || !IsNoiseLocation(*robotpath.rbegin(),pose))
		{
			boost::shared_ptr<CPose3D> final_pose(new CPose3D(pose));
			robotpath.push_back(pose);
	
			//cout<<"after SLAM!"<<endl;
			//pose.output(std::cout);
			Eigen::Matrix4f transformation;
			final_pose->getHomogeneousMatrix(transformation);
			final_transformation=transformation;

			if(status!=0){ // using IMU data
				// update IMU Pose info after each successful SLAM process
				m_graph_mgr.last_pose = m_graph_mgr.curr_pose;
			}

			
			return 0;
		}
		else
		{
			m_graph_mgr.deleteLastFrame();
			delete node_ptr;
			return -2; // noisy plane
		}
	}
	else
	{
		int ret_err;
		if(m_graph_mgr.matched) // matched but not big tra
		{
			ret_err = -1; // normal status
		}
		else
		{
			ret_err = -3;
		}
		delete node_ptr;
		return ret_err;
	}	
}





int Iot_Robot_rgdbslam::runoneframe(unsigned char *pucRGB,unsigned short *pusDepth, 
									boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > &cloud, 
									Eigen::Matrix4f &final_transformation)
{
	// 1. get depth_image and image_matadata

	// Copy RGB data from OpenNI image to cv mat image
	memcpy(m_cvRGBImage.data,pucRGB /*xnImd.Data()*/, 640*480*3);
	memcpy(m_cvDepthImage.data, pusDepth, 640*480*2);

	m_cvDepthImage.convertTo(m_cvDepth8UC1Image, CV_8UC1);

	// 2. Create node to extract feature and wrap the image and feature data
	Node* node_ptr = createFeatureNode(m_cvRGBImage, cloud, m_cvDepth8UC1Image);
	node_ptr->id_ = m_graph_mgr.graph_.size();

	// indicate whether this frame will be added
	bool isGoodTra = false; 
	boost::shared_ptr<CPose3D> final_pose(new CPose3D());

	if(robotpath.size() == 0) // first frame
	{
		node_ptr->buildFlannIndex();
		m_graph_mgr.graph_[node_ptr->id_] = node_ptr;
		robotpath.push_back( CPose3D());
		isGoodTra = true;
	}
	else{
		// 3. Match with last pose to compute relative pose
		Node* pre_node = m_graph_mgr.graph_[m_graph_mgr.graph_.size()-1];
		MatchingResult mr = node_ptr->matchNodePair(pre_node);
		// relative Pose to previous frame
		boost::shared_ptr<CPose3D> pose(new CPose3D(mr.final_trafo)); 

		// If this new pose is good: not noise nor too small
		if(IsBigTra(*pose)){
			isGoodTra = true; // This is good transformation, 

			// last pose
			CPose3D prepose = *robotpath.rbegin();
			// compute pose relative to global coordinates
			*pose += prepose ; 
			// for debug
			//pose.get()->output(std::cout);
			final_pose = pose;

			// 4. Insert node into node-graph and Insert pose into robot-path
			node_ptr->buildFlannIndex();
			m_graph_mgr.graph_[node_ptr->id_] = node_ptr;
			robotpath.push_back(*pose);				
		}
		else
			delete node_ptr;
	}
	if(isGoodTra) // will display this frame
	{

		Eigen::Matrix4f transformation;
		final_pose->getHomogeneousMatrix(transformation);
		final_transformation=transformation;

		// Print the rotation matrix and translation vector
		isGoodTra=false;
		return 0;

	}

	return -1;
}

bool Iot_Robot_rgdbslam::IsNoiseLocation(CPose3D& lastpose,CPose3D& currentpose )
{
	return false;
	double uplimit_t = 0.5;
	double lxyz[3],cxyz[3];
	currentpose.getXYZ(cxyz);
	lastpose.getXYZ(lxyz);
	if(fabs(lxyz[0] - cxyz[0]) > uplimit_t || fabs(lxyz[1] - cxyz[1]) > uplimit_t || fabs(lxyz[2] - cxyz[2]) > uplimit_t)
		return true;
	return false;
}