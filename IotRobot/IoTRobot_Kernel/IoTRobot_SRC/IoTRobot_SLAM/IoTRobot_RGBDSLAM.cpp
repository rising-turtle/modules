#include "IoTRobot_RGBDSLAM.h"


#include "Openni.h"
#include <boost/array.hpp>
#include <boost/shared_array.hpp>
#include <opencv2/highgui/highgui_c.h>

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


// add feature to image
void Iot_Robot_rgdbslam::addFeatureToImage(Node* pNode, unsigned char* rgb_buff){

	int index =0;
	cv::Point2f p2d;
	for(unsigned int i = 0; i < pNode->feature_locations_2d_.size(); i++){
		p2d = pNode->feature_locations_2d_[i].pt;
		if (p2d.x >= pNode->pc_col.width  || p2d.x < 0 ||
			p2d.y >= pNode->pc_col.height || p2d.y < 0 ||
			_isnan(p2d.x) || _isnan(p2d.y)){ //TODO: Unclear why points should be outside the image or be NaN
				continue;
		}

		// get location of feature point
		index = (int)p2d.y * pNode->pc_col.width + (int)p2d.x;
		index *= 3;
		// set this point black
		rgb_buff[index] = 0;
		rgb_buff[index+1] = 0;
		rgb_buff[index+2] =0;
	}
}
// change sequence of rgb_flow
void Iot_Robot_rgdbslam::fromrgbtobgr(unsigned char* rgb_buff, int len){
	unsigned char tmp;
	unsigned char* pr = rgb_buff;
	for(int i=0;i<len; i+=3)
	{ 
		// from rgb to bgr 
		tmp = *(pr+i);
		*(pr+i) = *(pr+i+2);
		*(pr+i+2) = tmp;
	}
	return ;
}
void Iot_Robot_rgdbslam::getFileName(int frame_num, std::string & filename){
	static string path_name("D:\\PCL_install_on_VS2008\\PCL-No_CMake\\PCL-1.1.1-Source\\PCL-1.1.1-Source\\bin32\\rgdbslam\\Nodefeatures\\");
	static string bmp(".bmp");
	static string file_name("frame");
	char num_str[10];
	itoa(frame_num,num_str,10);
	string tmp(num_str);
	// clear already name
	filename.clear();
	filename = path_name + file_name + tmp + bmp;
	return ;
}
// convert to opencv image format
void Iot_Robot_rgdbslam::convertToIplImage(boost::shared_ptr<IplImage> pImage,unsigned char * rgb_buff){

	pImage->width = 640;
	pImage->height = 480;
	pImage->widthStep = pImage->width * 3;
	pImage->imageSize = pImage->height * pImage->widthStep;
	pImage->dataOrder = 0; // interleaved

	pImage->ID = 0;
	pImage->nChannels = 3;
	pImage->depth = IPL_DEPTH_8U;
	pImage->origin = 0; // windows style top-left

	// must be null
	pImage->maskROI = NULL; 
	pImage->roi = NULL;
	pImage->imageData = (char*)rgb_buff;
	pImage->nSize = sizeof(*(pImage.get()));

	//pImage->imageDataOrigin = (char*)rgb_buff;
}
// save rgb_buff into bmp
void Iot_Robot_rgdbslam::saveImgeToBmp(std::string& filename,int frame_num, unsigned char* rgb_buff){	
	fromrgbtobgr(rgb_buff,640*480*3);
	getFileName(frame_num,filename);
	boost::shared_ptr<IplImage> pImage(new IplImage);
	convertToIplImage(pImage,rgb_buff);
	//cvSaveImage(filename.c_str(),rgb_buff);
	cvSaveImage(filename.c_str(),pImage.get());
}


//  0: matched and successful!
// -1: matched but not big tra : normal situation
// -2: it matched with a noisy frame
// -3: it lost: not matched with previous frames!
// -4: it lost: current frame has too little features < 50 !
int Iot_Robot_rgdbslam::RunOneFrameHogman(unsigned char *pucRGB,unsigned short *pusDepth,
										  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > &cloud, 
										  Eigen::Matrix4f &final_transformation,unsigned long *pulRunInfo,
										  double* pdIMUdata, int status,bool& rebuild_map,vector<CPose3D>& robotpath_update,
										  vector<int>& id_of_node,int& num_of_node,Node3DDesc** features)
{
	memcpy(m_cvRGBImage.data,pucRGB, 640*480*3);
	memcpy(m_cvDepthImage.data, pusDepth, 640*480*2);
	m_cvDepthImage.convertTo(m_cvDepth8UC1Image, CV_8UC1);
	// reset the PF vars
	gl_pf_fe = 0;
	gl_pf_fm = 0;
	gl_pf_id = 0;
	gl_pf_me = 0;
	gl_pf_op = 0;
	gl_pf_slam = 0;
	double slam_tick_start = ::GetTickCount();
	Node* node_ptr = createFeatureNode(m_cvRGBImage, cloud, m_cvDepth8UC1Image);
	pulRunInfo[0]=::GetTickCount();
	
	if(node_ptr->feature_locations_2d_.size() < FEATURES_THRESHOLD)
	{
		return -4;
	}

	//std::cout<<"IMU DATA 2:"<<(*pdIMUdata)<<" "<<*(pdIMUdata+1)<<" "<<*(pdIMUdata+2)<<" "<<*(pdIMUdata+3)<<" "<<*(pdIMUdata+4)<<" "<<*(pdIMUdata+5)<<endl;

		/*if(fabs(*pdIMUdata - 1.0)<1e-3 && fabs(*(pdIMUdata+1)-2.0) < 1e-3 && fabs(*(pdIMUdata+2)- 3.0)<1e-3 \
			&& fabs(*(pdIMUdata+3)- 4.0)<1e-3 && fabs(*(pdIMUdata+4)-5.0)<1e-3 && fabs(*(pdIMUdata+5)-6.0)<1e-3)
		{
			cout<<"filtering! &&&&&&&&&&&&&&&&&&&"<<endl;
			return -1;
		}*/
	/*
		IMU		(roll, pitch, yaw)
		KINECT	(yaw, -roll, pitch)
	*/
	if(status !=0 ) // using IMUData
	{
		//for the first frame
		static bool first_pose = true;
		static CPose3D firstPose;
		if(first_pose){  // relative to absolute coordinate, we have to transfer to local coordinate of first frame
			//firstPose.setrpy(D2R(*(pdIMUdata+3)),D2R(*(pdIMUdata+4)),D2R(*(pdIMUdata+5)));
			firstPose.setrpy(D2R(*(pdIMUdata+4)*(-1)),D2R((*(pdIMUdata+5))),D2R(*(pdIMUdata+3)));
			
			cout<<"FIRST_POSE to absolute coordinate!"<<endl;
			firstPose.output(std::cout);
			first_pose = false;
		}
		else
		{
			CPose3D abspose(D2R(*(pdIMUdata+3)),D2R((*(pdIMUdata+5))),D2R(*(pdIMUdata+4)*(-1)),0,0,0);//(D2R(*(pdIMUdata+5)),D2R(*(pdIMUdata+4)),D2R(*(pdIMUdata+3)),0,0,0);
			m_graph_mgr.curr_pose = abspose - firstPose;
			cout<<"CURR_POSE IMU Data: "<<endl;
			m_graph_mgr.curr_pose.output(std::cout);
		}
		// using IMU data when calculate relative pose
		m_graph_mgr.usingIMU = true;
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

			std::map<int, AISNavigation::Graph::Vertex*>::reverse_iterator vertex_iter = m_graph_mgr.optimizer_->vertices().rbegin();
			id_of_node.push_back(vertex_iter->second->id()); // record id of this new node
			robotpath.push_back(pose);


			// transform features of node
			*features=(Node3DDesc*)realloc(*features,sizeof(Node3DDesc)+node_ptr->feature_locations_3d_.size()*sizeof(Feature3DDesc));
			Node3DDesc* pNodeFeatures=*features;
			pNodeFeatures->id = node_ptr->id_;
			pNodeFeatures->n = node_ptr->feature_locations_3d_.size();
			Feature3DDesc* pFeatures = pNodeFeatures->desc3DList;
			for(int i=0; i<pNodeFeatures->n; i++)
			{
				Eigen::Vector4f feature_loc=node_ptr->feature_locations_3d_[i];
				pFeatures[i].xyz.x=feature_loc[0];
				pFeatures[i].xyz.y=feature_loc[1];
				pFeatures[i].xyz.z=feature_loc[2];
				for(int j=0;j<64;j++)
					pFeatures[i].desc[j] = node_ptr->feature_descriptors_.at<float>(i,j);
			}
			 

			// print slam performance data
			gl_pf_slam = ::GetTickCount()-slam_tick_start;
			gl_pf_gs = m_graph_mgr.graph_.size();
			double pf_gs = gl_pf_gs;

			mylogfile.writeid(gl_pf_fid++);
			mylogfile.writeintolog(pf_gs);
			mylogfile.writeintolog(gl_pf_fe);
			mylogfile.writeintolog(gl_pf_fm);
			mylogfile.writeintolog(gl_pf_id);
			mylogfile.writeintolog(gl_pf_me);
			mylogfile.writeintolog(gl_pf_op);
			mylogfile.writeintolog(gl_pf_slam, true);
	
		//	cout<<"after SLAM!"<<endl;
		//	pose.output(std::cout);
			Eigen::Matrix4f transformation;
			final_pose->getHomogeneousMatrix(transformation);
			final_transformation=transformation;

			if(status){ // using IMU data
				// update IMU Pose info after each successful SLAM process
				m_graph_mgr.last_pose = m_graph_mgr.curr_pose;
			}

			// save this frame into files for debug
			/*string filename;
			static int num_frame = 0;
			addFeatureToImage(node_ptr,pucRGB);
			saveImgeToBmp(filename,++num_frame,pucRGB);*/

			static int num_to_refresh = -1;
			if(++num_to_refresh >= 0){
				num_to_refresh = -1;
				//cout<<"!!! Clean the Cells in Mapbuilder!!!"<<endl;
				// update all path node's position
				// transformation all point cloud from Nodes
				// Merge all pc into a global_pc
				std::map<int, AISNavigation::Graph::Vertex*>::iterator vertex_iter = m_graph_mgr.optimizer_->vertices().begin();
				std::map<int, Node* >::iterator vertex_iter_graph = m_graph_mgr.graph_.begin();
				pcl::PointCloud<point_type>::Ptr pCloud;
				CPose3D node_pose;
				
				// refill robotpath_update
				robotpath_update.clear();
				// refill id_of_node
				id_of_node.clear();
				
				//pGlobalMap.reset(new pcl::PointCloud<pcl::PointXYZRGB>); // clear pGlobalMap
				for(vector<CPose3D>::iterator p_node_pose=robotpath.begin(); p_node_pose!=robotpath.end() && vertex_iter_graph!=m_graph_mgr.graph_.end();\
					p_node_pose++,vertex_iter++,vertex_iter_graph++)
				{
					AISNavigation::PoseGraph3D::Vertex* v_to_del =  reinterpret_cast<AISNavigation::PoseGraph3D::Vertex*>(vertex_iter->second);
					double x1=v_to_del->transformation.translation().x();
					double y1=v_to_del->transformation.translation().y();
					double z1=v_to_del->transformation.translation().z();
					_Vector<3, double> rpy = v_to_del->transformation.rotation().rotationMatrix().angles();
					double roll = rpy.roll();
					double pitch = rpy.pitch();
					double yaw = rpy.yaw();
					node_pose.setrpy(roll,pitch,yaw);
					node_pose.setxyz(x1,y1,z1);
					// output robotpath at Client_end
					//cout<<"fill node: "<<v_to_del->id()<<endl;
					//node_pose.output(std::cout)<<endl;
					robotpath_update.push_back(node_pose);
					id_of_node.push_back(v_to_del->id());
				}
				// set rebuild ready bit
				rebuild_map = true;
			}
			// obtain number of nodes in Graph
			num_of_node = m_graph_mgr.graph_.size();
			return 0;
		}
		else
		{
			m_graph_mgr.deleteLastFrame();
			delete node_ptr;
			// obtain number of nodes in Graph
			num_of_node = m_graph_mgr.graph_.size();
			return -2; // noisy plane
		}
	}
	else
	{
		int ret_err;
		if(m_graph_mgr.matched) // matched but not big tra
		{
			// obtain number of nodes in Graph
			num_of_node = m_graph_mgr.graph_.size();
			ret_err = -1; // normal status
		}
		else
		{
			// obtain number of nodes in Graph
			num_of_node = m_graph_mgr.graph_.size();
			ret_err = -3;
		}
		delete node_ptr;
		// obtain number of nodes in Graph
		num_of_node = m_graph_mgr.graph_.size();
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

void Iot_Robot_rgdbslam::reset(){
	m_graph_mgr.resetGraph();
	robotpath.clear();
}

void Iot_Robot_rgdbslam::resetSession(CPose3D pose){
	m_graph_mgr.resetSession(pose);
	robotpath.clear();
} 