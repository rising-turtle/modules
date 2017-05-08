#include "test_surf.h"
#include "Openni.h"
#include <boost/array.hpp>
#include <boost/shared_array.hpp>
#include <opencv2/highgui/highgui_c.h>
#include <pcl/io/pcd_io.h>

#define pi 3.141592654
#define R2D(r) ((r)/pi * 180)
#define D2R(d) ((d)*pi/180)

#define features_threshold 50	// if a new frame has less than this,then drop it!
#define search_range 2			// back search units

//using namespace cv;
// Reference to openni_listener in rgbdslam application
//FeatureDetector* CVisualSlam::createDetector( const string& detectorType ) {
//	FeatureDetector* fd = 0;
//    if( !detectorType.compare( "FAST" ) ) {
//        //fd = new FastFeatureDetector( 20/*threshold*/, true/*nonmax_suppression*/ );
//        fd = new DynamicAdaptedFeatureDetector (new FastAdjuster(20,true));
//
////												params->get<int>("adjuster_min_keypoints"),
////												params->get<int>("adjuster_max_keypoints"),
////												params->get<int>("fast_adjuster_max_iterations"));
//    }
//    else if( !detectorType.compare( "STAR" ) ) {
//        fd = new StarFeatureDetector( 16/*max_size*/, 5/*response_threshold*/, 10/*line_threshold_projected*/,
//                                      8/*line_threshold_binarized*/, 5/*suppress_nonmax_size*/ );
//    }
//    else if( !detectorType.compare( "SIFT" ) ) {
//        fd = new SiftFeatureDetector(SIFT::DetectorParams::GET_DEFAULT_THRESHOLD(),
//                                     SIFT::DetectorParams::GET_DEFAULT_EDGE_THRESHOLD());
//    }
//    else if( !detectorType.compare( "SURF" ) ) {
//		fd = new DynamicAdaptedFeatureDetector(new SurfAdjuster());
////        										params->get<int>("adjuster_min_keypoints"),
////												params->get<int>("adjuster_max_keypoints"),
////												params->get<int>("surf_adjuster_max_iterations"));
//    }
//    else if( !detectorType.compare( "MSER" ) ) {
//        fd = new MserFeatureDetector( 1/*delta*/, 60/*min_area*/, 14400/*_max_area*/, 0.35f/*max_variation*/,
//                0.2/*min_diversity*/, 200/*max_evolution*/, 1.01/*area_threshold*/, 0.003/*min_margin*/,
//                5/*edge_blur_size*/ );
//    }
//    else if( !detectorType.compare( "GFTT" ) ) {
//        fd = new GoodFeaturesToTrackDetector( 200/*maxCorners*/, 0.001/*qualityLevel*/, 1./*minDistance*/,
//                                              5/*int _blockSize*/, true/*useHarrisDetector*/, 0.04/*k*/ );
//    }
//    else {
//      fd = createDetector("SURF"); //recursive call with correct parameter
//    }
//    return fd;
//}

//DescriptorExtractor* CVisualSlam::createDescriptorExtractor( const string& descriptorType ) {
//    DescriptorExtractor* extractor = 0;
//    if( !descriptorType.compare( "SIFT" ) ) {
//        extractor = new SiftDescriptorExtractor();/*( double magnification=SIFT::DescriptorParams::GET_DEFAULT_MAGNIFICATION(), bool isNormalize=true, bool recalculateAngles=true, int nOctaves=SIFT::CommonParams::DEFAULT_NOCTAVES, int nOctaveLayers=SIFT::CommonParams::DEFAULT_NOCTAVE_LAYERS, int firstOctave=SIFT::CommonParams::DEFAULT_FIRST_OCTAVE, int angleMode=SIFT::CommonParams::FIRST_ANGLE )*/
//    }
//    else if( !descriptorType.compare( "SURF" ) ) {
//        extractor = new SurfDescriptorExtractor();/*( int nOctaves=4, int nOctaveLayers=2, bool extended=false )*/
//    }
//    else {
//      extractor = createDescriptorExtractor("SURF");
//    }
//    return extractor;
//}

//void CVisualSlam::extractVisualFeatures(const cv::Mat& visual , const cv::Mat& detection_mask , std::vector<cv::KeyPoint>& feature_locations_2d){
//	m_detector->detect( visual, feature_locations_2d, detection_mask);// fill 2d locations	
//	cv::Mat feature_descriptors_;
//	m_extractor->compute(visual, feature_locations_2d, feature_descriptors_);
//}

Node* CVisualSlam::createFeatureNode(const cv::Mat& visual , boost::shared_ptr<pointcloud_type>& point_cloud, 
									 const cv::Mat& depth){
	Node* node_ptr = new Node(visual, m_detector, m_extractor, m_matcher,
		point_cloud, depth);
	return node_ptr;
}

bool CVisualSlam::IsNoiseLocation(CPose3D& lastpose,CPose3D& currentpose )
{
	double uplimit_t = 0.4;
	double lxyz[3],cxyz[3];
	currentpose.getXYZ(cxyz);
	lastpose.getXYZ(lxyz);
	if(fabs(lxyz[0] - cxyz[0]) > uplimit_t || fabs(lxyz[1] - cxyz[1]) > uplimit_t || fabs(lxyz[2] - cxyz[2]) > uplimit_t)
		return true;
	return false;
}

bool CVisualSlam::computeNodePose(Node* node_ptr, double* xyz, double* rpy){

	if(m_graph_mgr.addNode(node_ptr))
	{
		xyz[0] = m_graph_mgr.latest_pose[0];
		xyz[1] = m_graph_mgr.latest_pose[1];
		xyz[2] = m_graph_mgr.latest_pose[2];

		rpy[0] = m_graph_mgr.latest_pose[3];
		rpy[1] = m_graph_mgr.latest_pose[4];
		rpy[2] = m_graph_mgr.latest_pose[5];

		return true;
	}
	else
		return false;
}



void CVisualSlam::displayResult(boost::shared_ptr<CPose3D> & pose){
	
	Eigen::Matrix4f final_transformation;
	pose->getHomogeneousMatrix(final_transformation);

	// Print the rotation matrix and translation vector
	Eigen::Matrix3f rotation = final_transformation.block<3,3>(0, 0);
	Eigen::Vector3f translation = final_transformation.block<3,1>(0, 3);

	printf("value by align");
	printf ("\n");
	printf ("    | %6.3f %6.3f %6.3f | \n", rotation (0,0), rotation (0,1), rotation (0,2));
	printf ("R = | %6.3f %6.3f %6.3f | \n", rotation (1,0), rotation (1,1), rotation (1,2));
	printf ("    | %6.3f %6.3f %6.3f | \n", rotation (2,0), rotation (2,1), rotation (2,2));
	printf ("\n");
	printf ("t = < %0.3f, %0.3f, %0.3f >\n", translation (0), translation (1), translation (2));
}
void showxyzrpy(double xyz[3],double rpy[3]){

//	cout<<"(x,y,z)" <<"("<<xyz[0]<<","<<xyz[1]<<","<<xyz[2]<<")" \
//		<<"(r,p,y)" <<"("<<R2D(rpy[0])<<","<<R2D(rpy[1])<<","<<R2D(rpy[2])<<")"<<endl;
}

void CVisualSlam::getImagesandDepthMetaData(boost::shared_ptr<pointcloud_type> point_cloud,
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

// add feature to image
void CVisualSlam::addFeatureToImage(Node* pNode, unsigned char* rgb_buff){
	
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
void CVisualSlam::getFileName(int frame_num, std::string & filename){
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
void CVisualSlam::convertToIplImage(boost::shared_ptr<IplImage> pImage,unsigned char * rgb_buff){
	
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
// change sequence of rgb_flow
void CVisualSlam::fromrgbtobgr(unsigned char* rgb_buff, int len){
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
// save rgb_buff into bmp
void CVisualSlam::saveImgeToBmp(std::string& filename,int frame_num, unsigned char* rgb_buff){	
	fromrgbtobgr(rgb_buff,640*480*3);
	getFileName(frame_num,filename);
	boost::shared_ptr<IplImage> pImage(new IplImage);
	convertToIplImage(pImage,rgb_buff);
	//cvSaveImage(filename.c_str(),rgb_buff);
	cvSaveImage(filename.c_str(),pImage.get());
}

// whether pose is good
bool CVisualSlam::IsBigTra(CPose3D & pose){
	if(fabs(pose.m_coords[0]) > 2 || \
		fabs(pose.m_coords[1]) > 2 || \
		fabs(pose.m_coords[2]) > 2 || \
		fabs(R2D(pose.yaw)) > 50 || \
		fabs(R2D(pose.pitch)) > 50 || \
		fabs(R2D(pose.roll)) > 50 )
			return false;
	if(fabs(pose.m_coords[0]) > 0.06 || \
		fabs(pose.m_coords[1]) > 0.06 || \
		fabs(pose.m_coords[2]) > 0.06 || \
		fabs(R2D(pose.yaw)) > 5 || \
		fabs(R2D(pose.pitch)) > 5 || \
		fabs(R2D(pose.roll)) > 5 )
		return true;
	return false;
}

// search all robotpath to obtain a valid match
int CVisualSlam::SearchAllPath(Node* new_node,boost::shared_ptr<CPose3D>& fpose){
	// threshold for matched pairs
	static int valid_numnber_of_matches = 30;
	// search robot_path to find most similar Node
	for(size_t i=1 ;i< search_range/*= m_graph_mgr.graph_.size()*/; i++)
	{
		Node* pre_node = m_graph_mgr.graph_[m_graph_mgr.graph_.size()-i];
		MatchingResult mr = new_node->matchNodePair(pre_node);
		if(mr.inlier_matches.size() >= valid_numnber_of_matches) // this is valid match
		{
			boost::shared_ptr<CPose3D> pose(new CPose3D(mr.final_trafo)); // relative Pose to previous frames
			fpose = pose;
			return i;
		}
			
	}
	// failed to find a similar Node
	return -1;
}

/*
void CVisualSlam::runHogman(){
	
	boost::shared_ptr<openni_wrapper::Image> openni_image;
	boost::shared_ptr<openni_wrapper::DepthImage> openni_depth_image;
	boost::shared_ptr<pointcloud_type> point_cloud;

	// Create cv mat image with the right size
	cv::Mat cvRGBImage(480, 640, CV_8UC3);
	cv::Mat cvGrayImage(480, 640,CV_8UC1);
	cv::Mat cvDepthImage(480, 640, CV_16UC1);
	cv::Mat cvDepth8UC1Image(480, 640, CV_8UC1);

	// 1. Initialize OpenNI Graber and Mapbuilder
	SimpleOpenNIViewer v;
	//v.startframe();
	v.start();

	boost::shared_ptr<MapBuilder> pMapbuilder(new MapBuilder);

	// Get OpenNI visual image
	static unsigned rgb_array_size = 0;
	static boost::shared_array<unsigned char> rgb_array(0);
	static unsigned char* rgb_buffer = 0;
	rgb_array_size = 480*640*3; // size of each frame
	rgb_array.reset(new unsigned char [rgb_array_size]);
	rgb_buffer = rgb_array.get();

	// Get OpenNI depth image
	static unsigned depth_array_size = 0;
	static boost::shared_array<unsigned char> depth_array(0);
	static unsigned char* depth_buffer = 0;
	depth_array_size = 480*640*2;
	depth_array.reset(new unsigned char [depth_array_size]);
	depth_buffer = depth_array.get();

	// record num of frames
	static int frame_num = 0;
	double start_t_slam, end_t_slam;  // record SLAM time-consuming
	char buf[100]; // for receive char* from sprintf

	// check every step of SLAM
	double data_start_t, data_end_t, fd_start_t,fd_end_t;

	while(1){
		while(v.getCloudPoint( point_cloud))
		{
			// record start time of SLAM
			//start_t_slam = ::GetTickCount();
			data_start_t = ::GetTickCount();


			// get depth_image and image_matadata
			getImagesandDepthMetaData(point_cloud,rgb_buffer, depth_buffer);

			// Copy RGB data from OpenNI image to cv mat image
			memcpy(cvRGBImage.data,rgb_buffer , 640*480*3);
			memcpy(cvDepthImage.data, depth_buffer, 640*480*2);
			cvDepthImage.convertTo(cvDepth8UC1Image, CV_8UC1);

			data_end_t = ::GetTickCount();

			fd_start_t = data_end_t;

			// 3. Create node to extract feature and wrap the image and feature data
			Node* node_ptr = createFeatureNode(cvRGBImage, point_cloud, cvDepth8UC1Image);

			fd_end_t = ::GetTickCount();
			// filter frame that obtains too small features
			if(node_ptr->feature_locations_2d_.size() < features_threshold){
				cout<<" this frame has too small features! drop it!"<<endl;
				continue;
			}
			fd_end_t = ::GetTickCount();

			
			start_t_slam = fd_end_t;//::GetTickCount();

			// 4. Insert node into node-graph to compute the pose of the node
			double xyz[3]={0,0,0};
			double rpy[3]={0,0,0};
			if(computeNodePose(node_ptr,xyz,rpy)){
					// Record current pose info
					CPose3D pose(rpy[2],rpy[1],rpy[0],xyz[0],xyz[1],xyz[2]);
					
					if(robotpath.size()==0 || !IsNoiseLocation(*robotpath.rbegin(),pose)){
						boost::shared_ptr<CPose3D> final_pose(new CPose3D(pose));
						robotpath.push_back(pose);
						pose.output(std::cout);

						// Logfile for slam_process
						end_t_slam = ::GetTickCount(); // end of SLAM for a valid frame
						sprintf(buf,"%d \t%.2f ",frame_num++,end_t_slam-start_t_slam);
						std::string slamstr(buf);
						mylogfile.writeintolog(slamstr);
						
						mylogfile.writeintolog(data_start_t,data_end_t);
						mylogfile.writeintolog(fd_start_t,fd_end_t);				
						
						// Send point_cloud and pose to Mapbuilder
						while(!pMapbuilder->setrawmapwithtransform(point_cloud,final_pose)){
							boost::this_thread::yield();
						}
				}
					// noisy translation
					else{
						// delete last noisy frame
						m_graph_mgr.deleteLastFrame();
						cout<<"delete noisy plane!"<<endl;
						delete node_ptr;
						continue;
					}
			}
			else{
				cout<<"Abandon this node!"<<endl;
				delete node_ptr;
				continue;
			}		
		}
	}
		
}

*/
void CVisualSlam::run(){

	boost::shared_ptr<openni_wrapper::Image> openni_image;
	boost::shared_ptr<openni_wrapper::DepthImage> openni_depth_image;
	boost::shared_ptr<pointcloud_type> point_cloud;

	// Create cv mat image with the right size
	cv::Mat cvRGBImage(480, 640, CV_8UC3);
	cv::Mat cvGrayImage(480, 640,CV_8UC1);
	cv::Mat cvDepthImage(480, 640, CV_16UC1);
	cv::Mat cvDepth8UC1Image(480, 640, CV_8UC1);

	// 1. Initialize OpenNI Graber and Mapbuilder
	SimpleOpenNIViewer v;
	//v.startframe();
	v.start();

	boost::shared_ptr<MapBuilder> pMapbuilder(new MapBuilder);

	// Get OpenNI visual image
	
	static unsigned rgb_array_size = 0;
	static boost::shared_array<unsigned char> rgb_array(0);
	static unsigned char* rgb_buffer = 0;

	rgb_array_size = 480*640*3; // size of each frame
	rgb_array.reset(new unsigned char [rgb_array_size]);
	rgb_buffer = rgb_array.get();
	
	// Get OpenNI depth image
	static unsigned depth_array_size = 0;
	static boost::shared_array<unsigned char> depth_array(0);
	static unsigned char* depth_buffer = 0;

	depth_array_size = 480*640*2;
	depth_array.reset(new unsigned char [depth_array_size]);
	depth_buffer = depth_array.get();

	// record num of frames
	static int frame_num = 0;
	double start_t_slam, end_t_slam;  // record SLAM time-consuming
	char buf[100]; // for receive char* from sprintf

	while(1){
		while(v.getCloudPoint( point_cloud))
		{
			/*xn::ImageMetaData xnImd;
			xnImd.CopyFrom(openni_image->getMetaData());*/

			// record start time of SLAM
			start_t_slam = ::GetTickCount();

			// get depth_image and image_matadata
			getImagesandDepthMetaData(point_cloud,rgb_buffer, depth_buffer);
			//openni_depth_image->fillDepthImageRaw(640,480,depth_buffer) ;
			//openni_image->fillRGB(640,480,rgb_buffer);

			// Copy RGB data from OpenNI image to cv mat image
			memcpy(cvRGBImage.data,rgb_buffer /*xnImd.Data()*/, 640*480*3);
			memcpy(cvDepthImage.data, depth_buffer, 640*480*2);

			//memcpy(cvGrayImage.data, xnImd.Grayscale8Data(), 640*480*1);
			//memcpy(cvDepthImage.data, xnDmd.Data(), 640*480*2);
			// Convert RGB image to grayscale image
			//cvRGBImage.convertTo(cvGrayImage, CV_8UC1);
			cvDepthImage.convertTo(cvDepth8UC1Image, CV_8UC1);

			// 3. Create node to extract feature and wrap the image and feature data
			Node* node_ptr = createFeatureNode(cvRGBImage, point_cloud, cvDepth8UC1Image);

			// filter frame that obtains too small features
			if(node_ptr->feature_locations_2d_.size() < features_threshold){
				cout<<" this frame has too small features! drop it!"<<endl;
				continue;
			}

			// 4. Insert node into node-graph to compute the pose of the node
			//double xyz[3]={0,0,0};
			//double rpy[3]={0,0,0};
			//needs to be done here as the graph size can change inside this function
			node_ptr->id_ = m_graph_mgr.graph_.size();

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
				// search robot_path to find most similar Node
				//Node* pre_node = m_graph_mgr.graph_[m_graph_mgr.graph_.size()-1];
				//MatchingResult mr = node_ptr->matchNodePair(pre_node);
				//boost::shared_ptr<CPose3D> pose(new CPose3D(mr.final_trafo)); // relative Pose to previous frame

				// find pose relative to robot_path[index] 
				boost::shared_ptr<CPose3D> pose(new CPose3D);
				int index = SearchAllPath(node_ptr,pose); 
				if(index == -1) 
				{
					//cout<<"failed to find a valid match from robot_path!"<<endl;
					delete node_ptr;
					continue;

				}
				if(IsBigTra(*pose)){
					isGoodTra = true; // This is good transformation, 
					
					//boost::shared_ptr<CPose3D> prepose = *robotpath.rbegin();
					//CPose3D prepose = *robotpath.rbegin();
					CPose3D prepose = robotpath[robotpath.size() - index];
					*pose += prepose ; // compute pose relative to global coordinates
					pose.get()->output(std::cout);
					final_pose = pose;
	
					// record this location
					node_ptr->buildFlannIndex();
					m_graph_mgr.graph_[node_ptr->id_] = node_ptr;
					robotpath.push_back(*pose);				
				}
				else
					delete node_ptr;
			}
			if(isGoodTra) // will display this frame
			{
				end_t_slam = ::GetTickCount(); // end of SLAM for a valid frame
				sprintf(buf,"%d SLAM :%f\t",frame_num++,end_t_slam-start_t_slam);
				std::string slamstr(buf);
				mylogfile.writeintolog(slamstr);

				// save this frame into files for debug
				string filename;
				static int num_frame = 0;
				addFeatureToImage(node_ptr,rgb_buffer);
				saveImgeToBmp(filename,++num_frame,rgb_buffer);

				while(!pMapbuilder->setrawmapwithtransform(point_cloud,final_pose)){
					boost::this_thread::yield();
				}
				isGoodTra = false;
			}

		}
	}
	/*		if(computeNodePose(node_ptr, xyz, rpy)){

				if(robotpath.size() == 0){
					
				}*/
			
	//		// use adapted rgbdslam 

	//		dst_ptr->buildFlannIndex();

	//		MatchingResult mr = src_ptr->matchNodePair(dst_ptr);

	//		CPose3D pose(mr.final_trafo);
	//		//cout<<pose;
	//		pose.output(std::cout);

	//			//	string filename;
	//			//	static int num_frame = 0;
	//			//	addFeatureToImage(node_ptr,rgb_buffer);
	//			//	saveImgeToBmp(filename,++num_frame,rgb_buffer);


	//			// after feature match
	//			showxyzrpy(xyz,rpy);


	//			// just for debug

	//			/*static string file_name("D:\\PCL_install_on_VS2008\\PCL-No_CMake\\PCL-1.1.1-Source\\PCL-1.1.1-Source\\bin32\\rgdbslam\\featurebmp\\");
	//			static int num_frame = 0;
	//			char c = '0' + (++num_frame);
	//			string filename = file_name + c +".pcd"; */
	//			// here we will save the cloud_point as for debugging
	//			/*	const pcl::PointCloud<point_type>& py = *point_cloud.get();
	//			pcl::io::savePCDFileBinary<point_type>(filename,*point_cloud.get());*/

	//			

	//			// 5. compute current pose relative to global coordinates
	//			boost::shared_ptr<CPose3D> current(new CPose3D(rpy[2],rpy[1],rpy[0],xyz[0],xyz[1],xyz[2]));
	//		//	*current += *robotpath.rbegin(); // this is already global pose info, so, 
	//			robotpath.push_back(*current);

	//			// see what changed from CPose3D
	//			/*current->getYawPitchRoll(rpy[2],rpy[1],rpy[0]);
	//			current->getXYZ(xyz);*/

	//			//showxyzrpy(xyz,rpy);
	//
	//			// show transformation matrix
	//			//displayResult(current);

	//			// 6. set raw points and pose info to map-builder
	//			while(!pMapbuilder->setrawmapwithtransform(point_cloud,current)){
	//				boost::this_thread::yield();
	//			}
	//			//// 6. set translated points
	//			//while(!pMapbuilder->setMap(point_cloud)){
	//			//	boost::this_thread::yield();
	//			//}
	//		}
	//		else
	//			delete node_ptr;
	//	}

	//}

}

void CVisualSlam::test(){
	string srcf("D:\\PCL_install_on_VS2008\\PCL-No_CMake\\PCL-1.1.1-Source\\PCL-1.1.1-Source\\bin32\\rgdbslam\\featurebmp\\2.pcd");
	string dstf("D:\\PCL_install_on_VS2008\\PCL-No_CMake\\PCL-1.1.1-Source\\PCL-1.1.1-Source\\bin32\\rgdbslam\\featurebmp\\1.pcd");

	boost::shared_ptr<pointcloud_type> src_cloud(new pointcloud_type);
	boost::shared_ptr<pointcloud_type> dst_cloud(new pointcloud_type);
	
	pcl::io::loadPCDFile(srcf,*src_cloud);
	pcl::io::loadPCDFile(dstf,*dst_cloud);

	// Create cv mat image with the right size
	cv::Mat cvRGBImage(480, 640, CV_8UC3);
	cv::Mat cvDepthImage(480, 640, CV_16UC1);
	cv::Mat cvDepth8UC1Image(480, 640, CV_8UC1);

	// Get OpenNI visual image
	static boost::shared_array<unsigned char> src_rgb_array(0);
	src_rgb_array.reset(new unsigned char[480*640*3]);
	static unsigned char* src_rgb_buffer = src_rgb_array.get();

	static boost::shared_array<unsigned char> dst_rgb_array(0);
	dst_rgb_array.reset(new unsigned char[480*640*3]);
	static unsigned char* dst_rgb_buffer = dst_rgb_array.get();

	// Get OpenNI depth image
	static unsigned depth_array_size = 0;
	static boost::shared_array<unsigned char> depth_array(0);
	static unsigned char* depth_buffer = 0;

	depth_array_size = 480*640*2;
	depth_array.reset(new unsigned char [depth_array_size]);
	depth_buffer = depth_array.get();

	// generate Node src
	getImagesandDepthMetaData(src_cloud,src_rgb_buffer, depth_buffer);
	memcpy(cvRGBImage.data,src_rgb_buffer, 640*480*3);
	memcpy(cvDepthImage.data, depth_buffer, 640*480*2);
	cvDepthImage.convertTo(cvDepth8UC1Image, CV_8UC1);
	Node* src_ptr = createFeatureNode(cvRGBImage, src_cloud, cvDepth8UC1Image);
	src_ptr->buildFlannIndex();

	// generate Node dst
	getImagesandDepthMetaData(dst_cloud,dst_rgb_buffer, depth_buffer);
	memcpy(cvRGBImage.data,dst_rgb_buffer, 640*480*3);
	memcpy(cvDepthImage.data, depth_buffer, 640*480*2);
	cvDepthImage.convertTo(cvDepth8UC1Image, CV_8UC1);
	Node* dst_ptr = createFeatureNode(cvRGBImage, dst_cloud, cvDepth8UC1Image);
	dst_ptr->buildFlannIndex();

	double xyz[3] = {0,0,0};
	double rpy[3] = {0,0,0};
	
	// using Hogman method! 
	bool bmr = computeNodePose(dst_ptr,xyz,rpy);
	bmr = computeNodePose(src_ptr,xyz,rpy);

//	MatchingResult mr = src_ptr->matchNodePair(dst_ptr);

	CPose3D pose(rpy[2],rpy[1],rpy[0],xyz[0],xyz[1],xyz[2]);
//	CPose3D pose(2*rpy[2],2*rpy[1],2*rpy[0],xyz[0],xyz[1],xyz[2]);
//	CPose3D pose(mr.final_trafo);
	pose.output(std::cout);

	MapBuilder mapbuilder;
	mapbuilder.global_map = dst_cloud;
	mapbuilder.weighted.resize(mapbuilder.global_map->points.size(),1);
	//mapbuilder._pose  = pose;
	boost::shared_ptr<CPose3D> npose(new CPose3D(pose));
	mapbuilder.cloudRegistration(npose,src_cloud);
	mapbuilder.local_map = src_cloud;
	
	//mapbuilder.MergeLocalMap();
	//mapbuilder.viewer.showCloud(mapbuilder.global_map);
	
	mapbuilder.fromPCtoCells(mapbuilder.local_map);
	mapbuilder.fromPCtoCells(mapbuilder.global_map);
	mapbuilder.viewer.showCloud(mapbuilder.global_cell_map);

	while(!mapbuilder.viewer.wasStopped())
	{
		boost::this_thread::yield();
	}

	delete src_ptr;
	delete dst_ptr;

}


/*
void main(){
	//	SimpleOpenNIViewer v;
	//	openni_wrapper::Image openni_image;
	//const xn::ImageMetaData xnImd;
	//const xn::DepthMetaData xnDmd;
	//// Get OpenNI visual image
	//// Get OpenNI depth image

	//// Create cv mat image with the right size
	//cv::Mat cvRGBImage(480, 640, CV_8UC3);
	//cv::Mat cvGrayImage(480, 640,CV_8UC1);
	//cv::Mat cvDepthImage(480, 640, CV_16UC1);
	//cv::Mat cvDepth8UC1Image(480, 640, CV_8UC1);
	//// Copy rgb data from OpenNI image to cv mat image
	//memcpy(cvRGBImage.data, xnImd.Data(), 640*480*3);
	//memcpy(cvDepthImage.data, xnDmd.Data(), 640*480*2);

	//// Convert RGB image to grayscale image
	//cvRGBImage.convertTo(cvGrayImage, CV_8UC1);
	//cvDepthImage.convertTo(cvDepth8UC1Image, CV_8UC1); 

	CVisualSlam slam("SURF", "SURF", "BruteForce");
	//CVisualSlam slam("FAST", "SURF", "BruteForce");
	//slam.run();
	//SimpleOpenNIViewer v;
	//v.start();
	//slam.test();
	slam.runHogman();
}*/