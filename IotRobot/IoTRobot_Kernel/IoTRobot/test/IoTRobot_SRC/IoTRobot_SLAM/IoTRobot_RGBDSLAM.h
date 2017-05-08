#ifndef IOT_ROBOT_RGBDSLAM
#define IOT_ROBOT_RGBDSLAM

#include "IoTRobot_SLAM.h"

#include <iostream>
#include <string>
#include <boost/shared_ptr.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/common/time.h>
#include "graph_manager.h"
#include "CPose3D.h"
#include "Viewer.h"

//#define features_threshold 50	// if a new frame has less than this,then drop it!
//#define search_range 2			// back search units

#define FEATURES_THRESHOLD 50

class Iot_Robot_rgdbslam : public Iot_Robot_Slam
{
public:
	Iot_Robot_rgdbslam(const string& detectorType, const string& descriptorType, const string& descriptorMatcherType){
		setDetector(detectorType);
		setExtractor(descriptorType);
		setMatcher(descriptorMatcherType);
	}
	~Iot_Robot_rgdbslam(){}

	// overload init() uninit() runoneframe()
	int init();
	int uninit(){return 0;};
	//void runoneframe(pcl::PointCloud<pcl::PointXYZRGB>::Ptr frame){};
	int runoneframe(
		unsigned char *pucRGB,unsigned short *pusDepth,
		boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > &cloud,
		Eigen::Matrix4f &final_transformation);


	int RunOneFrameHogman(
		unsigned char *pucRGB,unsigned short *pusDepth,
		boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > &cloud,
		Eigen::Matrix4f &final_transformation,unsigned long *pulRunInfo,double *pdIMUdata, int status);


	// parameters for feature extractor
	void setDetector(const string& detectorType){
		m_detector = cv::FeatureDetector::create(detectorType);
	};
	void setExtractor(const string& descriptorType){
		m_extractor = cv::DescriptorExtractor::create(descriptorType);
	};
	void setMatcher(const string& descriptorMatcherType){
		m_matcher = cv::DescriptorMatcher::create(descriptorMatcherType);
	};

	////Interfaces of opencv SIFT/SURF/FAST/STAR/GFTT algorithm
	//cv::FeatureDetector* createDetector( const string& detectorType );
	//cv::DescriptorExtractor* createDescriptorExtractor( const string& descriptorType );
	Node* createFeatureNode(const cv::Mat& visual , boost::shared_ptr<pointcloud_type>& point_cloud, const cv::Mat& depth = cv::Mat());
	bool computeNodePose(Node* node_ptr, double* xyz, double* rpy);

	// whether this pose is good: not noise nor too small
	bool IsBigTra(CPose3D & pose);
	bool IsNoiseLocation(CPose3D& lastpose,CPose3D& currentpose );
	// split CPointCloud into image and depth_image metadata
	void getImagesandDepthMetaData(boost::shared_ptr<pointcloud_type> point_cloud,
		unsigned char* rgbbuf,
		unsigned char* depthbuf
		);

	//OpenCV variables 
	cv::Ptr<cv::FeatureDetector> m_detector;
	cv::Ptr<cv::DescriptorExtractor> m_extractor;
	cv::Ptr<cv::DescriptorMatcher > m_matcher;

public:
	// Record pose queue
	vector<CPose3D> robotpath;

	// Map builder
	boost::shared_ptr<MapBuilder> pMapbuilder;

private:
	//
	GraphManager m_graph_mgr;


//SS Modification
private:
	cv::Mat m_cvRGBImage;
	cv::Mat m_cvGrayImage;
	cv::Mat m_cvDepthImage;
	cv::Mat m_cvDepth8UC1Image;

};



#endif