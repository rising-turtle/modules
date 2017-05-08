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
#include "FileConfig.h"

//#define features_threshold 50	// if a new frame has less than this,then drop it!
//#define search_range 2			// back search units

#define FEATURES_THRESHOLD 50

//#pragma pack(1)
typedef struct _XYZPt{
	float x; 
	float y; 
	float z;
} XYZPt;

// Descriptor is CV::MAT(N,64,float), N depends on the number of features.(We use ONLY non-extended surf descriptor)
// 3D Descriptor is the combination of 3D pose and descrpitor of features.
// The serialization of 3D Descriptor is as follows:
// (x0,y0,z0),(0,0,float),(0,1,float),...,(0,63,float); (x1,y1,z1),(1,0,float),(1,1,float),...,(1,63,float);... ;(xN-1,yN-1,zN-1),(N-1,0,float),(N-1,1,float),...,(N-1,63,float);
typedef struct _Feature3DDesc{
	XYZPt xyz;
	float desc[64];
} Feature3DDesc;

typedef struct _Node3DDesc{
	int id; // node ID
	int n; // the number of features
	Feature3DDesc desc3DList[0];
} Node3DDesc;
//#pragma pack()


class Iot_Robot_rgdbslam : public Iot_Robot_Slam
{
public:
	Iot_Robot_rgdbslam(const string& detectorType, const string& descriptorType, const string& descriptorMatcherType){
		// Load slam params from configuration file!!
		CFileConfig configSlam("D:\\IoTRobot.ini");
		if(configSlam.UpdateParamMap()){
			configSlam.UpdateGlobalParam();
		}
		setDetector(detectorType);
		setExtractor(descriptorType);
		setMatcher(descriptorMatcherType);
	}
	Iot_Robot_rgdbslam(){
		// Load slam params from configuration file!!
		CFileConfig configSlam("D:\\IoTRobot.ini");
		if(configSlam.UpdateParamMap()){
			configSlam.UpdateGlobalParam();
		}
		setDetector(global_feature_detector_type);
		setExtractor(global_feature_extractor_type);
		setMatcher("BruteForce");
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

	// save rgb_info in file 
	void fromrgbtobgr(unsigned char* rgb_buff, int len);
	void addFeatureToImage(Node* pNode, unsigned char* rgb_buff);
	void getFileName(int frame_num, std::string & filename);
	void convertToIplImage(boost::shared_ptr<IplImage> pImage,unsigned char * rgb_buff);
	void saveImgeToBmp(std::string& filename,int frame_num, unsigned char* rgb_buff);

	int RunOneFrameHogman(
		unsigned char *pucRGB,unsigned short *pusDepth,
		boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > &cloud,
		Eigen::Matrix4f &final_transformation,unsigned long *pulRunInfo,
		double *pdIMUdata, int status,bool& rebuild_map, vector<CPose3D>&, vector<int>&  ,int&, Node3DDesc** );


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
	void reset();
	void resetSession(CPose3D pose);

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
	CLogfile mylogfile;

};



#endif