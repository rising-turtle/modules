#pragma once
#include <vector>
#include <boost/shared_ptr.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "cv.h"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
using namespace std;
class ReadGroundTrues
{
public:
	ReadGroundTrues(void);
	~ReadGroundTrues(void);

	vector<string> m_vctDFileList;
	vector<string> m_vctRGBFileList;
	int Init();
	int Run(unsigned char *pucRGB,unsigned short *pusD,boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > &m_PC);
	int CreateFileList(vector<string> &vctFileList,char *pcPath,char *pcMark,char *pcSuffix);

	void calPCfromImageAndDepth(cv::Mat& image, cv::Mat& depth, \
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr& outPC);
};
