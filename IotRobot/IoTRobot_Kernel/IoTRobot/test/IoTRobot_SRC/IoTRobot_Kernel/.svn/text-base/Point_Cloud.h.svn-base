#pragma once
#include "Storage.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Core>
#include "pcl/common/common_headers.h"
#include "InternalDefine.h"
#include <vector>
//#include "pcl/registration/impl/transforms.hpp"


class CPoint_Cloud:public CStorage
{
public:
	CPoint_Cloud();
	~CPoint_Cloud();

//	boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > &m_cloud;
	Eigen::Matrix4f *m_pTransMat;
	ClassPtrs m_stClassPtrs;
	int NewFrameCloudIn(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > &cloud,Eigen::Matrix4f *pTransMat);
	void HandleCloud();
	virtual void GetData(int nPos,char *pcContent);
	virtual void SetData(int nPos,char *pcContent);

	unsigned char *m_pucRGB24;
	void ExternalCall(unsigned char *pucRGB24);


	//vector<boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>>> m_vPointCloudVec;
	//vector<Eigen::Matrix4f *> m_vTransVec;
	Eigen::Matrix4f m_TransMat;
	boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> m_PointCloud;
	bool m_bNewFrame;


	static UINT ThreadHandlePointCloud(LPVOID lpParam); 

	int PointCloudInit();
	int PointCloudRun();
	int PointCloudUninit();
protected:
private:
};