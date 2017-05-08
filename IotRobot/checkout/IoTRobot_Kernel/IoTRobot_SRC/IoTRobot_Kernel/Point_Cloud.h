#pragma once
#include "Storage.h"
#include <vector>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Core>
#include "InternalDefine.h"

class CPose3D;

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


	// call for Registration
	void cloudRegistration(boost::shared_ptr<CPose3D>& pose, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud);

	//vector<boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>>> m_vPointCloudVec;
	//vector<Eigen::Matrix4f *> m_vTransVec;
	Eigen::Matrix4f m_TransMat;
	boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> m_PointCloud;
	bool m_bNewFrame;
	
	// id of each node
	int id_of_node;

	static UINT ThreadHandlePointCloud(LPVOID lpParam); 

	int PointCloudInit();
	int PointCloudRun();
	int PointCloudUninit();
protected:
private:
};