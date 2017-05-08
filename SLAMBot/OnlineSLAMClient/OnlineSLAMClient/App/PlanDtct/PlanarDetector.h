#pragma once
#include <boost/thread/thread.hpp>

#include <pcl/common/time.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/ModelCoefficients.h>
#include <boost/shared_ptr.hpp>
#include <pcl/segmentation/planar_region.h>
#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl/segmentation/organized_connected_component_segmentation.h>
#include <pcl/filters/extract_indices.h>


#define MaxSize 100*100

typedef pcl::PointXYZRGBA PointT;

using namespace std;
typedef struct point
{
	int x;
	int y;
};
typedef struct PlaneContour
{
	unsigned char ContourRGB[MaxSize];
	unsigned short ContourDepth[MaxSize];
	point ContourXY[MaxSize];
	Eigen::Vector3f centroid;
	bool isFloor;
	int planepointnum;
	pcl::PointIndices inliers;
	int count;
	pcl::PointCloud<PointT>::Ptr planeContour;//(new pcl::PointCloud<PointT>);
	pcl::PointCloud<PointT>::Ptr planePoint;
};

class PlanarDetector
{
public:
	PlanarDetector(void);
	~PlanarDetector(void);
	void ConvertToXYZRGBPointCloud(unsigned char*pucRGB,unsigned short *pusDepth,pcl::PointCloud<PointT>::Ptr cloud);
	void ConvertContour(pcl::PointCloud<PointT>::Ptr plane, unsigned char*pucRGB,unsigned short *pusDepth,point *pusXY);
	bool run (unsigned char*pucRGB,unsigned short *pusDepth,float SensorPose[3]);
	bool SearchFloor(float SensorPose[3], Eigen::Vector3f centroid,  Eigen::Vector3f normal);
	void TransformPoint2Original(PlaneContour *plane);

	//boost::shared_ptr<pcl::visualization::PCLVisualizer> cloudViewer (pcl::PointCloud<PointT>::ConstPtr cloud);

	Eigen::Vector3f TransformAxis2Ground(Eigen::Vector3f point, float SensorPose[3]);
	pcl::PointCloud<PointT>::Ptr prev_cloud;
	vector<PlaneContour*> planeContours;

	//boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

};
