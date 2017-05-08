#include "PlanarDetector.h"
#include <XnCppWrapper.h>


#define Epsilon 0.3
#define DownSampleScale 4

int height = 480/DownSampleScale;
int width = 640/DownSampleScale;

int centerX = (width >> 1);
int centerY = (height >> 1);
float bad_point = std::numeric_limits<float>::quiet_NaN();

float fx = 1.0/577.245962578894;
float fy = 1.0/580.481741642517;

PlanarDetector::PlanarDetector(void)
{
	pcl::PointCloud<PointT>::Ptr  cloud(new pcl::PointCloud<PointT> ());
	prev_cloud =cloud;
}

PlanarDetector::~PlanarDetector(void)
{
}

void PlanarDetector::ConvertToXYZRGBPointCloud(unsigned char*pucRGB,unsigned short *pusDepth,pcl::PointCloud<PointT>::Ptr cloud)
{
	cloud->header.frame_id = "/openni_rgb_optical_frame";
	cloud->height = height;
	cloud->width = width;
	cloud->is_dense = false;
	cloud->points.resize(cloud->height * cloud->width);
	register int centerX = (cloud->width >> 1);
	int centerY = (cloud->height >> 1);
	register const XnDepthPixel* depth_map = pusDepth; 
	register int color_idx = 0, depth_idx = 0;

	for (int v = -centerY; v < centerY; ++v)
	{
		for (register int u = -centerX; u < centerX; ++u, color_idx += 3, ++depth_idx)
		{
			PointT & pt = cloud->points[depth_idx];
			/// @todo Different values for these cases
			// Check for invalid measurements
			if (depth_map[depth_idx] == 0 )
			{
				pt.x = pt.y = pt.z = bad_point;
			}
			else
			{
				pt.z=pusDepth[depth_idx] * 0.001f;
				pt.x=fx*u *pt.z;
				pt.y=fy*v *pt.z;
				pt.r=pucRGB[color_idx];
				pt.g=pucRGB[color_idx+1];
				pt.b=pucRGB[color_idx+2];
				//cout<< " z " <<pt.z << " y " <<pt.y << " x  "<< pt.x <<endl;
				//cout<< " r " <<pt.r << " g " <<pt.g << " b  "<< pt.b <<endl;
			}
		}
	}
}

void PlanarDetector::ConvertContour(pcl::PointCloud<PointT>::Ptr plane, unsigned char*pucRGB,unsigned short *pusDepth,point *pusXY)
{
	float fx = 577.245962578894;
	float fy = 580.481741642517;
	fx = 1.0f/fx;
	fy = 1.0f/fy;
	int color_idx = 0, depth_idx = 0;
	//planepointnum=plane->size();
//	IplImage* img_small = cvCreateImage(cvSize(160,120), 8, 1);
//	cvZero(img_small);
	for (int i = 0; i<plane->size();++i,color_idx += 3, ++depth_idx)
	{
		PointT & pt = plane->points[i];
		if (pt.x == bad_point && pt.y==bad_point && pt.z==bad_point)
		{
			pusDepth[depth_idx] = 0;
			pusXY[depth_idx].x=(pt.x/(pt.z*fx))+centerX;
			pusXY[depth_idx].y=(pt.y/(pt.z*fy))+centerY;
			pucRGB[color_idx+0] = pt.r;
			pucRGB[color_idx+1]=pt.g;
			pucRGB[color_idx+2]=pt.b;
		}
		else
		{
			pusXY[depth_idx].x=(pt.x/(pt.z*fx))+centerX;
			pusXY[depth_idx].y=(pt.y/(pt.z*fy))+centerY;
			/*if(i<10000)
			{
			planepoint[i].x=pusXY[depth_idx].x;
			planepoint[i].y=pusXY[depth_idx].y;
			}*/
			//cout <<"  i   " <<i << "x   "<<pusXY[depth_idx].x << "   y   " << pusXY[depth_idx].y<<endl;
			if (pusXY[depth_idx].x>width || pusXY[depth_idx].x<0 ||pusXY[depth_idx].y>height ||pusXY[depth_idx].y<0)
			{
				cout<< " error" <<endl;
			}
			pusDepth[depth_idx] = pt.z *1000;
			pucRGB[color_idx+0] = pt.r;
			pucRGB[color_idx+1]=pt.g;
			pucRGB[color_idx+2]=pt.b;
		}
	

	//	int offset = pusXY[depth_idx].y * width + pusXY[depth_idx].x;
	//	img_small->imageData[offset] = 255;
		//recover
		pusXY[depth_idx].x = DownSampleScale*pusXY[depth_idx].x;
		pusXY[depth_idx].y = DownSampleScale*pusXY[depth_idx].y;
	}

}


bool PlanarDetector:: run (unsigned char*pucRGB,unsigned short *pusDepth,float SensorPose[3])//,Eigen::Vector3f SensorPose)
{
	planeContours.clear();
	bool isFloor = false;
	ConvertToXYZRGBPointCloud(pucRGB, pusDepth,prev_cloud);

	pcl::IntegralImageNormalEstimation<PointT, pcl::Normal> ne;
	ne.setNormalEstimationMethod (ne.COVARIANCE_MATRIX);
	ne.setMaxDepthChangeFactor (0.03f);
	ne.setNormalSmoothingSize (20.0f);

	pcl::OrganizedMultiPlaneSegmentation<PointT, pcl::Normal, pcl::Label> mps;
	mps.setMinInliers (50);
	mps.setAngularThreshold (0.017453 * 2.0); //3 degrees
	mps.setDistanceThreshold (0.02); //2cm

	std::vector<pcl::PlanarRegion<PointT>, Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > > regions;
	std::vector<pcl::ModelCoefficients> model_coefficients;
	std::vector<pcl::PointIndices> inlier_indices;
	pcl::PointCloud<pcl::Label>::Ptr labels (new pcl::PointCloud<pcl::Label>);
	std::vector<pcl::PointIndices> label_indices;
	std::vector<pcl::PointIndices> boundary_indices;

	pcl::PointCloud<PointT>::Ptr contour (new pcl::PointCloud<PointT>);
	char name[1024];
	regions.clear();
	pcl::PointCloud<pcl::Normal>::Ptr normal_cloud (new pcl::PointCloud<pcl::Normal>);
	double normal_start = pcl::getTime ();
	ne.setInputCloud (prev_cloud);
	ne.compute (*normal_cloud);
	double normal_end = pcl::getTime ();
	//std::cout << "Normal Estimation took " << double (normal_end - normal_start) << std::endl;

	double plane_extract_start = pcl::getTime ();
	mps.setInputNormals (normal_cloud);
	mps.setInputCloud (prev_cloud);
	//mps.segmentAndRefine (regions);

	mps.segmentAndRefine (regions, model_coefficients, inlier_indices, labels, label_indices, boundary_indices);


	double plane_extract_end = pcl::getTime ();
	//std::cout << "Plane extraction took " << double (plane_extract_end - plane_extract_start) << std::endl;
	//std::cout << "Frame took " << double (plane_extract_end - normal_start) << std::endl;

	pcl::PointCloud<PointT>::Ptr cluster (new pcl::PointCloud<PointT>);

	for (size_t i = 0; i < regions.size (); i++)
	{
		Eigen::Vector3f centroid = regions[i].getCentroid ();
		Eigen::Vector4f model = regions[i].getCoefficients ();
		Eigen::Vector3f Normal;
		Normal[0] = model[0];
		Normal[1] = model[1];
		Normal[2] = model[2];
		pcl::PointXYZ pt1 = pcl::PointXYZ (centroid[0], centroid[1], centroid[2]);
		pcl::PointXYZ pt2 = pcl::PointXYZ (centroid[0] + (0.5f * model[0]),centroid[1] + (0.5f * model[1]),centroid[2] + (0.5f * model[2]));

		bool flag = SearchFloor(SensorPose,centroid,Normal);
		contour->points = regions[i].getContour ();
		if (flag)
		{
			isFloor = flag;
		}

		//PlaneContour RegionContour;
		PlaneContour *RegionContour = new PlaneContour(); 
		RegionContour->isFloor = flag;
		RegionContour->planeContour = contour;
		RegionContour->centroid = centroid;
		RegionContour->planepointnum = contour->size();
		RegionContour->inliers = inlier_indices[i];
		RegionContour->count = regions[i].getCount();
		TransformPoint2Original(RegionContour);
		ConvertContour(contour,RegionContour->ContourRGB,RegionContour->ContourDepth,RegionContour->ContourXY);
		planeContours.push_back(RegionContour);
	}
	return isFloor;
}


bool PlanarDetector::SearchFloor(float SensorPose[3], Eigen::Vector3f centroid, Eigen::Vector3f normal)
{
	bool isFloor = false;
	Eigen::Vector3f  TransNormalP = TransformAxis2Ground(normal,SensorPose);
	Eigen::Vector3f floor(1,0,1);
	float value = TransNormalP[0]+TransNormalP[2];
	if (abs(value) <= Epsilon && abs(TransNormalP[0])<=Epsilon &&abs(TransNormalP[2]<=Epsilon) && TransNormalP[1]<0)                                       
	{
		isFloor = true;
		//cout << " x           "<< TransNormalP[0] << "    y       " << TransNormalP[1] << "                z      "<<TransNormalP[2] <<endl;
	}
	return isFloor;
}

Eigen::Vector3f PlanarDetector::TransformAxis2Ground(Eigen::Vector3f point, float SensorPose[3])
{
	//pcl::PointXYZ transPoint;
	float theta = SensorPose[0]; //pitch
	float pusai = SensorPose[1];//yaw
	float phi = SensorPose[2];//roll
	Eigen::Matrix3f Rotation;
	Rotation(0,0) = sin(phi)*sin(theta)*sin(pusai)+cos(phi)*cos(pusai);
	Rotation(0,1) = cos(phi)*sin(theta)*sin(pusai)-sin(phi)*cos(pusai);
	Rotation(0,2) = cos(theta)*sin(pusai);
	Rotation(1,0) = sin(phi)*cos(theta);
	Rotation(1,1) = cos(phi)*cos(theta);
	Rotation(1,2) = -sin(theta);
	Rotation(2,0) = sin(phi)*sin(theta)*cos(pusai)-cos(phi)*sin(pusai);
	Rotation(2,1) = cos(phi)*sin(theta)*cos(pusai)+sin(phi)*sin(pusai);
	Rotation(2,2) = cos(phi)*cos(pusai);
	Eigen::Vector3f TransformPoint = Rotation*point;
	return TransformPoint;
}

void PlanarDetector::TransformPoint2Original(PlaneContour *plane)
{
	for (size_t i = 0; i<plane->inliers.indices.size();i++)
	{
		PointT & sp = prev_cloud->points[plane->inliers.indices[i]];
		int u = sp.x/(fx*sp.z)-1;
		int v = sp.y/(fy*sp.z)-1;
		int index = (v+centerY)*width+(u+centerX);
		plane->inliers.indices[i] = (v*4+centerY*DownSampleScale)*width*DownSampleScale+(u*4+centerX*DownSampleScale);
 		//cout << " index  " <<index << "   calculate  " <<plane->inliers.indices[i]<<endl;
	}
}