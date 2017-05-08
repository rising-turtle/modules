#ifndef VIEWER_H
#define VIEWER_H

#include <boost/thread/thread.hpp>
#include <boost/shared_ptr.hpp>
#include "pcl/common/common_headers.h"
#include "pcl/common/common_headers.h"
#include "pcl/features/normal_3d.h"
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/interactor_style.h>
#include <pcl/visualization/cloud_viewer.h>
#include <vector>
#include "CPose3D.h"
#include "pcl/registration/ia_ransac.h"
#include "CLogfile.h"

#define RX 1200 // X [-6 6]
#define RY 400	// Y [-2 2]
#define RZ 1200 // Z [-6 6]

#define CELLSIZE 4
#define X_CELL RX/CELLSIZE // 400
#define Y_CELL RY/CELLSIZE // 120
#define Z_CELL RZ/CELLSIZE // 400

#define X_STEP Y_CELL*Z_CELL // 120*400
#define Y_STEP Z_CELL // 400

#define ALL_CELLS X_CELL*Y_CELL*Z_CELL // 


class MapBuilder{
		
public:
	// to ues this type in .cpp
	typedef pcl::PointXYZRGB point_type;
	typedef pcl::PointCloud<point_type> pointcloud_type;

	MapBuilder(): viewer("Map Viewer"),new_map(false),blocked(false),
		global_map(new pcl::PointCloud<pcl::PointXYZRGB>),
		global_cell_map(new pcl::PointCloud<pcl::PointXYZRGB>),
		min_points_distance(0.0004) // 2cm * 2cm as the same point
	{
		boost::thread (boost::ref (*this));
		global_cells.resize(ALL_CELLS);
	}
	~MapBuilder(){}
	
	// set local_map, if succeed return true, else return false;
	// this function should be called by father thread
	bool setMap(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pmap){ 
		static int i=0;
		mutex_map.lock();
		if(blocked)
		{
			mutex_map.unlock();
			return true;
		}
		blocked = !blocked;
		new_map = true;
		local_map = pmap;
		cout<<"set "<<++i<<" th map"<<endl;
		mutex_map.unlock();
		return true;
	}

	// interface for input raw_cloudpoint and CPose3D
	bool setrawmapwithtransform(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& rawmap, boost::shared_ptr<CPose3D>& pose){
		static int i=0;
		mutex_map.lock();
		//if(blocked) // synchronization with slam
		//{
		//	mutex_map.unlock();
		//	return false;
		//}
		//blocked = !blocked;
		new_map = true;
		//local_map = rawmap;
		local_map_set.push_back(rawmap);
		//_pose = pose;
		_pose_set.push_back(pose);
		cout<<"set "<<++i<<" th map"<<endl;
		mutex_map.unlock();
		return true;
	}
	void displayResult(boost::shared_ptr<CPose3D> & pose){

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
	void cloudRegistration(boost::shared_ptr<CPose3D>& pose, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
	{
		// --------------------------------------------------------------------------
		//  SPECIAL CASE OF HORIZONTAL SCAN: QUICKER IMPLEMENTATION
		// --------------------------------------------------------------------------
		Eigen::Matrix4f	HM;
		pose->getHomogeneousMatrix(HM);

		/*using pcl::transformation instead*/ 
		pcl::transformPointCloud (*cloud, *cloud, HM);
	}

	// Index from (x,y,z)
	inline int getIndexCell(float& x,float& y, float& z){
		if(!pcl_isfinite(x) || !pcl_isfinite(y) || !pcl_isfinite(z))
			return -1;
		if(fabs(x) >= 10 || fabs(z) >= 10 || y<=-1 || y>=3 )
			return -1;
		int lx = ( x*100 + RX/2) / CELLSIZE;
		int ly = ( y*100 + RY/2) / CELLSIZE;
		int lz = ( z*100 + RZ/2) / CELLSIZE;
		return (lx*X_STEP + ly*Y_STEP + lz);
	}

	// erase Infinite points
	void eraseInfinitePoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pCloud);
	// Copy from CPointCloud to Cells
	void fromPCtoCells(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pCloud);
	// Show globalCells
	void ShowGlobalCells(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pCloud);
	bool IsNoisePoint(int index,boost::shared_ptr<pcl::PointXYZRGB>& p);

public:
	pcl::visualization::CloudViewer viewer;

	 // thread synchronization
	 boost::mutex mutex_map;
	 bool blocked; // if this thread is showing cloud, then, cannot obtain a new map
	 bool new_map ; // indicate whether a new map has been obtained
	 boost::shared_ptr<CPose3D> _pose; // for cloud registration
	 pcl::PointCloud<pcl::PointXYZRGB>::Ptr local_map; //current frame map after slam
	 std::list< pcl::PointCloud<pcl::PointXYZRGB>::Ptr> local_map_set; // all the local maps that should be displayed
	 std::list< boost::shared_ptr<CPose3D> > _pose_set; // for cloud registration
	 pcl::PointCloud<pcl::PointXYZRGB>::Ptr global_map; // global map
	 std::vector<int> weighted; // use MRPT weighted point Alignment
	 float min_points_distance; // determine whether these two points are the same point

	 pcl::PointCloud<pcl::PointXYZRGB>::Ptr global_cell_map; // to display cells
	 vector<boost::shared_ptr<pcl::PointXYZRGB> > global_cells; // all the discrete cells 5*5*5

	 // This is for recording mapbuilder-time-consumed
	 CLogfile mylogfile;

public:
	void operator() (){  // thread function
		while(!viewer.wasStopped()){
			mutex_map.lock();
			if(!new_map) // local map has not been updated
			{
				mutex_map.unlock();
				boost::thread::yield();
			}
			else{
				if(local_map_set.size() == 0)
				{
					new_map = false;
					mutex_map.unlock();
					continue;
				}

					static int i = 0;
					//	blocked = false;
					
					// start of mapbuilder
					double start_t_mapbuilder = ::GetTickCount();

					// get last point_cloud and pose, then delete this Node info
					local_map = *(local_map_set.begin());
					_pose = *(_pose_set.begin());
					
					// delete this Node info
					local_map_set.pop_front();
					_pose_set.pop_front();
				
					mutex_map.unlock();

					// this step is only for registration raw_cloudpoint
					cloudRegistration(_pose,local_map);
					// MergeLocalMap();
					fromPCtoCells(local_map); // add this local_map into global_cells
					viewer.showCloud(global_cell_map);

					// end of mapbuilder
					double end_t_mapbuilder = ::GetTickCount();
					char buf[100];
					sprintf(buf,"\t %0.2f",end_t_mapbuilder - start_t_mapbuilder);
					string str_builder(buf);
					mylogfile.writeintolog(str_builder,true); // write time cost by mapbuilder

					//viewer.showCloud(global_map);
					//mutex_map.unlock();
			}
		}
	}

public:

	// Merge local with global 
	void MergeLocalMap(){
		if(global_map->points.size()==0)// this is the first frame
		{
			global_map = local_map;
			weighted.resize(global_map->size(),1);
			return ;
		}

		// Use a KdTree to search for the nearest matches in feature space
		pcl::KdTreeFLANN<pcl::PointXYZRGB> descriptor_kdtree;
		descriptor_kdtree.setInputCloud (global_map);

		// Find the index of the best match for each keypoint, and store it in "correspondences_out"
		const int k = 1;
		std::vector<int> k_indices (k);
		std::vector<float> k_squared_distances (k);

		for (size_t i = 0; i < local_map->points.size (); ++i)
		{
			descriptor_kdtree.nearestKSearch (*local_map, i, k, k_indices, k_squared_distances);

			if(k_squared_distances[0] < min_points_distance) // if this two points are the same point
			{
				pcl::PointXYZRGB& tp = global_map->points[k_indices[0]];
				pcl::PointXYZRGB& sp = local_map->points[i];
				float factor = 1.0/(float)(weighted[k_indices[0]] + 1);
				tp.x = weighted[k_indices[0]]*tp.x * factor + sp.x * factor;
				tp.y = weighted[k_indices[0]]*tp.y * factor + sp.y * factor;
				tp.z = weighted[k_indices[0]]*tp.z * factor + sp.z * factor;
				weighted[k_indices[0]] ++;
			}
			else // this point is a new point
			{
				global_map->points.push_back(local_map->points[i]);
				weighted.push_back(1);
			}
		}	
	}
};

#endif