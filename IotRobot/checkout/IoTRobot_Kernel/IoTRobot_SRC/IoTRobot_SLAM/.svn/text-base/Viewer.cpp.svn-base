#include "Viewer.h"



// erase Infinite points
void MapBuilder::eraseInfinitePoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pCloud){
	/** \brief The point data. */
	std::vector<pcl::PointXYZRGB, Eigen::aligned_allocator<pcl::PointXYZRGB> >::iterator it = pCloud->points.begin();
	std::vector<pcl::PointXYZRGB, Eigen::aligned_allocator<pcl::PointXYZRGB> >::iterator it_begin = it;
	std::vector<pcl::PointXYZRGB, Eigen::aligned_allocator<pcl::PointXYZRGB> >::iterator it_end = it;

	int index = 0;
	bool flag = false;
	int num = 0; // record number of erased points
	for(size_t i=0; i< pCloud->points.size();  ){
		if (!pcl_isfinite ((*it).x) || 
			!pcl_isfinite ((*it).y) || 
			!pcl_isfinite ((*it).z))
		{
			if(!flag){ // first find begin of it
				flag = true;
				it_begin = it;
			}
			num ++;
			//it = pCloud->points.erase(it);
			it++;
			i++;
			continue;
		}
		if(flag) { // second find end of it
			it_end = it;
			flag = false;
			it = pCloud->points.erase(it_begin,it_end);
			i -= num;
			num = 0;
			continue;
		}
		it++;
		i++;
	}
}

bool MapBuilder::IsNoisePoint(int index,boost::shared_ptr<pcl::PointXYZRGB>& p){
	 
	 static float error_noise = 1e-2;
	 boost::shared_ptr<pcl::PointXYZRGB> sp;// = global_cells[i];
	 int l_x = index - X_STEP;
	 int r_x = index + Y_STEP;
	 int l_y = index - Y_STEP;
	 int r_y = index + Y_STEP;
	 int l_z = index - 1;
	 int r_z = index + 1;//

	 // we will search longer distance along Z-axis 
	 int range_z = 2;

	 if(l_x >=0 && l_x <ALL_CELLS){
		 sp = global_cells[l_x];
		 if(sp.get() != NULL)
			if(fabs(sp->rgb - p->rgb)<error_noise)
				 return true;
	 }
	 if(r_x >=0 && r_x <ALL_CELLS){
		 sp = global_cells[r_x];
		 if(sp.get() != NULL)
			if(fabs(sp->rgb - p->rgb)<error_noise)
				 return true;
	 }
	 if(l_y >=0 && l_y <ALL_CELLS){
		 sp = global_cells[l_y];
		  if(sp.get() != NULL)
			if(fabs(sp->rgb - p->rgb)<error_noise)
				 return true;
	 }
	 if(r_y >=0 && r_y <ALL_CELLS){
		 sp = global_cells[r_y];
		  if(sp.get() != NULL)
			if(fabs(sp->rgb - p->rgb)<error_noise)
				 return true;
	 }

	 for(int i= index - range_z; i<= index + range_z; i++)
	 {
		 if( i < 0 || i >=ALL_CELLS  )
			 continue;
		 sp = global_cells[i];
		 if(sp.get()!=NULL)
			 if(fabs(sp->rgb - p->rgb) < error_noise)
				 return true;
	 }
	 /*if(l_z >0 && l_z <ALL_CELLS){
		 sp = global_cells[l_z];
		  if(sp.get() != NULL)
			 if(fabs(sp->rgb - p->rgb)<error_noise)
				 return true;
	 }
	 if(r_z >0 && r_z <ALL_CELLS){
		 sp = global_cells[r_z];
		  if(sp.get() != NULL)
			if(fabs(sp->rgb - p->rgb)<error_noise)
				 return true;
	 }*/
	return false;
}

 // Copy from CPointCloud to Cells
void MapBuilder::fromPCtoCells(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pCloud){
	
	int N = pCloud->points.size();
	vector<bool> index_point(N,false);
	vector<int> index_set;


	for(size_t i=0;i<pCloud->points.size();i++)
	{
		pcl::PointXYZRGB& sp = pCloud->points[i];
		int index = getIndexCell(sp.x,sp.y,sp.z);
		if(index >=0 )
		{
			if(global_cells[index].get()!=NULL) // this is already painted
				continue; // not flush this point using the new point	
			boost::shared_ptr<pcl::PointXYZRGB> p(new pcl::PointXYZRGB);
			p->x = sp.x; p->y = sp.y; p->z = sp.z; 
			p->r = sp.r; p->g = sp.g; p->b = sp.b;
			if(!IsNoisePoint(index,p))
			{
				index_point[i] = true;
				index_set.push_back(index);
			}
			//global_cells[index] = p;
			//global_cell_map->points.push_back(sp);
			/*global_cells[index].r = sp.r;
			global_cells[index].g = sp.g;
			global_cells[index].b = sp.b;
			global_cells[index].x = sp.x;
			global_cells[index].y = sp.y;
			global_cells[index].z = sp.z;
			global_cells[index]._unused = 1;*/
		}
	}
	for(size_t i=0,j=0;i<N;i++)
		if(index_point[i])
		{
			pcl::PointXYZRGB& sp = pCloud->points[i];
			boost::shared_ptr<pcl::PointXYZRGB> p(new pcl::PointXYZRGB);
			p->x = sp.x; p->y = sp.y; p->z = sp.z; 
			p->r = sp.r; p->g = sp.g; p->b = sp.b;
			int index_cell = index_set[j++];
			if(global_cells[index_cell].get()== NULL){
				global_cells[index_cell/*index_set[j++]*/] = p;
				global_cell_map->points.push_back(sp);
			}
		}
}
// Show globalCells
void MapBuilder::ShowGlobalCells(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pCloud){
	for(size_t i=0;i<global_cells.size();i++){
		boost::shared_ptr<pcl::PointXYZRGB> sp = global_cells[i];
		if(sp.get() == NULL)
			continue;
		pCloud->points.push_back(*sp);
	}
}
