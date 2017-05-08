//#include "stdafx.h"
#include "MapBuilder.h"
#include "3DMap.h"


unsigned char *CMapBuilder::m_pucGetImgPtr;
CMapBuilder::CMapBuilder()
{
	m_pucGetImgPtr=0;
}


CMapBuilder::~CMapBuilder()
{

}

void CMapBuilder::GetRGB24Data(unsigned char *pucRGB24)
{
	//CPoint_Cloud::ExternalCall(pucRGB24);
}


int CMapBuilder::NewGlobalPCIn(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > &cloud)
{
	MergePointCloud2Area(cloud);
	return 0;
}



bool CMapBuilder::IsNoisePoint(int index,boost::shared_ptr<pcl::PointXYZRGB>& p){

	C3DMap *pc3Dmap=(C3DMap *)m_stClassPtrs.p3Dmap; 
	static float error_noise = 1e-2;
	boost::shared_ptr<pcl::PointXYZRGB> sp;// = global_cells[i];
	int l_x = index - X_STEP;
	int r_x = index + X_STEP;
	int l_y = index - Y_STEP;
	int r_y = index + Y_STEP;
	int l_z = index - 1;
	int r_z = index + 1;//

	// we will search longer distance along Z-axis 
	int range_z = 2;

	if(l_x >=0 && l_x <ALL_CELLS){
		sp =pc3Dmap->m_global_cells[l_x];
		if(sp.get() != NULL)
			if(fabs(sp->rgb - p->rgb)<error_noise)
				return true;
	}
	if(r_x >=0 && r_x <ALL_CELLS){
		sp = pc3Dmap->m_global_cells[r_x];
		if(sp.get() != NULL)
			if(fabs(sp->rgb - p->rgb)<error_noise)
				return true;
	}
	if(l_y >=0 && l_y <ALL_CELLS){
		sp = pc3Dmap->m_global_cells[l_y];
		if(sp.get() != NULL)
			if(fabs(sp->rgb - p->rgb)<error_noise)
				return true;
	}
	if(r_y >=0 && r_y <ALL_CELLS){
		sp = pc3Dmap->m_global_cells[r_y];
		if(sp.get() != NULL)
			if(fabs(sp->rgb - p->rgb)<error_noise)
				return true;
	}

	for(int i= index - range_z; i<= index + range_z; i++)
	{
		if( i < 0 || i >=ALL_CELLS  )
			continue;
		sp = pc3Dmap->m_global_cells[i];
		if(sp.get()!=NULL)
			if(fabs(sp->rgb - p->rgb) < error_noise)
				return true;
	}
	return false;
}


int CMapBuilder::TMat_RAngle(float *pfTMat,float *pfRAngle)
{
	C3DMap *pc3Dmap=(C3DMap *)m_stClassPtrs.p3Dmap; 
	memcpy(pc3Dmap->m_fTMat,pfTMat,12);
	memcpy(pc3Dmap->m_fRAngle,pfRAngle,12);
	return 0;
}


void CMapBuilder::MergePointCloud2Area(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > &cloud)
{

	C3DMap *pc3Dmap=(C3DMap *)m_stClassPtrs.p3Dmap; 
	int N = cloud->points.size();
	vector<bool> index_point(N,false);
	vector<int> index_set;


	for(size_t i=0;i<cloud->points.size();i++)
	{
		pcl::PointXYZRGB& sp = cloud->points[i];
		int index = getIndexCell(sp.x,sp.y,sp.z);
		if(index >=0 )
		{
			if(pc3Dmap->m_global_cells[index].get()!=NULL) // this is already painted
				continue; // not flush this point using the new point	
			boost::shared_ptr<pcl::PointXYZRGB> p(new pcl::PointXYZRGB);
			p->x = sp.x; p->y = sp.y; p->z = sp.z; 
			p->r = sp.r; p->g = sp.g; p->b = sp.b;
			if(!IsNoisePoint(index,p))
			{
				index_point[i] = true;
				index_set.push_back(index);
			}
		}
	}
	for(size_t i=0,j=0;i<N;i++)
		if(index_point[i])
		{
			pcl::PointXYZRGB& sp = cloud->points[i];
			boost::shared_ptr<pcl::PointXYZRGB> p(new pcl::PointXYZRGB);
			p->x = sp.x; p->y = sp.y; p->z = sp.z; 
			p->r = sp.r; p->g = sp.g; p->b = sp.b;
			int index_cell = index_set[j++];
			if(pc3Dmap->m_global_cells[index_cell].get()== NULL){
				pc3Dmap->m_global_cells[index_cell] = p;
				pc3Dmap->m_global_cell_map->points.push_back(sp);
			}
		}
		//printf("MB Pass  !!!!\n");
	pc3Dmap->RenderFrame();
}