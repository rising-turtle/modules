#include "CFixed2DMap.h"
//#include "CDynamicArea.h"
#include "3DMap.h"
#include "CSession.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <iostream>

using namespace std;
 CFixed2DMap::CFixed2DMap(){InitMap();}
 CFixed2DMap::~CFixed2DMap(){}

 // 400*400 Grids around 50*50 meters
 void CFixed2DMap::InitMap()
 {
	m_area_width=50; // width 50m
	m_area_length=50; // length 50m
	m_gridsize=10;	// n grids/1 meter
	m_width=m_area_width*m_gridsize;
	m_length=m_area_length*m_gridsize;
	m_total_grid=m_width*m_length;
	m_offset_width=m_width>>1;
	m_offset_length=m_length>>1;
	
	m_lower_height=-0.1;
	m_high_height=0.5;

	m_vGrids.resize(m_width);
	for(unsigned int i=0;i<m_width;i++)
	{
		m_vGrids[i].resize(m_length,Grid_S::Unknown);
	}
 }

 void CFixed2DMap::UnInitMap()
 {
	 {
		 for(unsigned int i=0;i<m_vGrids.size();i++)
			 m_vGrids[i].swap(std::vector<Grid_S>());
		 m_vGrids.clear();
		 m_vGrids.swap(std::vector<std::vector<Grid_S> >());
	 }
 }

 int CFixed2DMap::getRow(float z)
 {
	int ret=z*10;
	int ori=m_length>>1;
	if(ret< -(ori) || ret>=(ori))
		return -1;
	return (ret+m_offset_length);
 }
 int CFixed2DMap::getCol(float x)
 {
	 int ret=x*10;
	 int ori=m_width>>1;
	 if(ret<-(ori) || ret>=(ori))
		 return -1;
	 return (ret+m_offset_width);
 }
 void CFixed2DMap::UpdateMap(void* pPc)
 {
	 C3DMap* p3DMap=(C3DMap*)pPc;
	 if(p3DMap==NULL)
	 {
		cout<<"This is invalid p3DMap!"<<endl;
		return;
	 }
	 pcl::PointCloud<pcl::PointXYZRGB>::Ptr p3DPc=p3DMap->m_global_cell_map;
	 // Clear changed indices 
	 m_Indices.clear();

	 bool changed;
	 for(size_t i=0;i<p3DPc->points.size();i++)
	 {
		 pcl::PointXYZRGB& sp = p3DPc->points[i];
		 int row = getRow(sp.z);
		 int col = getCol(sp.x);
		 changed=false;
		 if(row<0 || col<0 || row>=m_length || col>=m_width)
		 {
			cout<<"range beyond!"<<endl;
			continue;
		 }
		 if(sp.y>=m_lower_height && sp.y<=m_high_height)
		 {
			 m_vGrids[row][col]=Grid_S::Block;
			changed=true;
		 }
		 else if(sp.y>m_high_height)
		 {
			 if(m_vGrids[row][col]==Grid_S::Unknown)
			 {
				 m_vGrids[row][col]=Grid_S::Floor;
				 changed=true;
			 }
		 }
		 else if(sp.y<m_lower_height) 
		 {
				//
		 }
		 if(changed)
		 {
			// record changed point
			m_Indices.push_back(row*m_width+col);
		 }
	 }
	 return ;
 }

 //void CFixed2DMap::UpdateMap(void* pPc)
 //{
	// CSession* pSession=(CSession*)pPc;
	// if(pSession==NULL)
	// {
	//	 cout<<"This is invalid p3DMap!"<<endl;
	//	 return;
	// }
	// pcl::PointCloud<pcl::PointXYZRGB>::Ptr p3DPc=pSession->m_pDyArea2->m_dynamic_cell_map;
	// // Clear changed indices 
	// m_Indices.clear();

	// bool changed;
	// for(size_t i=0;i<p3DPc->points.size();i++)
	// {
	//	 pcl::PointXYZRGB& sp = p3DPc->points[i];
	//	 int row = getRow(sp.z);
	//	 int col = getCol(sp.x);
	//	 changed=false;
	//	 if(row<0 || col<0 || row>=m_length || col>=m_width)
	//	 {
	//		 cout<<"range beyond!"<<endl;
	//		 continue;
	//	 }
	//	 if(sp.y>=m_lower_height && sp.y<=m_high_height)
	//	 {
	//		 m_vGrids[row][col]=Grid_S::Block;
	//		 changed=true;
	//	 }
	//	 else if(sp.y>m_high_height)
	//	 {
	//		 if(m_vGrids[row][col]==Grid_S::Unknown)
	//		 {
	//			 m_vGrids[row][col]=Grid_S::Floor;
	//			 changed=true;
	//		 }
	//	 }
	//	 else if(sp.y<m_lower_height) 
	//	 {
	//			//
	//	 }
	//	 if(changed)
	//	 {
	//		 // record changed point
	//		 m_Indices.push_back(row*m_width+col);
	//	 }
	// }
	// return ;
 //}

 void CFixed2DMap::ClearMap()
 {
	 for(unsigned int i=0;i<m_vGrids.size();i++)
	 {
		 m_vGrids[i].clear();
		 m_vGrids[i].resize(m_length,Unknown);
	 }
 }

 void CFixed2DMap::outStream(unsigned int row,unsigned int col,unsigned char* pBuf)
 {
	unsigned char* pTmpBuf=pBuf;
	row=min(row,m_length);
	col=min(col,m_width);
	memcpy(pTmpBuf,&row,sizeof(int));
	memcpy(pTmpBuf+sizeof(int),&col,sizeof(int));
	pTmpBuf+=2*sizeof(int);
	for(int i=0;i<row;i++)
		for(int j=0;j<col;j++)
		{
			switch(m_vGrids[i][j])
			{
			case Grid_S::Unknown:
				(*pTmpBuf++)=0;
				break;
			case Grid_S::Block:
				(*pTmpBuf++)=1;
				break;
			case Grid_S::Floor:
				(*pTmpBuf++)=2;
				break;
			case Grid_S::Sink:
				(*pTmpBuf++)=3;
				break;
			default:
				cout<<"error in outStream()!"<<endl;
				break;
			}
		}
	return;
 }

 void CFixed2DMap::outStream(unsigned int lrow,unsigned int hrow, unsigned int lcol,unsigned int hcol, unsigned char* pBuf)
 {
	if( hrow<=lrow || hcol<=lcol)
	{
		cout<<"error at outStream!"<<endl;
		return ;
	}
	unsigned char*pTmpBuf=pBuf;
	unsigned int rspan=hrow-lrow;
	unsigned int cspan=hcol-lcol;
	memcpy(pTmpBuf,&rspan,sizeof(unsigned int));
	memcpy(pTmpBuf+sizeof(unsigned int),&cspan,sizeof(unsigned int));
	pTmpBuf+=2*sizeof(unsigned int);
	for(unsigned int i=lrow; i<hrow;i++)
	{
		for(unsigned int j=lcol;j<hcol;j++)
		{
			switch(m_vGrids[i][j])
			{
			case Grid_S::Unknown:
				(*pTmpBuf++)=0;
				break;
			case Grid_S::Block:
				(*pTmpBuf++)=1;
				break;
			case Grid_S::Floor:
				(*pTmpBuf++)=2;
				break;
			case Grid_S::Sink:
				(*pTmpBuf++)=3;
				break;
			default:
				cout<<"error in outStream()!"<<endl;
				break;
			}
		}
	}
	return ;
 }

 void  CFixed2DMap::outChangedGrid(unsigned char*pBuf)
 {
	 unsigned char* pTmpBuf=pBuf;
	 unsigned int T_N=m_Indices.size();
	 memcpy(pTmpBuf,&T_N,4);
	 pTmpBuf+=4;
	 for(size_t i=0;i<m_Indices.size();i++)
	 {
		int row=m_Indices[i]/m_width;
		int col=m_Indices[i]-row*m_width;
		if(col<0 || col>m_width)
		{
			cout<<"error at outChangedGrid!"<<endl;
			continue;
		}
		memcpy(pTmpBuf,&m_Indices[i],4);
		pTmpBuf+=4;
		switch(m_vGrids[row][col]){
			case Grid_S::Unknown:
				(*pTmpBuf++)=0;
				break;
			case Grid_S::Block:
				(*pTmpBuf++)=1;
				break;
			case Grid_S::Floor:
				(*pTmpBuf++)=2;
				break;
			case Grid_S::Sink:
				(*pTmpBuf++)=3;
				break;
		}
	 }
 }