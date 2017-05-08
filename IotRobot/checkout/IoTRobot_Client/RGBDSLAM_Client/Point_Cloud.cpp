//#include "stdafx.h"
#include "Point_Cloud.h"
#include <string>


#include <boost/thread/thread.hpp>
#include <boost/shared_ptr.hpp>
#include "pcl/common/common_headers.h"
#include "pcl/features/normal_3d.h"
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/interactor_style.h>
#include <pcl/visualization/cloud_viewer.h>
#include "MapBuilder.h"
#include "pcl/registration/ia_ransac.h"

#include "CPose3D.h"
//unsigned char * CPoint_Cloud::m_pucRGB24;

CPoint_Cloud::CPoint_Cloud()
{
	m_bNewFrame=false;
//	m_pucRGB24=0;
}

CPoint_Cloud::~CPoint_Cloud()
{

}

int CPoint_Cloud::PointCloudRun()
{
	LPDWORD ID=0;
	CreateThread(NULL,0,(LPTHREAD_START_ROUTINE)ThreadHandlePointCloud,this,0,ID);
	return 0;
}


UINT CPoint_Cloud::ThreadHandlePointCloud(LPVOID lpParam)
{
	CPoint_Cloud *pPC=(CPoint_Cloud *)lpParam;
//	CMapBuilder* pcMapBuilder=(CMapBuilder*)pPC->m_stClassPtrs.pMapBuilder;
//	C3DMap* p3DMap = (C3DMap*)pPC->m_stClassPtrs.p3Dmap;
	while (1)
	{
		if (pPC->m_bNewFrame==true)
		{
		
			Eigen::Matrix3f rotation = pPC->m_TransMat.block<3,3>(0, 0);
			Eigen::Vector3f translation = pPC->m_TransMat.block<3,1>(0, 3);
			
			CPose3D pose(pPC->m_TransMat);
			cout<<"in Mapbuilder!"<<endl;
			pose.output(std::cout);
			
			double xyz[3],rpy[3];
			float fTMat[3],fRAngle[3];
			pose.getXYZ(xyz);
			pose.getYawPitchRoll((double)rpy[2],(double)rpy[1],(double)rpy[0]);
			fTMat[0]=xyz[0];
			fTMat[1]=xyz[1];
			fTMat[2]=xyz[2];

			fRAngle[0]=rpy[0]/3.1415926*180;
			fRAngle[1]=rpy[1]/3.1415926*180;
			fRAngle[2]=rpy[2]/3.1415926*180;
			
			
			transformPointCloud (*pPC->m_PointCloud, *pPC->m_PointCloud, pPC->m_TransMat);
			

			// here we need to refresh all cells
			if(pcMapBuilder->refresh_cells){
				p3DMap->m_global_cell_map->points.clear(); // clear current points
				p3DMap->valid_flag.reset();
				p3DMap->pGlobalDisplay->points.clear(); // clear last points
				pcMapBuilder->refresh_cells = false;  // refresh 
				for( map<int,pcl::PointCloud<pcl::PointXYZRGB>::Ptr >::iterator iter_it=pcMapBuilder->Node_To_Cells.begin(); iter_it!=pcMapBuilder->Node_To_Cells.end();iter_it++)
					pcMapBuilder->MergeLMapwithGMap(p3DMap->pGlobalDisplay.get(),iter_it->second.get());
				pcMapBuilder->NewGlobalPCIn(p3DMap->pGlobalDisplay,-1); // Pass Global points into cells
				p3DMap->refresh_points = true;  // refresh points in OpenGL
				cout<<"all Cells of Node will be redraw!!"<<endl;
			}

			pcMapBuilder->TMat_RAngle(fTMat,fRAngle);
			pcMapBuilder->Id_Cells_Pose.insert(make_pair(pPC->id_of_node,pose));
			pcMapBuilder->NewGlobalPCIn(pPC->m_PointCloud,pPC->id_of_node);
			pPC->m_bNewFrame=false;
		}
		Sleep(20);
	}
	return 0;
}
int CPoint_Cloud::NewFrameCloudIn(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > &cloud,Eigen::Matrix4f *pTransMat)
{
	m_TransMat=*pTransMat;
	m_PointCloud=cloud;
	m_bNewFrame=true;
	return 0;
}

void CPoint_Cloud::cloudRegistration(boost::shared_ptr<CPose3D>& pose, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud)
{
	// --------------------------------------------------------------------------
	//  SPECIAL CASE OF HORIZONTAL SCAN: QUICKER IMPLEMENTATION
	// --------------------------------------------------------------------------
	Eigen::Matrix4f	HM;
	pose->getHomogeneousMatrix(HM);

	/*using pcl::transformation instead*/ 
	pcl::transformPointCloud (*cloud, *cloud, HM);
}