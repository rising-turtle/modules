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
#include "pcl/registration/ia_ransac.h"
#include "MapBuilder.h"

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
	CMapBuilder* pcMapBuilder=(CMapBuilder*)pPC->m_stClassPtrs.pMapBuilder;
	while (1)
	{
		if (pPC->m_bNewFrame==true)
		{
		
			Eigen::Matrix3f rotation = pPC->m_TransMat.block<3,3>(0, 0);
			Eigen::Vector3f translation = pPC->m_TransMat.block<3,1>(0, 3);
			
			CPose3D pose(pPC->m_TransMat);
		//	cout<<"in Mapbuilder!"<<endl;
		//	pose.output(std::cout);
		/*	printf("value by align");
			printf ("\n");
			printf ("    | %6.3f %6.3f %6.3f | \n", rotation (0,0), rotation (0,1), rotation (0,2));
			printf ("R = | %6.3f %6.3f %6.3f | \n", rotation (1,0), rotation (1,1), rotation (1,2));
			printf ("    | %6.3f %6.3f %6.3f | \n", rotation (2,0), rotation (2,1), rotation (2,2));
			printf ("\n");
			printf ("t = < %0.3f, %0.3f, %0.3f >\n", translation (0), translation (1), translation (2));*/
			
			double xyz[3],rpy[3];
			float fTMat[3],fRAngle[3];
			pose.getXYZ(xyz);
			pose.getYawPitchRoll((double)rpy[2],(double)rpy[1],(double)rpy[0]);
			fTMat[0]=xyz[0];
			fTMat[1]=xyz[1];
			fTMat[2]=xyz[2];

			fRAngle[0]=rpy[0]/3.1415926*180.0;
			fRAngle[1]=rpy[1]/3.1415926*180.0;
			fRAngle[2]=rpy[2]/3.1415926*180.0;
			
			
			transformPointCloud (*pPC->m_PointCloud, *pPC->m_PointCloud, pPC->m_TransMat);
			
			pcMapBuilder->TMat_RAngle(fTMat,fRAngle);
			pcMapBuilder->NewGlobalPCIn(pPC->m_PointCloud);
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


void CPoint_Cloud::SetData(int nPos,char *pcContent)
{
	switch(nPos)
	{
	case 0:
		m_pucRGB24=(unsigned char *)pcContent;
		//memcpy(m_stSettingsData.pcSLAMMode,pcContent,SLAMMODE_MEMORY_SIZE);
		break;
	default:
		break;

	}
}



void CPoint_Cloud::GetData(int pos,char *pcContent)
{
	switch(pos)
	{
	case 0:
		//memcpy(pcContent,m_stSettingsData.pcSLAMMode,SLAMMODE_MEMORY_SIZE);
		pcContent=(char *)m_pucRGB24;
		break;
	default:
		break;

	}
}


void CPoint_Cloud::ExternalCall(unsigned char *pucRGB24)
{
//	pucRGB24=(unsigned char *)m_pucRGB24;
	if(m_pucRGB24!=0)
	memcpy(pucRGB24,m_pucRGB24,640*480*3);
}