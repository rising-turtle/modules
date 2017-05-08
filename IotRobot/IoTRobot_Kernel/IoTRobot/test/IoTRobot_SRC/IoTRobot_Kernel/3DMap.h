#pragma once
#include <boost/shared_ptr.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Core>
#include <vector>
#include "InternalDefine.h"

class C3DMap
{
public:
	C3DMap();
	~C3DMap();

	int C3DMapInit();
	int C3DMapRun();
	int C3DMapUninit();

	int m_Width;
	int m_Height;
	HWND m_hWindow;
	bool m_bOneRenderFrameReady;

	int RenderFrame();
	static UINT ThreadCloudViewer(LPVOID lpParam);
	pcl::visualization::CloudViewer *m_pcCloudViewer;

	ClassPtrs m_stClassPtrs;

	static pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_global_cell_map; // to display cells
	std::vector<boost::shared_ptr<pcl::PointXYZRGB> > m_global_cells; // all the discrete cells 5*5*5
	//static void CallBack_PointCloud(unsigned char *pucImg,float *pfVertex,void *pContext);
	static CallBack_PointCloud m_cbPointCould;
	static CallBack_SLAM_PC m_cbSLAMPC;
	static CallBack_Path m_cbPath;
	static unsigned char *m_pucImg;
	static float *m_pfVertex;

	static float m_fTMat[3];
	static float m_fRAngle[3];



	static int m_nCurrentVertexNum;



	DWORD m_ulFinishMapBulider;
	int m_nMapBuilderFrameCount;

protected:
private:
};