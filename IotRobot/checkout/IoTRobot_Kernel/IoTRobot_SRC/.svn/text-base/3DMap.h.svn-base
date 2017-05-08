#pragma once
#include <boost/shared_ptr.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Core>
#include <vector>
#include "InternalDefine.h"
#include "CPose3D.h"

class C3DMap
{
public:
	C3DMap();
	~C3DMap();

	IoTRobot_3DMapMSG m_st3DMapMSG;
	void NewCommand(const IoTRobot_Message MSG);

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

	// Triangles
	bool m_IsTriangle;
	// From Cells to Triangles pucImg->Color Array, pfVertex->Vertex Array
	void FromCell2Triangles(unsigned char *pucImg,float *pfVertex);
	void drawVertex(boost::shared_ptr<pcl::PointXYZRGB> p[3],unsigned char *pucImg,float *pfVertex);
	// number of points to be rendered
	size_t m_num_of_ver;

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


	// temporary middle variable 
	static pcl::PointCloud<pcl::PointXYZRGB>::Ptr pGlobalDisplay;

	// index whether cells are still valid
	bitset<ALL_CELLS> valid_flag;
	
	// whether refresh all points in OpenGL
	bool refresh_points;

protected:
private:
};