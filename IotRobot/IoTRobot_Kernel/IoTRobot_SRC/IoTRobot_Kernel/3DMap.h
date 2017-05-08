#pragma once
#include <boost/shared_ptr.hpp>
#include <boost/dynamic_bitset.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Core>
#include <vector>
#include "InternalDefine.h"
#include "CPose3D.h"
#include "CDynamicArea.h"
#include "CSession.h"

class C2DMap;

#define LOWER_ASS(Low,V){if(V<Low) Low=V;}
#define UPPER_ASS(Upp,V){if(V>Upp) Upp=V;}

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

	//for Trangles!
	void drawVertex(boost::shared_ptr<pcl::PointXYZRGB> p[3],unsigned char *pucImg,float *pfVertex);
	void FromCell2Triangles(unsigned char *pucImg,float *pfVertex);
	size_t m_num_of_ver;
	bool m_IsTriangle;

	ClassPtrs m_stClassPtrs;

	// only use getIndexCell in class CMapbuilder!!
	/*inline int getIndexCell(float& x,float& y, float& z)
	{
		if(!pcl_isfinite(x) || !pcl_isfinite(y) || !pcl_isfinite(z))
			return -1;
		if(fabs(x) >= 6 || fabs(z) >= 6 || y<=-2 || y>=2 )
			return -1;
		int lx = ( x*100 + RX/2) / CELLSIZE;
		int ly = ( y*100 + RY/2) / CELLSIZE;
		int lz = ( z*100 + RZ/2) / CELLSIZE;
		if(lx >= X_CELL || ly>= Y_CELL || lz >= Z_CELL)
		{
			cout<<"error in indexCell!"<<endl;
			return -1;
		}
		return (lx*X_STEP + ly*Y_STEP + lz);
	}*/
	void FromPC2Cell(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > &cloud);
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

	static CallBack_Session_Planes m_cbPlane;
	static unsigned char *m_pPlaneImg;
	static float *m_pPlaneVertex;

	// robot path
	std::map<int,CPose3D> m_robot_path;
	float* m_pfTrs;
	float* m_pfRot;
	int m_num_of_path_node;
	void FromMap2Pointer(float* pTrs,float* pRot,int& num);

	static int m_nCurrentVertexNum;

	DWORD m_ulFinishMapBulider;
	int m_nMapBuilderFrameCount;

	// temporary middle variable 
	static pcl::PointCloud<pcl::PointXYZRGB>::Ptr pGlobalDisplay;

	// index whether cells are still valid
	boost::dynamic_bitset<> valid_flag;
	
	// whether refresh all points in OpenGL
	bool refresh_points;

	// from Grid2Data
	void fromGrid2Data(unsigned char* pGrid,int& nCount);

	// dynamic Area to display
	bool m_bGlobalShow;
	boost::shared_ptr<CDynamicArea<pcl::PointXYZRGB> > m_pcDynamicArea;
	// whether to display all points in a session!
	bool m_display_whole_session;
	int m_x_cell,m_y_cell,m_z_cell; // dynamic value of Area
	int m_x_step,m_y_step;	// dynamic value of steps along axis
	bool m_bisswapped; // recover basic area after swap
	void DynamicEnswap();	// Swap with Dynamic area then to display
	void DynamicUnswap();	// Unswap to recover original Area

	// for debug
	bool m_bIsSave; 
	void SavetoFile(unsigned char* pImg, float* pVer, int nCount);
	void SavetoFile(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pCloud);
	void ResetBound();

	// absolute coordinate of x,y,z 
	float m_abs_lower_x,m_abs_upper_x;
	float m_abs_lower_y,m_abs_upper_y;
	float m_abs_lower_z,m_abs_upper_z;

	// for Load Sessions from disk
	vector<boost::shared_ptr<CSession> > m_Sessions;

	// for 2D Map 
	C2DMap* m_p2DMap;
	unsigned char* m_p2DMapBuf;
	void Generate2DMap();
	bool m_bIs2DAvaliable;

	// for Reset
	void Reset();

protected:
private:
};