#include "3DMap.h"



//pcl::PointCloud<pcl::PointXYZRGB>::Ptr C3DMap::m_global_cell_map; // to display cells


CallBack_PointCloud C3DMap::m_cbPointCould;
CallBack_SLAM_PC C3DMap::m_cbSLAMPC;

CallBack_Path C3DMap::m_cbPath;
unsigned char *C3DMap::m_pucImg;
float *C3DMap::m_pfVertex;
int C3DMap::m_nCurrentVertexNum;
float C3DMap::m_fTMat[3];
float C3DMap::m_fRAngle[3];


extern DWORD g_ulRunTimeBuff[LOG_ARRAY_LEN][8];
C3DMap::C3DMap()
{
	m_bOneRenderFrameReady=false;
}

C3DMap::~C3DMap()
{

}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr C3DMap::m_global_cell_map(new pcl::PointCloud<pcl::PointXYZRGB>);
int C3DMap::C3DMapInit()
{
	m_global_cells.resize(ALL_CELLS);
	m_pucImg=new unsigned char[640*480*3];
	m_pfVertex=new float[640*480*3];
	m_nCurrentVertexNum=0;

	m_nMapBuilderFrameCount=0;
	
	return 0;
}
int C3DMap::C3DMapRun()
{
	LPDWORD ID=0;
	CreateThread(NULL,0,(LPTHREAD_START_ROUTINE)ThreadCloudViewer,this,0,ID);
	return 0;
}

int C3DMap::RenderFrame()
{
	g_ulRunTimeBuff[m_nMapBuilderFrameCount][5]=::GetTickCount();
	m_nMapBuilderFrameCount++;
	if (m_nMapBuilderFrameCount==LOG_ARRAY_LEN)
	{
		m_nMapBuilderFrameCount=0;
	}
	m_bOneRenderFrameReady=true;
	return 0;
}
UINT C3DMap::ThreadCloudViewer(LPVOID lpParam)
{
	C3DMap *p3Dmap=(C3DMap *)lpParam;
	int i,nCount;
	unsigned char *pucImg;
	float *pfVertex;
	while (1)
	{
		if (p3Dmap->m_bOneRenderFrameReady)
		{

			//printf("3D map start  !!!\n");
			pucImg=m_pucImg;
			pfVertex=m_pfVertex;
			if (m_global_cell_map->points.size()<IOTGUI_SLAM_SIZE)
			{
				for (i=m_nCurrentVertexNum;i<m_global_cell_map->points.size();i++)
				{
					pcl::PointXYZRGB& pt = m_global_cell_map->points[i];
					*(pucImg++)=pt.r;
					*(pucImg++)=pt.g;
					*(pucImg++)=pt.b;

					*(pfVertex++)=pt.x;
					*(pfVertex++)=pt.y;
					*(pfVertex++)=pt.z;
				}
				nCount=(m_global_cell_map->points.size()-m_nCurrentVertexNum)*3;
				m_nCurrentVertexNum=m_global_cell_map->points.size();


			//	printf("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@Transmit matrix  :  %f,   %f,   %f  \n",m_fTMat[0],m_fTMat[1],m_fTMat[2]);
				m_cbSLAMPC(m_pucImg,m_pfVertex,NULL,nCount);
				m_cbPath(m_fTMat,m_fRAngle,NULL);


				p3Dmap->m_bOneRenderFrameReady=false;
				//printf("3D map pass  !!!\n");
				//m_cbPointCould(m_pucImg,m_pfVertex,NULL);
				//p3Dmap->m_pcCloudViewer->showCloud(p3Dmap->m_global_cell_map);
			}
			else
			{
				//printf("FUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUULLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL!!!!!\n");
			}
			
		}
		//printf("3D11111 map pass  !!!\n");
		Sleep(10);
		//printf("3D222222 map pass  !!!\n");
	}

	return 1;
}