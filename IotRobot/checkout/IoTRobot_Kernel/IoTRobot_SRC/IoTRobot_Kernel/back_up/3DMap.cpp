#include "3DMap.h"



//pcl::PointCloud<pcl::PointXYZRGB>::Ptr C3DMap::m_global_cell_map; // to display cells

//decide whether to use Triangle!
#define TRIANGLE

CallBack_PointCloud C3DMap::m_cbPointCould;
CallBack_SLAM_PC C3DMap::m_cbSLAMPC;

CallBack_Path C3DMap::m_cbPath;
unsigned char *C3DMap::m_pucImg;
float *C3DMap::m_pfVertex;
int C3DMap::m_nCurrentVertexNum;
float C3DMap::m_fTMat[3];
float C3DMap::m_fRAngle[3];


extern DWORD g_ulRunTimeBuff[LOG_ARRAY_LEN][8];
C3DMap::C3DMap():refresh_points(false)
{
	m_bOneRenderFrameReady=false;
	m_num_of_ver = 0;
	m_hThreadCloudViewer=0;
}

C3DMap::~C3DMap()
{

}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr C3DMap::m_global_cell_map(new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr C3DMap::pGlobalDisplay(new pcl::PointCloud<pcl::PointXYZRGB>);

int C3DMap::C3DMapInit()
{
	m_bStop3DMap=true;
	m_global_cells.resize(ALL_CELLS);
#ifdef TRIANGLE
	m_pucImg = new unsigned char[ALL_CELLS*3];
	m_pfVertex = new float [ALL_CELLS*3];
	m_IsTriangle = true;
#else
	m_pucImg=new unsigned char[640*480*3];
	m_pfVertex=new float[640*480*3];
	m_IsTriangle = false;
#endif
	//m_pucImg=new unsigned char[640*480*3];
	//m_pfVertex=new float[640*480*3];
	m_nCurrentVertexNum=0;

	m_nMapBuilderFrameCount=0;
	

	// for decomposite map<int,CPose> to float* 
	// at most 500 nodes
	m_pfTrs = new float[500*3];
	m_pfRot = new float[500*3];
	m_num_of_path_node=0;
	return 0;
}

int C3DMap::C3DMapUninit()
{
	if (m_pucImg!=NULL)
	{
		delete [] m_pucImg;
		m_pucImg=NULL;
	}

	if (m_pfVertex!=NULL)
	{
		delete [] m_pfVertex;
		m_pfVertex=NULL;
	}

	if (m_pfTrs!=NULL)
	{
		delete [] m_pfTrs;
		m_pfTrs=NULL;
	}

	if (m_pfRot!=NULL)
	{
		delete [] m_pfRot;
		m_pfRot=NULL;
	}
	return 0;
}
int C3DMap::C3DMapRun()
{
	LPDWORD ID=0;
	m_bStop3DMap=false;
	m_hThreadCloudViewer=CreateThread(NULL,0,(LPTHREAD_START_ROUTINE)ThreadCloudViewer,this,0,ID);
	return 0;
}

int C3DMap::C3DMapStop()
{
	m_bStop3DMap=true;
	if (m_hThreadCloudViewer!=0)
	{
		WaitForSingleObject(m_hThreadCloudViewer,INFINITE);
	}
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
void C3DMap::FromPC2Cell(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > &cloud)
{

	C3DMap *pc3Dmap=(C3DMap *)m_stClassPtrs.p3Dmap; 
	int N = cloud->points.size();

	static bitset<ALL_CELLS> cur_frame;
	cur_frame.reset();

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc(new pcl::PointCloud<pcl::PointXYZRGB>);

	cout<<"PC2Cell PC SIZE: "<<cloud->points.size()<<endl;
	for(size_t i=0;i<cloud->points.size();i++)
	{
		pcl::PointXYZRGB& sp = cloud->points[i];
		int index = getIndexCell(sp.x,sp.y,sp.z);
		if(index >=0 )
		{
			if(pc3Dmap->valid_flag[index] /*pc3Dmap->m_global_cells[index].get()!=NULL*/) // this is already painted
				continue; // not flush this point using the new point	
			boost::shared_ptr<pcl::PointXYZRGB> p(new pcl::PointXYZRGB);
			p->x = sp.x; p->y = sp.y; p->z = sp.z; 
			p->r = sp.r; p->g = sp.g; p->b = sp.b;
			if(!cur_frame[index]){
				cur_frame.set(index);
				pc3Dmap->m_global_cells[index] = p;
				pc3Dmap->m_global_cell_map->points.push_back(sp);
				pc->points.push_back(sp);
			}
		}
	}
	cout<<"!!! N of points into Node: "<< pc->points.size()<<endl;
	pc3Dmap->valid_flag |= cur_frame; // 
}

void C3DMap::drawVertex(boost::shared_ptr<pcl::PointXYZRGB> p[3],unsigned char *pucImg,float *pfVertex){
	for(size_t i=0;i<3;i++)
	{
		*(pucImg++) = p[i]->r;
		*(pucImg++) = p[i]->g;
		*(pucImg++) = p[i]->b;
		*(pfVertex++) = p[i]->x;
		*(pfVertex++) = p[i]->y;
		*(pfVertex++) = p[i]->z;
	}
	m_num_of_ver+=3;
	//glColor3ub(p->r,p->g,p->b);
	//glVertex3f(p->x,p->y,p->z);
}
// From Cells to Triangles pucImg->Color Array, pfVertex->Vertex Array
void C3DMap::FromCell2Triangles(unsigned char *pucImg,float *pfVertex)
{
	static double color_similar = 3;
	bool d_left,d_behind,d_down;
	boost::shared_ptr<pcl::PointXYZRGB> pself;
	boost::shared_ptr<pcl::PointXYZRGB> pP[3];
	int d_n = 0;
	unsigned char *pImg = pucImg;
	float * pVertex = pfVertex;
	//CMapBuilder * pMapbuilder = (CMapBuilder*)this->m_stClassPtrs->pMapBuilder;
	// create ShowList
	/*GLuint cloud_list_index = glGenLists(1);
	if(!cloud_list_index) {
	cout<<"No display list could be created"<<endl;
	return -1;
	}*/
	//glNewList(cloud_list_index, GL_COMPILE);
	//cloud_list_indices.push_back(cloud_list_index);


	for(int lx=0;lx<X_CELL;lx++) // traverse along x-axis
		for(int ly=0;ly<Y_CELL;ly++) // traverse along y-axis
			for(int lz=0;lz<Z_CELL;lz++) // traverse along z-axis
			{
				// initilization 

				// find point (lx,ly,lz) 
				int index_self = lx*X_STEP + ly*Y_STEP + lz; 
				pself = m_global_cells[index_self];
				pP[0] = pself;//global_cells[index_self];
				if(!valid_flag[index_self] || pself.get()==NULL){
					continue;
				}

				// on xoy plane
				//glBegin(GL_TRIANGLE_STRIP);
				//drawVertex(pself,pImg,pVertex);

				bool draw_xoy = true;
				int ll = lx-1; // left 
				int ld = ly-1; // down 
				int lb = lz-1; // before

				int left_p = ll*X_STEP + ly*Y_STEP + lz;
				int before_p = lx*X_STEP + ly*Y_STEP + lb;
				int down_p = lx*X_STEP + ld*Y_STEP + lz;

				// down point on xoy
				if( ll>=0 && ld >=0)
				{
					pP[1] = m_global_cells[ll*X_STEP + ld*Y_STEP + lz]; // down-left point
					pP[2] = m_global_cells[down_p]; // down point
				}
				else
					draw_xoy = false;
				if(draw_xoy && valid_flag[ll*X_STEP + ld*Y_STEP + lz] && valid_flag[down_p] && pP[1].get() != NULL && pP[2].get() != NULL && fabs(pP[1]->rgb - pP[0]->rgb) < color_similar)
				{
					drawVertex(pP,pImg,pVertex);
					pImg +=9;
					pVertex +=9;
				}
				else
					draw_xoy = false;
				if(draw_xoy)
				{
					pP[2] = m_global_cells[left_p]; // left point
					if(valid_flag[left_p] && pP[2].get()!=NULL)
					{
						drawVertex(pP,pImg,pVertex);
						pImg +=9;
						pVertex +=9;
					}
				}


				// on xoz plane
				bool draw_xoz = true;
				if( lb>= 0 && ll>=0){ 
					pP[1] = m_global_cells[ll*X_STEP + ly*Y_STEP + lb]; // left-before point
					pP[2] = m_global_cells[left_p]; // left point
				}
				else
					draw_xoz = false;
				if(draw_xoz && valid_flag[ll*X_STEP + ly*Y_STEP + lb] && valid_flag[left_p] && pP[1].get() != NULL && pP[2].get()!= NULL && fabs(pP[1]->rgb - pP[0]->rgb) < color_similar)
				{
					drawVertex(pP,pImg,pVertex);		
					pImg +=9;
					pVertex +=9;
				}
				else
					draw_xoz = false;
				if(draw_xoz){
					pP[2] = m_global_cells[before_p]; // before point
					if(valid_flag[before_p] && pP[2].get() != NULL)
					{
						drawVertex(pP,pImg,pVertex);
						pImg +=9;
						pVertex +=9;
					}
				}

				// on yoz plane
				bool draw_yoz = true;
				if(lb >=0 && ld>=0)
				{
					pP[1] = m_global_cells[lx*X_STEP + ld*Y_STEP + lb]; // before-down point
					pP[2] = m_global_cells[down_p];						// down point
				}
				else
					draw_yoz = false;
				if(draw_yoz && valid_flag[lx*X_STEP + ld*Y_STEP + lb] && valid_flag[down_p] && pP[1].get()!=NULL && pP[2].get()!=NULL && fabs(pP[1]->rgb - pP[0]->rgb)<color_similar)
				{
					drawVertex(pP,pImg,pVertex);
					pImg +=9;
					pVertex +=9;
				}
				else
					draw_yoz = false;
				if(draw_yoz)
				{
					pP[2] = m_global_cells[before_p]; // before point
					if(valid_flag[before_p] && pP[2].get()!=NULL)
					{
						drawVertex(pP,pImg,pVertex);
						pImg +=9;
						pVertex +=9;
					}
				}
			}
			return;
}
void C3DMap::FromMap2Pointer(float* pTrs,float* pRot,int& num)
{
	num=0;
	for(std::map<int,CPose3D>::iterator it_path=m_robot_path.begin();it_path!=m_robot_path.end();it_path++)
	{
		num++;
		(*pTrs++)=it_path->second.m_coords[0];
		(*pTrs++)=it_path->second.m_coords[1];
		(*pTrs++)=it_path->second.m_coords[2];
		(*pRot++)=it_path->second.roll;
		(*pRot++)=it_path->second.pitch;
		(*pRot++)=it_path->second.yaw;
	}
}

UINT C3DMap::ThreadCloudViewer(LPVOID lpParam)
{
	C3DMap *p3Dmap=(C3DMap *)lpParam;
	int i,nCount;
	unsigned char *pucImg;
	float *pfVertex;
	while (!p3Dmap->m_bStop3DMap)
	{
		if (p3Dmap->m_bOneRenderFrameReady)
		{

			//printf("3D map start  !!!\n");
			pucImg=m_pucImg;
			pfVertex=m_pfVertex;
			int nCleanFlag=0;

			if(p3Dmap->refresh_points)
			{
				cout<<"clear all points in Cells and rebuild!!"<<endl;
				// clear all points in OpenGL
				m_nCurrentVertexNum = 0;
				nCount = 0;
			}

			if(p3Dmap)
			if (m_global_cell_map->points.size()<IOTGUI_SLAM_SIZE)
			{

				if(p3Dmap->m_IsTriangle){
					p3Dmap->m_num_of_ver = 0;
					p3Dmap->FromCell2Triangles(pucImg,pfVertex);
					nCleanFlag = -1;
					nCount = p3Dmap->m_num_of_ver*3; 
					m_nCurrentVertexNum = 0;
				}
				else{
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
				}

				
				if(p3Dmap->refresh_points)
				{
					nCleanFlag=-1;
					m_nCurrentVertexNum = 0;
					//nCount = 0;
				}
				else
					m_nCurrentVertexNum=m_global_cell_map->points.size();

				// 
				m_nCurrentVertexNum = 0;
				nCleanFlag = -1;

				void *pFlag=(void*)&nCleanFlag;
				printf("@@@@@@@@@@@@@@@@@Transmit matrix  :  %f,   %f,   %f  \n",m_fTMat[0],m_fTMat[1],m_fTMat[2]);
				m_cbSLAMPC(m_pucImg,m_pfVertex,pFlag,nCount);
				
				p3Dmap->FromMap2Pointer(p3Dmap->m_pfTrs,p3Dmap->m_pfRot,p3Dmap->m_num_of_path_node);
				m_cbPath(p3Dmap->m_pfTrs,p3Dmap->m_pfRot,(void*)&p3Dmap->m_num_of_path_node);
				//m_cbPath(m_fTMat,m_fRAngle,NULL);


				p3Dmap->m_bOneRenderFrameReady=false;
				p3Dmap->refresh_points = false;
				//printf("3D map pass  !!!\n");
				//m_cbPointCould(m_pucImg,m_pfVertex,NULL);
				//p3Dmap->m_pcCloudViewer->showCloud(p3Dmap->m_global_cell_map);
			}
			else
			{
				printf("FUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUULLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL!!!!!\n");
			}
			
		}
		//printf("3D11111 map pass  !!!\n");
		Sleep(10);
		//printf("3D222222 map pass  !!!\n");
	}

	return 1;
}

void C3DMap::NewCommand(const IoTRobot_Message MSG)
{
	switch (MSG.cCommand)
	{
	case MAP_BUILDER_MSG_MSG1:
		m_st3DMapMSG.nMsg1=MSG.nParam1;
		break;
	default:
		break;
	}
	//memcpy(&m_stAlgOpt.cAlgorithmFlag,pCmd,ALGORITHM_NUM);
}