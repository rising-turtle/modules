#include "3DMap.h"
#include "MapBuilder.h"

#define MAXBUF100M 1024*1024*50

//pcl::PointCloud<pcl::PointXYZRGB>::Ptr C3DMap::m_global_cell_map; // to display cells

//decide whether to use Triangle!
#define TRIANGLE

CallBack_PointCloud C3DMap::m_cbPointCould;
CallBack_SLAM_PC C3DMap::m_cbSLAMPC;

CallBack_Path C3DMap::m_cbPath;
CallBack_Session_Planes C3DMap::m_cbPlane;
unsigned char *C3DMap::m_pucImg;
float *C3DMap::m_pfVertex;
int C3DMap::m_nCurrentVertexNum;
float C3DMap::m_fTMat[3];
float C3DMap::m_fRAngle[3];

unsigned char* C3DMap::m_pPlaneImg;
float* C3DMap::m_pPlaneVertex;

extern DWORD g_ulRunTimeBuff[LOG_ARRAY_LEN][8];
C3DMap::C3DMap():refresh_points(false),m_pcDynamicArea(new CDynamicArea<pcl::PointXYZRGB>),m_bGlobalShow(false)
{
	m_bOneRenderFrameReady=false;
	m_num_of_ver = 0;
	m_x_cell=X_CELL;
	m_y_cell=Y_CELL;
	m_z_cell=Z_CELL;
	m_x_step=X_STEP;
	m_y_step=Y_STEP;
	m_bisswapped=false;
	m_display_whole_session=false;
	valid_flag.resize(ALL_CELLS,false);
}

C3DMap::~C3DMap()
{

}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr C3DMap::m_global_cell_map(new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr C3DMap::pGlobalDisplay(new pcl::PointCloud<pcl::PointXYZRGB>);

int C3DMap::C3DMapInit()
{
	m_global_cells.resize(ALL_CELLS);
#ifdef TRIANGLE
	/*m_pucImg = new unsigned char[ALL_CELLS*3];
	m_pfVertex = new float [ALL_CELLS*3];*/
	m_pucImg = new unsigned char[MAXBUF100M*3];
	m_pfVertex = new float [MAXBUF100M*3];
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
	
	// initialze according to Kinect's capability
	m_abs_lower_x=20.0;
	m_abs_upper_x=-20.0;
	m_abs_lower_y=20.0;
	m_abs_upper_y=-20.0;
	m_abs_lower_z=20.0;
	m_abs_upper_z=-20.0;

	m_bIsSave=false;

	// for send plane info 
	m_pPlaneImg=new unsigned char[1024*1024*3]; //3MB to save plane-color
	m_pPlaneVertex=new float[1024*1024*3]; //3Mb to save plane-boundary-points

	// for decomposite map<int,CPose> to float* 
	// at most 500 nodes
	m_pfTrs = new float[500*3];
	m_pfRot = new float[500*3];
	m_num_of_path_node=0;
	return 0;
}
void C3DMap::ResetBound()
{
	// initialze according to Kinect's capability
	m_abs_lower_x=20.0;
	m_abs_upper_x=-20.0;
	m_abs_lower_y=20.0;
	m_abs_upper_y=-20.0;
	m_abs_lower_z=20.0;
	m_abs_upper_z=-20.0;
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
void C3DMap::FromPC2Cell(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > &cloud)
{
	C3DMap *pc3Dmap=(C3DMap *)m_stClassPtrs.p3Dmap; 
	CMapBuilder* pMapbuilder=(CMapBuilder*)m_stClassPtrs.pMapBuilder;
	int N = cloud->points.size();

	static boost::dynamic_bitset<> cur_frame;
	cur_frame.resize(ALL_CELLS,false);
	cur_frame.reset();

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc(new pcl::PointCloud<pcl::PointXYZRGB>);

	//cout<<"PC2Cell PC SIZE: "<<cloud->points.size()<<endl;
	for(size_t i=0;i<cloud->points.size();i++)
	{
		pcl::PointXYZRGB& sp = cloud->points[i];

		LOWER_ASS(m_abs_lower_x,sp.x);
		LOWER_ASS(m_abs_lower_y,sp.y);
		LOWER_ASS(m_abs_lower_z,sp.z);

		UPPER_ASS(m_abs_upper_x,sp.x);
		UPPER_ASS(m_abs_upper_y,sp.y);
		UPPER_ASS(m_abs_upper_z,sp.z);

		int index = pMapbuilder->getIndexCell(sp.x,sp.y,sp.z);
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
	//cout<<"!!! N of points into Node: "<< pc->points.size()<<endl;
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

	for(int lx=0;lx<m_x_cell;lx++) // traverse along x-axis
		for(int ly=0;ly<m_y_cell;ly++) // traverse along y-axis
			for(int lz=0;lz<m_z_cell;lz++) // traverse along z-axis
			{
				// initilization 

				// find point (lx,ly,lz) 
				int index_self = lx*m_x_step+ly*m_y_step + lz; 
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

				int left_p = ll*m_x_step + ly*m_y_step + lz;
				int before_p = lx*m_x_step + ly*m_y_step + lb;
				int down_p = lx*m_x_step + ld*m_y_step + lz;

				// down point on xoy
				if( ll>=0 && ld >=0)
				{
					pP[1] = m_global_cells[ll*m_x_step + ld*m_y_step + lz]; // down-left point
					pP[2] = m_global_cells[down_p]; // down point
				}
				else
					draw_xoy = false;
				if(draw_xoy && valid_flag[ll*m_x_step + ld*m_y_step + lz] && valid_flag[down_p] && pP[1].get() != NULL && pP[2].get() != NULL && fabs(pP[1]->rgb - pP[0]->rgb) < color_similar)
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
					pP[1] = m_global_cells[ll*m_x_step + ly*m_y_step + lb]; // left-before point
					pP[2] = m_global_cells[left_p]; // left point
				}
				else
					draw_xoz = false;
				if(draw_xoz && valid_flag[ll*m_x_step + ly*m_y_step + lb] && valid_flag[left_p] && pP[1].get() != NULL && pP[2].get()!= NULL && fabs(pP[1]->rgb - pP[0]->rgb) < color_similar)
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
					pP[1] = m_global_cells[lx*m_x_step + ld*m_y_step + lb]; // before-down point
					pP[2] = m_global_cells[down_p];						// down point
				}
				else
					draw_yoz = false;
				if(draw_yoz && valid_flag[lx*m_x_step + ld*m_y_step + lb] && valid_flag[down_p] && pP[1].get()!=NULL && pP[2].get()!=NULL && fabs(pP[1]->rgb - pP[0]->rgb)<color_similar)
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

// From Cells to Triangles pucImg->Color Array, pfVertex->Vertex Array
/*void C3DMap::FromCell2Triangles(unsigned char *pucImg,float *pfVertex)
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

/*
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
*/
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
			std::cout<<"IN P3DRENDER "<<std::endl;
			//printf("3D map start  !!!\n");
			pucImg=m_pucImg;
			pfVertex=m_pfVertex;
			int nCleanFlag=0;

			nCount=0;
			// need to display Sessions
			if(p3Dmap->m_Sessions.size()!=0)
			{
				cout<<"Sessions size="<<p3Dmap->m_Sessions.size()<<endl;
				/*for(size_t i=0;i<p3Dmap->m_Sessions.size();i++)
				{
					p3Dmap->m_Sessions[i].TransmitPC(pucImg,pfVertex);
					pucImg+=p3Dmap->m_Sessions[i].m_pc->points.size()*3;
					pfVertex+=p3Dmap->m_Sessions[i].m_pc->points.size()*3;
					nCount+=p3Dmap->m_Sessions[i].m_pc->points.size()*3;
				}*/			
				//for(size_t i=0;i<p3Dmap->m_Sessions.size();i++)
				//{
				//	//p3Dmap->m_Sessions[i].TransmitPC2(pucImg,pfVertex);
				//	p3Dmap->m_Sessions[i].TransmitPC3(pucImg,pfVertex);
				//	pucImg+=p3Dmap->m_Sessions[i].m_rendercolor->points.size()*3;
				//	pfVertex+=p3Dmap->m_Sessions[i].m_rendercolor->points.size()*3;
				//	nCount+=p3Dmap->m_Sessions[i].m_rendercolor->points.size()*3;
				//}
				/*for(size_t i=0;i<p3Dmap->m_Sessions.size();i++)
				{
					p3Dmap->m_Sessions[i].m_pDyArea2->FromCell2Triangles(pucImg,pfVertex);
					nCount+=p3Dmap->m_Sessions[i].m_pDyArea2->m_num_of_ver*3;
				}*/

				vector<boost::shared_ptr<CSession> >::iterator it=p3Dmap->m_Sessions.begin();
				if(it!=p3Dmap->m_Sessions.end())
				{
					unsigned char* pPlaneimg=m_pPlaneImg;
					float* pPlaneVertex=m_pPlaneVertex;
					int nCountP=0;
					(*it)->SendPlaneInfoParsePts(pPlaneimg,pPlaneVertex,nCountP);
					m_cbPlane(pPlaneimg,pPlaneVertex,nCountP);
					p3Dmap->m_Sessions.erase(it);
				}
				/*vector<CSession>::iterator it=p3Dmap->m_Sessions.begin();
				if(it!=p3Dmap->m_Sessions.end())
				{
					unsigned char* pPlaneimg=m_pPlaneImg;
					float* pPlaneVertex=m_pPlaneVertex;
					int nCountP=0;
					it->SendPlaneInfo(pPlaneimg,pPlaneVertex,nCountP);
					m_cbPlane(pPlaneimg,pPlaneVertex,nCountP);
					p3Dmap->m_Sessions.erase(it);
				}	*/
			}

			if(p3Dmap->refresh_points)
			{
				cout<<"clear all points in Cells and rebuild!!"<<endl;
				// clear all points in OpenGL
				m_nCurrentVertexNum = 0;
				nCount = 0;
			}

			if(p3Dmap)
			//if (m_global_cell_map->points.size()<IOTGUI_SLAM_SIZE)
			if( m_global_cell_map->points.size()*6 <= MAXBUF100M)
			{

				//cout<<"m_dynamic_cell_map.size="<<p3Dmap->m_pcDynamicArea->m_dynamic_cell_map->points.size()<<endl;
				//cout<<"m_global_cell_map.size="<<m_global_cell_map->points.size()<<endl;
				cout<<"before FromCell2Triangles"<<endl;
				if(p3Dmap->m_IsTriangle){
					p3Dmap->m_num_of_ver = 0;
					p3Dmap->FromCell2Triangles(pucImg,pfVertex);
					nCleanFlag = -1;
					nCount += p3Dmap->m_num_of_ver*3; 
					m_nCurrentVertexNum = 0;
				}
				else{
					for (i=m_nCurrentVertexNum;i<m_global_cell_map->points.size();i++)
					{
						/*if(p3Dmap->m_bIsSave)
						{
							p3Dmap->SavetoFile(m_global_cell_map);
							cout<<"!!! succeed to save fileBack!!!!!!"<<endl;
						}*/
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
				cout<<"after FromCell2Triangles"<<endl;
				
				if(p3Dmap->refresh_points)
				{
					nCleanFlag=-1;
					m_nCurrentVertexNum = 0;
					//nCount = 0;
				}
				else
					m_nCurrentVertexNum=m_global_cell_map->points.size();

				// for debug
				m_nCurrentVertexNum = 0;
				nCleanFlag = -1;

				cout<<"before m_cbSLAMPC"<<endl;
				void *pFlag=(void*)&nCleanFlag;
				printf("@@@@@@@@@@@@@@@@@Transmit matrix  :  %f,   %f,   %f  \n",m_fTMat[0],m_fTMat[1],m_fTMat[2]);
				m_cbSLAMPC(m_pucImg,m_pfVertex,pFlag,nCount);

				cout<<"after m_cbSLAMPC"<<endl;
				cout<<"before FromMap2Pointer"<<endl;
				p3Dmap->FromMap2Pointer(p3Dmap->m_pfTrs,p3Dmap->m_pfRot,p3Dmap->m_num_of_path_node);
				m_cbPath(p3Dmap->m_pfTrs,p3Dmap->m_pfRot,(void*)&p3Dmap->m_num_of_path_node);
			//	m_cbPath(m_fTMat,m_fRAngle,NULL);

				cout<<"nCount="<<nCount<<endl;

				cout<<"after FromMap2Pointer"<<endl;

			/*	if(p3Dmap->m_bIsSave)
				{
					p3Dmap->SavetoFile(m_pucImg,m_pfVertex,nCount);
					p3Dmap->m_bIsSave=false;
					cout<<"!!! succeed to save file!!!!!!"<<endl;
				}*/

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
	std::cout<<"OUT P3DRENDER "<<std::endl;
	return 1;
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

void C3DMap::SavetoFile(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pCloud)
{
	std::string save_f("D:\\tmpfiles\\VertexBack.txt");
	ofstream s_f(save_f.c_str());
	if(s_f.is_open())
	{
		int nCount=pCloud->points.size();
		s_f<<nCount<<endl;
		for(int i=0;i<nCount;i++)
		{
			pcl::PointXYZRGB& sp=pCloud->points[i];
			s_f<<"("<<sp.x<<","<<sp.y<<","<<sp.z<<")" <<endl;
			//<<"("<<std::hex<<*(ptmpImg)<<","<<std::hex<<*(ptmpImg+1)<<","<<std::hex<<*(ptmpImg+2)<<")"<<endl;
		}
		s_f.close();
	}
}
void C3DMap::SavetoFile(unsigned char* pImg, float* pVer, int nCount)
{
	std::string save_f("D:\\tmpfiles\\Vertex.txt");
	ofstream s_f(save_f.c_str());
	if(s_f.is_open())
	{
		s_f<<nCount<<endl;
		unsigned char* ptmpImg=pImg;
		float* ptmpVer=pVer;
		for(int i=0;i<nCount;i++)
		{
			s_f<<"("<<*(ptmpVer)<<","<<*(ptmpVer+1)<<","<<*(ptmpVer+2)<<")" <<endl;
				//<<"("<<std::hex<<*(ptmpImg)<<","<<std::hex<<*(ptmpImg+1)<<","<<std::hex<<*(ptmpImg+2)<<")"<<endl;

			//ptmpImg+=3;
			ptmpVer+=3;
		}
		s_f.close();
	}
}

void C3DMap::NewCommand(const IoTRobot_Message MSG)
{
	switch (MSG.cCommand)
	{
	case DMAP_MSG_MSG1:
		if (m_st3DMapMSG.nMsg1=MSG.nParam1)
		{
			m_display_whole_session=true;
		}
		else
		{
			m_bGlobalShow=false;
		}
		

		printf("hahhahahahhahah   %d  \n",MSG.nParam1);
		break;
	default:
		break;
	}
	//memcpy(&m_stAlgOpt.cAlgorithmFlag,pCmd,ALGORITHM_NUM);
}

void C3DMap::DynamicEnswap()	// Swap with Dynamic area then to display
{
	m_x_cell=m_pcDynamicArea->m_x_cell;
	m_y_cell=m_pcDynamicArea->m_y_cell;
	m_z_cell=m_pcDynamicArea->m_z_cell;
	m_x_step=m_pcDynamicArea->m_x_step;
	m_y_step=m_pcDynamicArea->m_y_step;
	m_global_cell_map.swap(m_pcDynamicArea->m_dynamic_cell_map);
	m_global_cells.swap(m_pcDynamicArea->m_dynamic_cells);
	valid_flag.swap(m_pcDynamicArea->m_valid_flag);
	CMapBuilder::m_iCellSize=m_pcDynamicArea->m_s_CellSize;
	m_bisswapped=true;
}
void C3DMap::DynamicUnswap()	// Unswap to recover original Area
{
	m_x_cell=X_CELL;
	m_y_cell=Y_CELL;
	m_z_cell=Z_CELL;
	m_x_step=X_STEP;
	m_y_step=Y_STEP;
	m_global_cell_map.swap(m_pcDynamicArea->m_dynamic_cell_map);
	m_global_cells.swap(m_pcDynamicArea->m_dynamic_cells);
	valid_flag.swap(m_pcDynamicArea->m_valid_flag);
	m_pcDynamicArea->m_s_CellSize=CMapBuilder::m_iCellSize;
	CMapBuilder::m_iCellSize=2;
	m_bisswapped=false;
}

void C3DMap::fromGrid2Data(unsigned char* pGrid,int& nCount)
{
	for(size_t _row=0;_row<m_pcDynamicArea->m_vGrids.size();_row++)
		for(size_t _col=0;_col<m_pcDynamicArea->m_vGrids[_row].size();_col++)
		{
			switch(_col<m_pcDynamicArea->m_vGrids[_row][_col])
			{
			case CDynamicArea<pcl::PointXYZRGB>::UnKnown:
				(*pGrid++)=-1;
				break;
			case CDynamicArea<pcl::PointXYZRGB>::Block:
				(*pGrid++)=0;
				break;
			case CDynamicArea<pcl::PointXYZRGB>::Floor:
				(*pGrid++)=1;
				break;
			case CDynamicArea<pcl::PointXYZRGB>::Sink:
				(*pGrid++)=2;
				break;
			default:
				cout<<"error in fromGrid2Data()!"<<endl;
				break;
			}
		}
		nCount=m_pcDynamicArea->m_vGrids.size()*m_pcDynamicArea->m_vGrids[0].size();
}