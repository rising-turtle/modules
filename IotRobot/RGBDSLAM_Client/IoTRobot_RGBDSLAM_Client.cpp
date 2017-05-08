#include "IoTRobot_RGBDSLAM_Client.h"

typedef struct IoTRobot_SLAM_SETTING
{
	float fMinTransVal;
	float fMaxTransVal;
	float fMinRotVal;
	float fMaxRotVal;
	int nGraphDegree;
	int nInliner;
	int nMinGraph;
	int nMaxGraph;
	int nDecType;
}IoTRobot_SLAM_SETTING;


IoTRobot_RGBDSLAM_Client::IoTRobot_RGBDSLAM_Client():m_subPC( new pcl::PointCloud<pcl::PointXYZRGB>)
{
	m_hPCData=0;
	m_hSLAM=0;
	m_pNodeFeatures=(Node3DDesc**)malloc(sizeof(Node3DDesc*));
	*m_pNodeFeatures=(Node3DDesc*)malloc(sizeof(Node3DDesc));
}

IoTRobot_RGBDSLAM_Client::~IoTRobot_RGBDSLAM_Client()
{
	free(*m_pNodeFeatures);
	free(m_pNodeFeatures);
}

int IoTRobot_RGBDSLAM_Client::SLAMInit()
{
	m_CRGBDSLAM.IoTRobot_RGBDSLAM_Init();
	m_pucRGB=new unsigned char[640*480*3];
	m_pusDepth=new unsigned short[640*480];
	m_pucSLAMData=new unsigned char[1024*1024*15];
	m_nSLAMDataLen=0;
	memset(m_dCurPos,0,48);
	m_bResetSession = false;
	return 0;
}


UINT IoTRobot_RGBDSLAM_Client::ThreadSLAM(LPDWORD lpParam)
{
	IoTRobot_RGBDSLAM_Client *pSLAMClient=(IoTRobot_RGBDSLAM_Client *)lpParam;
	Eigen::Matrix4f final_transformation;
	Eigen::Matrix4f tmp_matrix;
	int nAcc=0;
	int i,j,k,nCount=0,nRtn,nPCLen,nId_MATLen,nNodeNumLen=4;
	int nCurrentNodeNum;
	unsigned char *pucTmp=NULL,*pucID_MATHead,*pucSLAMDataHead,*pucNodeNumHead,*pucFeatureHead,cLook1[64];

	unsigned char *pucSparsePC=new unsigned char[640*480*15];
	int nSparsePCDataLen;

	while (!pSLAMClient->m_bStopSLAM)
	{
		if (pSLAMClient->m_bNewSyncDataArrive&&nAcc==0)
		{
			pSLAMClient->m_nSLAMDataLen=4;
			nAcc+=1;

			// 此刻我们需要从 GroundTruth 的测试集里面读取点运数据，进行SLAM测试
			memcpy(pucSparsePC,pSLAMClient->m_pucSparsePC,pSLAMClient->m_nSparsePCDataLen);
			nSparsePCDataLen=pSLAMClient->m_nSparsePCDataLen;

			if(pSLAMClient->m_CReadGroundTrues.Run(
				pSLAMClient->m_pucRGB,
				pSLAMClient->m_pusDepth,
				pSLAMClient->m_PC
				)!=0
				)
			{
				break;
			}

			

			nRtn=pSLAMClient->m_CRGBDSLAM.IoTRobot_RGBDSLAM_RunOneFrame2(
				pSLAMClient->m_pucRGB,
				pSLAMClient->m_pusDepth,
				pSLAMClient->m_PC,
				final_transformation,
				&pSLAMClient->m_ulRunTimeBuff[0][2],
				NULL,0,
				pSLAMClient->m_rebuild_cells,
				pSLAMClient->m_robotpath_update,
				pSLAMClient->m_id_of_node,
				nCurrentNodeNum,
				pSLAMClient->m_pNodeFeatures);
			if (nRtn==0)
			{
				//transfer current SLAM Status
				memset(pSLAMClient->m_pucSLAMData,0,4);

				//transfer PC
				pucSLAMDataHead=pSLAMClient->m_pucSLAMData+pSLAMClient->m_nSLAMDataLen;
				pucTmp=pucSLAMDataHead+4;
				pSLAMClient->MergePointCloud2Area(pSLAMClient->m_PC,
					*pSLAMClient->m_id_of_node.rbegin(),
					pucTmp,&nPCLen);
				memcpy(pucSLAMDataHead,&nPCLen,4);
				pSLAMClient->m_nSLAMDataLen+=4;
				pSLAMClient->m_nSLAMDataLen+=nPCLen;
				
				//transfer Node info
				pucID_MATHead=pSLAMClient->m_pucSLAMData+pSLAMClient->m_nSLAMDataLen;
				pucTmp=pucID_MATHead+4;

				//pSLAMClient->m_id_of_node2.clear();
				//pSLAMClient->m_id_of_node2=pSLAMClient->m_id_of_node; // record id_of_node
				vector<float> tmpNode;
				tmpNode.resize(16);
				i=0;
				for (j=0;j<4;j++)
				{
					for (k=0;k<4;k++)
					{
						tmpNode[i++]=final_transformation(j,k);
					}
				}

				//get current pos
				CPose3D cur_pose(final_transformation);
				double xyz[3],rpy[3];
				cur_pose.getrpy(rpy);
				cur_pose.getXYZ(xyz);
				memcpy(pSLAMClient->m_dCurPos,xyz,3*sizeof(double));
				memcpy(pSLAMClient->m_dCurPos+3,rpy,3*sizeof(double));

				// Dynamically transformation
				static size_t last_size=pSLAMClient->m_id_of_node.size();
				if(pSLAMClient->m_id_of_node.size()<last_size) // means dynamic translation
				{
					vector<int>::iterator it_del_first_id=pSLAMClient->m_id_of_node2.begin();
					vector<vector<float> >::iterator it_del_first_matrix=pSLAMClient->m_matrix_of_nodeF.begin();
					
					if(pSLAMClient->m_id_of_node2.size()!=pSLAMClient->m_matrix_of_nodeF.size())
					{
						cout<<"ca!!!"<<endl;
					}

					for(;it_del_first_id!=pSLAMClient->m_id_of_node2.end();it_del_first_id++,it_del_first_matrix++)
					{
						if(*it_del_first_id==*pSLAMClient->m_id_of_node.begin())
							break;
					}
					pSLAMClient->m_id_of_node2.erase(pSLAMClient->m_id_of_node2.begin(),it_del_first_id);
					pSLAMClient->m_matrix_of_nodeF.erase(pSLAMClient->m_matrix_of_nodeF.begin(),it_del_first_matrix);
				}
				
				pSLAMClient->m_id_of_node2.push_back(*pSLAMClient->m_id_of_node.rbegin()); // record id_of_node
				pSLAMClient->m_matrix_of_nodeF.push_back(tmpNode);				
				last_size=pSLAMClient->m_id_of_node.size();
				//	pSLAMClient->m_matrix_of_node.push_back(final_transformation);						   // record pose_of_node

				// update all pose of previous node!
				Eigen::Matrix4f out_HM;
				for(size_t i=0;i<pSLAMClient->m_robotpath_update.size()-1;i++)
				{
					CPose3D& cur_pose=pSLAMClient->m_robotpath_update[i];
					vector<float>& tmpNode=pSLAMClient->m_matrix_of_nodeF[i];
					cur_pose.getHomogeneousMatrix(out_HM);
					int index=0;
					for (j=0;j<4;j++)
					{
						for (k=0;k<4;k++)
						{
							tmpNode[index++]=out_HM(j,k);
						}
					}
				}

				nId_MATLen=0;
				nCount=0;
				for (i=0;i<pSLAMClient->m_id_of_node2.size();i++)
				{
					memcpy(pucTmp,&pSLAMClient->m_id_of_node2[i],4);
					pucTmp+=4;
					nCount+=4;
					for (j=0;j<16;j++)
					{
						memcpy(pucTmp,&pSLAMClient->m_matrix_of_nodeF[i][j],4);
						memcpy(cLook1,&pSLAMClient->m_matrix_of_nodeF[i][j],4);
						pucTmp+=4;
						nCount+=4;
					}
				//	memcpy(pucTmp,&pSLAMClient->m_matrix_of_nodeF[i],64);
				//	memcpy(cLook1,&pSLAMClient->m_matrix_of_nodeF[i],64);
				//	pucTmp+=64;
				//	nCount+=64;
				}
				if (pSLAMClient->m_id_of_node2.size()*68!=nCount)
				{
					printf("pSLAMClient->m_id_of_node2   size wrong!!!! \n");
				}
				else
				{
					nId_MATLen=nCount;
					memcpy(pucID_MATHead,&nId_MATLen,4);
					pSLAMClient->m_nSLAMDataLen+=4;
					pSLAMClient->m_nSLAMDataLen+=nCount;
				}

				//transfer Node Number
				pucNodeNumHead=pSLAMClient->m_pucSLAMData+pSLAMClient->m_nSLAMDataLen;
				pucTmp=pucNodeNumHead+4;
				memcpy(pucNodeNumHead,&nNodeNumLen,4);
				memcpy(pucTmp,&nCurrentNodeNum,4);
				pSLAMClient->m_nSLAMDataLen+=8;

				if(*pSLAMClient->m_id_of_node.begin()!=0)
				{
					int debug=0;
				}

				// transform features of this node
				// n: number of feature points		4
				// id: id of this node				4
				// xyz location of each feature		4*3
				// float[64] descriptor of feature	4*64
				pucFeatureHead=pSLAMClient->m_pucSLAMData+pSLAMClient->m_nSLAMDataLen;
				pucTmp=pucFeatureHead+4+4;
				int n_features=(*pSLAMClient->m_pNodeFeatures)->n;
				int nFeatureDataLen=(64+3)*4;//64*4;//n_features*64*8;
				int nTotalLen=nFeatureDataLen*n_features;
				memcpy(pucFeatureHead,&nTotalLen,4);

				int cur_id=*pSLAMClient->m_id_of_node.rbegin();
				memcpy(pucFeatureHead+4,&cur_id,4); // id of this node
				for(int i=0;i<n_features;i++)
				{
					memcpy(pucTmp,&((*pSLAMClient->m_pNodeFeatures)->desc3DList[i].xyz),3*4);
					pucTmp+=3*4; // float xyz, 3*4 
					memcpy(pucTmp,(*pSLAMClient->m_pNodeFeatures)->desc3DList[i].desc,64*4);//nFeatureDataLen);
					pucTmp+=64*4;//nFeatureDataLen;
				}
				pSLAMClient->m_nSLAMDataLen+=8;
				pSLAMClient->m_nSLAMDataLen+=nFeatureDataLen*n_features;


				if(pSLAMClient->m_rebuild_cells) 
				{
					pSLAMClient->m_rebuild_cells = false;
				}
			}
			else
			{
				memcpy(pSLAMClient->m_pucSLAMData,&nRtn,4);
			}
			pSLAMClient->m_cbSLAMOK(pSLAMClient->m_pucSLAMData,pSLAMClient->m_nSLAMDataLen,pSLAMClient->m_pSLAMOKContext);

			pSLAMClient->m_bNewSyncDataArrive=false;
			nAcc-=1;
			// Check if we should reset session??
			if (true == pSLAMClient->m_bResetSession)
			{
				pSLAMClient->m_CRGBDSLAM.IoTRobot_RGBDSLAM_ResetSession(pSLAMClient->m_SessionPose);
				pSLAMClient->m_bResetSession =false;
			}
		}
		else
		{
			Sleep(10);
		}

	}
	return 0;
}

int IoTRobot_RGBDSLAM_Client::resetSession(CPose3D pose){

	m_SessionPose = pose;
	m_bResetSession = true;
	return 0;

}

int IoTRobot_RGBDSLAM_Client::getIndexCell(float& x,float& y, float& z)
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
}

void IoTRobot_RGBDSLAM_Client::MergePointCloud2Area
(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > &cloud, int id_of_node,unsigned char *pucDataHead,int *pnPCLen)
{
	int N = cloud->points.size();
	int nCount=0;
	static bitset<ALL_CELLS> cur_frame;
	cur_frame.reset();

	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc(new pcl::PointCloud<pcl::PointXYZRGB>);

	// clear sub-PointCloud after Voxel Filtering
	m_subPC->points.clear();
	unsigned char *pucTmp;
	pucTmp=pucDataHead;
	int nPCLen=0;
	for(size_t i=0;i<cloud->points.size();i++)
	{
		pcl::PointXYZRGB& sp = cloud->points[i];
		int index = getIndexCell(sp.x,sp.y,sp.z);
		if(index >=0 )
		{
			//if(m_valid_flag[index] /*pc3Dmap->m_global_cells[index].get()!=NULL*/) // this is already painted
			//	continue; // not flush this point using the new point	
			if(cur_frame[index])
				continue;
			boost::shared_ptr<pcl::PointXYZRGB> p(new pcl::PointXYZRGB);
			p->x = sp.x; 
			p->y = sp.y;
			p->z = sp.z; 
			p->r = sp.r;
			p->g = sp.g; 
			p->b = sp.b;	

			//可优化
			memcpy(pucTmp,&p->x,4);
			pucTmp+=4;
			nCount+=4;
			memcpy(pucTmp,&p->y,4);
			pucTmp+=4;
			nCount+=4;
			memcpy(pucTmp,&p->z,4);
			pucTmp+=4;
			nCount+=4;
			memcpy(pucTmp,&p->r,1);
			pucTmp+=1;
			nCount+=1;
			memcpy(pucTmp,&p->g,1);
			pucTmp+=1;
			nCount+=1;
			memcpy(pucTmp,&p->b,1);
			pucTmp+=1;
			nCount+=1;


			cur_frame.set(index);
			m_subPC->points.push_back(sp);
		}
	}

	if (m_subPC->points.size()*15!=nCount)
	{
		printf("subPC size wrong!!!! \n");
	}
	else
	{
		nPCLen=nCount;
		*pnPCLen=nPCLen;
	}
	//cout<<"!!! N of points into Node: "<< m_subPC->points.size()<<endl;
}
bool IoTRobot_RGBDSLAM_Client::IsNoisePoint(int index,boost::shared_ptr<pcl::PointXYZRGB>& p){

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
		sp =m_global_cells[l_x];
		if(m_valid_flag[l_x] && sp.get() != NULL)
			if( fabs(sp->rgb - p->rgb)<error_noise)
				return true;
	}
	if(r_x >=0 && r_x <ALL_CELLS){
		sp = m_global_cells[r_x];
		if(m_valid_flag[r_x] && sp.get() != NULL)
			if(fabs(sp->rgb - p->rgb)<error_noise)
				return true;
	}
	if(l_y >=0 && l_y <ALL_CELLS){
		sp = m_global_cells[l_y];
		if(m_valid_flag[l_y] && sp.get() != NULL)
			if(fabs(sp->rgb - p->rgb)<error_noise)
				return true;
	}
	if(r_y >=0 && r_y <ALL_CELLS){
		sp = m_global_cells[r_y];
		if(m_valid_flag[r_y] && sp.get() != NULL)
			if(fabs(sp->rgb - p->rgb)<error_noise)
				return true;
	}

	for(int i= index - range_z; i<= index + range_z; i++)
	{
		if( i < 0 || i >=ALL_CELLS  )
			continue;
		sp =m_global_cells[i];
		if(m_valid_flag[i] && sp.get()!=NULL)
			if(fabs(sp->rgb - p->rgb) < error_noise)
				return true;
	}
	return false;
}

int IoTRobot_RGBDSLAM_Client::SLAMRun()
{

	m_bStopSLAM=false;
	m_bNewSyncDataArrive=false;
	m_bStopSLAMClient=false;
	m_rebuild_cells=false;


	ID_SLAM=0;
	m_hSLAM=0;
	m_hSLAM=CreateThread(NULL,0,(LPTHREAD_START_ROUTINE)ThreadSLAM,this,0,ID_SLAM);

	ID_PCData=0;
	m_hPCData=0;
	m_hPCData=CreateThread(NULL,0,(LPTHREAD_START_ROUTINE)ThreadGetPCData,this,0,ID_PCData);
	return 0;
}

int IoTRobot_RGBDSLAM_Client::SLAMUnint()
{
	if (m_pusDepth!=NULL)
	{
		delete [] m_pusDepth;
		m_pusDepth=NULL;
	}

	if (m_pucRGB!=NULL)
	{
		delete [] m_pucRGB;
		m_pucRGB=NULL;
	}

	if (m_pucSLAMData!=NULL)
	{
		delete [] m_pucSLAMData;
		m_pucSLAMData=NULL;
	}
	m_CRGBDSLAM.IoTRobot_RGBDSLAM_Uninit();
	return 0;
}


void IoTRobot_RGBDSLAM_Client::getImagesandDepthMetaData(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> >& point_cloud,
									  unsigned char* rgbbuf,
									  unsigned short* depthbuf
											)
{
	unsigned short * pdepth =NULL;
	pdepth=depthbuf;
	unsigned char  * pimage = rgbbuf;  
	unsigned int totalnum = point_cloud->width * point_cloud->height;

	float bad_point = std::numeric_limits<float>::quiet_NaN();


	for(size_t i=0;i<totalnum;i++){

		pcl::PointXYZRGB& pt = point_cloud->points[i];
		// get rgb-info 
		*pimage = pt.r;
		pimage++;
		*pimage = pt.g;
		pimage++;
		*pimage = pt.b;
		pimage++;

		// get depth-info
		if(pt.x == bad_point && pt.y == bad_point && pt.z == bad_point){
			*pdepth = 0;
		}
		else
		{
			*pdepth = pt.z * 1000.0f;
		}
		pdepth ++;
	}
}


int IoTRobot_RGBDSLAM_Client::SLAMStop()
{
	m_bStopSLAM=true;
	if (m_hSLAM!=0)
	{
		WaitForSingleObject(m_hSLAM,INFINITE);
	}
	m_bStopSLAMClient=true;
	if (m_hPCData!=0)
	{
		WaitForSingleObject(m_hPCData,INFINITE);
	}
	m_CRGBDSLAM.IoTRobot_RGBDSLAM_Reset();
	return 0;
}


UINT IoTRobot_RGBDSLAM_Client::ThreadGetPCData(LPDWORD lpParam)
{
	IoTRobot_RGBDSLAM_Client *pRGBDSlam=(IoTRobot_RGBDSLAM_Client *)lpParam;
	boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > point_cloud(new pcl::PointCloud<pcl::PointXYZRGB > ());
	boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > point_cloudA(new pcl::PointCloud<pcl::PointXYZRGB > ());
	boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > point_cloudB(new pcl::PointCloud<pcl::PointXYZRGB > ());

	bool bJumpOut=false;


	int nBuffIdx=0;
	point_cloud=point_cloudA;

	pRGBDSlam->m_COpenNIViewer.start();
	while(!pRGBDSlam->m_bStopSLAMClient)
	{
		if (pRGBDSlam->m_COpenNIViewer.getCloudPoint(point_cloud))
		{
			pRGBDSlam->getImagesandDepthMetaData(point_cloud,pRGBDSlam->m_pucRGB,pRGBDSlam->m_pusDepth);
			if (!pRGBDSlam->m_bNewSyncDataArrive)
			{
				pRGBDSlam->m_PC=point_cloud;
				if (nBuffIdx==0)
				{
					point_cloud=point_cloudB;
					nBuffIdx=1;
				}
				else
				{
					point_cloud=point_cloudA;
					nBuffIdx=0;
				}
				pRGBDSlam->m_bNewSyncDataArrive=true;
			}
			pRGBDSlam->m_cbRGBDData(pRGBDSlam->m_pucRGB,pRGBDSlam->m_pusDepth,pRGBDSlam->m_pRGBDDataContext);
		}
		else
		{
			Sleep(20);
		}
	}
	return 0;
}




int IoTRobot_RGBDSLAM_Client::SLAMParamsSetting(void *pParam)
{
	IoTRobot_SLAM_SETTING *pstSetting=(IoTRobot_SLAM_SETTING*)pParam;
	char cTmp[32];
	map<string,string> SLAMParams;
	

	sprintf(cTmp,"%f",pstSetting->fMinTransVal);
	string tmp1(cTmp);
	SLAMParams.insert(pair<string,string>("min_trans",tmp1));

	sprintf(cTmp,"%f",pstSetting->fMaxTransVal);
	string tmp2(cTmp);
	SLAMParams.insert(pair<string,string>("max_trans",tmp2));

	sprintf(cTmp,"%f",pstSetting->fMinRotVal);
	string tmp3(cTmp);
	SLAMParams.insert(pair<string,string>("min_rot",tmp3));

	sprintf(cTmp,"%f",pstSetting->fMaxRotVal);
	string tmp4(cTmp);
	SLAMParams.insert(pair<string,string>("max_rot",tmp4));

	sprintf(cTmp,"%d",pstSetting->nGraphDegree);
	string tmp5(cTmp);
	SLAMParams.insert(pair<string,string>("graph_degree",tmp5));

	sprintf(cTmp,"%d",pstSetting->nInliner);
	string tmp6(cTmp);
	SLAMParams.insert(pair<string,string>("min_inlier",tmp6));

	sprintf(cTmp,"%d",pstSetting->nMinGraph);
	string tmp7(cTmp);
	SLAMParams.insert(pair<string,string>("min_graph_thres",tmp7));


	sprintf(cTmp,"%d",pstSetting->nMaxGraph);
	string tmp8(cTmp);
	SLAMParams.insert(pair<string,string>("max_graph_thres",tmp8));


	string tmp9;
	switch (pstSetting->nDecType)
	{
	case 0:
		SLAMParams.insert(pair<string,string>("detector_type","SURF"));
		break;
	case 1:
		SLAMParams.insert(pair<string,string>("detector_type","FAST"));
		break;
	case 2: 
		SLAMParams.insert(pair<string,string>("detector_type","HARRIS"));
		break;
	case 3: 
		SLAMParams.insert(pair<string,string>("detector_type","GFTT"));
		break;
	case 4: 
		SLAMParams.insert(pair<string,string>("detector_type","STAR"));
		break;
	case 5: 
		SLAMParams.insert(pair<string,string>("detector_type","PyramidFAST"));
		break;
	case 6: 
		SLAMParams.insert(pair<string,string>("detector_type","SIFT"));
		break;
	}
	
	m_CRGBDSLAM.IoTRobot_RGBDSLAM_SetParams(SLAMParams);

	return 0;
}