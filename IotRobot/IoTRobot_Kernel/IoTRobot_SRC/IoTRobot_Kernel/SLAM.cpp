//#include "stdafx.h"
#include "CommandAndControl.h"
#include "Point_Cloud.h"
#include "SLAM.h"
#include "Openni.h"
#include <time.h>
#include "Openni1.h"
#include <boost/shared_array.hpp>
#include "AreaStore.h"

CCriticalSection lock_to_write;

/*

bool CSLAM::m_bFrameReady;
bool CSLAM::m_bFinishFrame;
unsigned char *CSLAM::m_pucRGBBuff;
unsigned short *CSLAM::m_pusDepth;
bool g_bStartToRun=false;

int CSLAM::m_nCount;
unsigned int CSLAM::m_uiFrameNum;
CallBack_RunState CSLAM::m_cCallBackState;
double *CSLAM::m_pdIMUData;
extern DWORD g_ulRunTimeBuff[LOG_ARRAY_LEN][8];
boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > CSLAM::m_SLAMPC(new pcl::PointCloud<pcl::PointXYZRGB>);
vector<int> CSLAM::m_id_of_node2;
vector<vector<float> > CSLAM::m_matrix_of_nodeF;
bool CSLAM::m_bReadyforMB = false;

*/




void *g_PCLASS;
CSLAM::CSLAM():m_SLAMPC(new pcl::PointCloud<pcl::PointXYZRGB>)
{
	rebuild_cells = false;
	m_bManualMap = false;//true;//false;
	m_bUpdateManually = false;
}
CSLAM::CSLAM(void *pCCmdCtrl)
{
	//m_pCCmdCtrl=pCCmdCtrl;
}


CSLAM::~CSLAM()
{

}
void CSLAM::Reset()
{
	{
		boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > pTmp(new pcl::PointCloud<pcl::PointXYZRGB>());
		m_SLAMPC.swap(pTmp);
	}
	rebuild_cells = false;
	m_bManualMap = false;//true;//false;
	m_bUpdateManually = false;
	
	m_id_of_node2.clear();
	//id_of_node.clear();

	C3DMap* p3DMap=(C3DMap*)m_stClassPtrs.p3Dmap;
	p3DMap->Reset();
	CMapBuilder* pMapBuilder=(CMapBuilder*)m_stClassPtrs.pMapBuilder;
	pMapBuilder->Reset();


}

void CSLAM::NewCommand(const IoTRobot_Message MSG)
{
	switch (MSG.cCommand)
	{
	case SLAM_MSG_IMU:
		m_stSLAMSG.nApplyIMU=MSG.nParam1;
		break;
	case SLAM_MSG_BUILD_MAP_STYLE:
		m_stSLAMSG.nMapBuilderStyle=MSG.nParam1;
		if (m_stSLAMSG.nMapBuilderStyle==2)
		{
			m_bManualMap=true;
			m_bUpdateManually=true;
		}
		else // auto
		{
			m_bManualMap=false;
		}
		break;
	case SLAM_MSG_SETTINGS:
		m_stSLAMSG.nSLAMSettingAddr=MSG.nParam1;
		break;
	default:
		break;
	}
}


void CSLAM::SLAMInit(IoT_CallBackFuncSet *pstCallBackFuncSet)
{	
	m_bReadyforMB=false;
	m_hThreadRGBDSlam=0;
	m_bStopSlam=true;
}

void CSLAM::SLAMStop()
{
	m_bStopSlam=true;
	if (m_hThreadRGBDSlam!=0)
	{
			WaitForSingleObject(m_hThreadRGBDSlam,INFINITE);
	}

}

void CSLAM::SLAMUninit()
{
	SLAMStop();
}
void CSLAM::Callback_SLAM(unsigned char* pucSLAMData,void *pContext)
{
	//////////////////////////////////////////////////////////////////////
	// 4bits                   state
	// 4bits                   pc data len
	// pc data len bits        pc data
	int i,j;
	int nDataTotalLen,nAccDatalen=0;
	CSLAM *pSLAM=(CSLAM *)g_PCLASS;
	CMapBuilder* pMapbuilder = (CMapBuilder*)pSLAM->m_stClassPtrs.pMapBuilder;
	pSLAM->m_SLAMPC->points.clear();

	int nStt,nLen1,nLen2,nPointsNUm,nNodeNum,nNodeID,nSendNodeNum,nFeatureDataLen;
	float x,y,z,fEle;
	unsigned char r,g,b;
	unsigned char *pucTmp;
	static char cCBStt[200];
	pucTmp=pucSLAMData;

	//float fFeature[262144];

	float fWrongFlag=-99999;

	//Data total length
	memcpy(&nDataTotalLen,pContext,4);

	//Current SLAM Status
	memcpy(&nStt,pucTmp,4);
	memcpy(cCBStt,pucTmp,4);

	nAccDatalen+=4;
	if (nStt==0)
	{
		//点云信息
		pucTmp+=4;  //SLAM Status Occupy
		nAccDatalen+=4;
		memcpy(&nLen1,pucTmp,4);
		nAccDatalen+=nLen1;
		pucTmp+=4; //PC Number Occupy
		nPointsNUm=nLen1/15;

		for (i=0;i<nPointsNUm;i++)
		{
			pcl::PointXYZRGB p;
			memcpy(&x,pucTmp,4);
			pucTmp+=4;
			memcpy(&y,pucTmp,4);
			pucTmp+=4;
			memcpy(&z,pucTmp,4);
			pucTmp+=4;
			memcpy(&r,pucTmp,1);
			pucTmp+=1;
			memcpy(&g,pucTmp,1);
			pucTmp+=1;
			memcpy(&b,pucTmp,1);
			pucTmp+=1;
			p.x=x;
			p.y=y;
			p.z=z;
			p.r=r;
			p.g=g;
			p.b=b;
			pSLAM->m_SLAMPC->points.push_back(p);
		}


		//node信息
		memcpy(&nLen2,pucTmp,4);//Node Data Length
		nNodeNum=nLen2/68;
		pucTmp+=4; //Node Data Length occupy


		nAccDatalen+=4;
		nAccDatalen+=nLen2;
		for (i=0;i<nNodeNum;i++)
		{
			memcpy(&nNodeID,pucTmp,4);
			pucTmp+=4;
			pSLAM->m_id_of_node2.push_back(nNodeID);
			vector<float> vcMat;
			for (j=0;j<16;j++)
			{
				memcpy(&fEle,pucTmp,4);
				pucTmp+=4;
				vcMat.push_back(fEle);
			}
			pSLAM->m_matrix_of_nodeF.push_back(vcMat);
		}

		//node 数目

		//在传Node数之前把Session数传过去
		memcpy(cCBStt+4,&pMapbuilder->m_SessionID,4);

		pucTmp+=4;//直接跨过 Node Number Length
		memcpy(&nSendNodeNum,pucTmp,4);
		pucTmp+=4;//Node Number 占了4位

		memcpy(cCBStt+8,&nSendNodeNum,4);//Node number 负值回传给 GUI
		nAccDatalen+=8;


		//Feature Info
		// n: number of feature points		4
		// id: id of this node				4
		// xyz location of each feature		4*3
		// float[64] descriptor of feature	4*64
		memcpy(&nFeatureDataLen,pucTmp,4);
		pucTmp+=4;
		int id_of_cur_node;
		memcpy(&id_of_cur_node,pucTmp,4);
		pucTmp+=4;
		int nFeatureNum=nFeatureDataLen/268;//256;
		//memcpy(fFeature,pucTmp,nFeatureDataLen);
		std::vector<Feature3DDesc> tmpFeature;
		tmpFeature.resize(nFeatureNum);

		//*(pMapbuilder->m_pFeaturesNode)=(Node3DDesc*)realloc(*(pMapbuilder->m_pFeaturesNode),8+nFeatureNum*sizeof(Feature3DDesc));
		//Node3DDesc* pFeature=*(pMapbuilder->m_pFeaturesNode);
		//pFeature->n=nFeatureNum;
		//pFeature->id=id_of_cur_node;
		for(int i=0;i<nFeatureNum;i++)
		{
			memcpy(&tmpFeature[i].xyz,pucTmp,3*4);
			//memcpy(pFeature->desc3DList[i].xyz,pucTmp,3*4);
			pucTmp+=3*4;

			memcpy(tmpFeature[i].desc,pucTmp,64*4);
			//memcpy(pFeature->desc3DList[i].desc,pucTmp,64*4);
			pucTmp+=64*4;
		}
		pMapbuilder->Id_Node_Feature.insert(make_pair(id_of_cur_node,tmpFeature));
		pucTmp+=nFeatureDataLen;

		lock_to_write.Lock();
		boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > pPc(new pcl::PointCloud<pcl::PointXYZRGB>());
		pSLAM->m_SLAMPC.swap(pPc);
		pMapbuilder->Node_To_Cells_Back.insert(make_pair(*(pSLAM->m_id_of_node2.rbegin()),pPc));
		pMapbuilder->updateAllNodesBack(pSLAM->m_matrix_of_nodeF,pSLAM->m_id_of_node2);

		// Insert new ids of m_id_of_node into m_id_of_node_Back
		vector<int>::iterator it_first_l=pSLAM->m_id_of_node2.begin();
		while(1)
		{
			if(it_first_l==pSLAM->m_id_of_node2.end())
				break;
			// changed by ZH
			if(pMapbuilder->m_id_of_node_Back.size()>0 && it_first_l!=pSLAM->m_id_of_node2.end() && *it_first_l<=*(pMapbuilder->m_id_of_node_Back.rbegin()))
				it_first_l++;
			else if(it_first_l!=pSLAM->m_id_of_node2.end())
			{
				pMapbuilder->m_id_of_node_Back.insert(pMapbuilder->m_id_of_node_Back.end(),it_first_l,pSLAM->m_id_of_node2.end());
				break;
			}

		}
		pSLAM->m_id_of_node2.clear();
		pSLAM->m_matrix_of_nodeF.clear();

		cout<<"last node pose: in IMUCALLBACK"<<endl;
		std::map<int, CPose3D>::reverse_iterator it_last_node=pMapbuilder->Id_Cells_New_Pose_Back.rbegin();
		//memcpy(cCBStt+56,&fWrongFlag,4);
		memcpy(cCBStt+60,&fWrongFlag,4);
		if(it_last_node!=pMapbuilder->Id_Cells_New_Pose_Back.rend())
		{
			//printf("hahhahhahahahhahhahhahahahaahha  \n");
			//it_last_node->second.output(std::cout);
			/*memcpy(cCBStt+56,&it_last_node->second.m_coords[0],8);
			memcpy(cCBStt+64,&it_last_node->second.m_coords[1],8);
			memcpy(cCBStt+72,&it_last_node->second.m_coords[2],8);
			memcpy(cCBStt+80,&it_last_node->second.roll,8);
			memcpy(cCBStt+88,&it_last_node->second.pitch,8);
			memcpy(cCBStt+96,&it_last_node->second.yaw,8);*/

			memcpy(cCBStt+60,&it_last_node->second.m_coords[0],8);
			memcpy(cCBStt+68,&it_last_node->second.m_coords[1],8);
			memcpy(cCBStt+76,&it_last_node->second.m_coords[2],8);
			memcpy(cCBStt+84,&it_last_node->second.roll,8);
			memcpy(cCBStt+92,&it_last_node->second.pitch,8);
			memcpy(cCBStt+100,&it_last_node->second.yaw,8);
			pSLAM->m_cbState(cCBStt);
		}
		pSLAM->m_bReadyforMB = true;
		lock_to_write.Unlock();
	}
	else
	{
		memcpy(cCBStt+60,&fWrongFlag,4);
		pSLAM->m_cbState(cCBStt);
	}

	if (nDataTotalLen>nAccDatalen)
	{
		memcpy(cCBStt+12,pucSLAMData+nAccDatalen+4,48);
	}
}



UINT CSLAM::ThreadRGBDSlam(LPVOID lpParam)
{
	int nRtn=0;
	CSLAM *pCSLAM=(CSLAM *)lpParam;
	CPoint_Cloud  *pPointCould=(CPoint_Cloud *)pCSLAM->m_stClassPtrs.pPointCloud;
	CMapBuilder * pMapBuilder = (CMapBuilder*)pCSLAM->m_stClassPtrs.pMapBuilder;
	C3DMap* p3DMap = (C3DMap*)pCSLAM->m_stClassPtrs.p3Dmap;
	while (!pCSLAM->m_bStopSlam)
	{
		// will Load sessions from disk
		if(pMapBuilder->m_bLoadSessions)
		{
			pMapBuilder->LoadAllSessions("D:\\Work\\SessionFiles",p3DMap->m_Sessions);
			pMapBuilder->m_bLoadSessions=false;
			p3DMap->m_bOneRenderFrameReady=true;
		}
		// this is for Actual SLAM process
		if (pCSLAM->m_bReadyforMB)
		{
			if(pCSLAM->m_bManualMap)
			{
				if(pCSLAM->m_bUpdateManually)
				{
					pCSLAM->m_bUpdateManually = false;
					pCSLAM->m_bReadyforMB = false;
					lock_to_write.Lock();
					std::cout<<"Manually update Map, not move!!"<<std::endl;
					pMapBuilder->getdatafrombuf();
					lock_to_write.Unlock();
				}
				else
					continue;
			}
			else{
				pCSLAM->m_bReadyforMB = false;
				lock_to_write.Lock();
				pMapBuilder->getdatafrombuf();
				lock_to_write.Unlock();
			}

		//	if(!p3DMap->m_bGlobalShow){
				while (1)
				{
					if(!p3DMap->m_bOneRenderFrameReady)
					{

						// will Load sessions from disk
						if(pMapBuilder->m_bLoadSessions)
						{
							cout<<"Load Sessions!"<<endl;
							pMapBuilder->LoadAllSessions("D:\\Work\\Session1",p3DMap->m_Sessions);
							pMapBuilder->m_bLoadSessions=false;
						}

						if(pMapBuilder->m_bSessionSwitch)
						{
							pMapBuilder->SwitchSession(pMapBuilder->m_nfirstLastSession,pMapBuilder->m_nendLastSession);
							pMapBuilder->m_bSessionSwitch=false;
						}

						//std::cout<<"IN LOCAL "<<std::endl;
						//cout<<"last node pose in SLAM:"<<endl;
						pMapBuilder->Id_Cells_New_Pose.rbegin()->second.output(std::cout);
						if(pMapBuilder->IsexceedBounder(pMapBuilder->Id_Cells_New_Pose.rbegin()->second))
						{
							//cout<<"now we have to translate to new Area!"<<endl;
							pMapBuilder->TranslateArea(pMapBuilder->Id_Cells_New_Pose.rbegin()->second);
						}
						if (p3DMap->m_bisswapped)
						{
							p3DMap->DynamicUnswap();
						}
						p3DMap->valid_flag.reset();
						p3DMap->m_global_cell_map->points.clear();
						pMapBuilder->fromNodestoCell(); // Update All nodes to Cell
						// transform Pose info from Mapbuilder 2 C3DMap 
						p3DMap->m_robot_path=pMapBuilder->Id_Cells_Pose;
						p3DMap->RenderFrame();
						//std::cout<<"OUT LOCAL "<<std::endl;

						break;
					}
					else
						boost::this_thread::yield();
				}
		//	}
			//pCSLAM->m_bReadyforMB=false;
		}
		else
		{
			if(p3DMap->m_display_whole_session && !p3DMap->m_bisswapped ) // This command means to display a whole session
			{															 // and not swapped with original area
				p3DMap->m_display_whole_session=false;
		//		p3DMap->m_bGlobalShow=true; // continue to show global map
				//cout<<"We will dynamically display the whole Area!"<<endl;
				//cout<<"area x: ("<<p3DMap->m_abs_lower_x<<","<<p3DMap->m_abs_upper_x<<") y: ("<<p3DMap->m_abs_lower_y\
					<<","<<p3DMap->m_abs_upper_y<<") z: ("<<p3DMap->m_abs_lower_z<<","<<p3DMap->m_abs_upper_z<<")"<<endl;			
				// this swap is efficiently important
				while(1)
				{

					if(!p3DMap->m_bOneRenderFrameReady)
					{
						//std::cout<<"IN GLOBAL "<<std::endl;
						p3DMap->m_pcDynamicArea->reset(p3DMap->m_abs_lower_x,p3DMap->m_abs_upper_x,\
							p3DMap->m_abs_lower_y,p3DMap->m_abs_upper_y,p3DMap->m_abs_lower_z,p3DMap->m_abs_upper_z);
						//cout<<"after Reset!"<<endl;
						pMapBuilder->fromNodestoDynamicCell();
						//cout<<"after NodetoCell!"<<endl;
						p3DMap->DynamicEnswap();
						//std::cout<<"OUT GLOBAL "<<std::endl;
						break;
					}
					else
						boost::this_thread::yield();
				}
				p3DMap->RenderFrame();
			}
		}
	}

	// 将pose信息写入文件当中
	std::map<int, CPose3D>::iterator _node_pose=pMapBuilder->Id_Cells_Pose.begin();//Id_Cells_Pose.begin();
	while(_node_pose!=pMapBuilder->Id_Cells_Pose.end())
	{
		_node_pose->second.output(pMapBuilder->m_record_trajectory);
		_node_pose++;
	}

return 0;
}
void CSLAM::SLAMRun()
{
	ID_RGBDSlam=0;
	m_bStopSlam=false;
	g_PCLASS=(void*)this;
	m_hThreadRGBDSlam=CreateThread(NULL,0,(LPTHREAD_START_ROUTINE)ThreadRGBDSlam,this,0,ID_RGBDSlam);
}













/*


void CSLAM::CallBack_XYZRGB(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud,void *pContext)
{

}


UINT CSLAM::ThreadRGBDDynamicReceive(LPVOID lpParam)
{
int nRtn=0,i;
CSLAM *pCSLAM=(CSLAM *)lpParam;
COpenNI *pOpenNI=(COpenNI *)pCSLAM->m_stClassPtrs.pOpenNi;
struct RGBDData *pBuffList=pCSLAM->m_pstRGBDDataListHead;

for (i=0;i<MAX_STORE_BUFF_LEN;i++)
{
pCSLAM->m_pstRGBDDataCurWrite=pBuffList;
pCSLAM->m_pucRGBBuff=pBuffList->pucRGB;
pCSLAM->m_pusDepth=pBuffList->pusDepth;
pCSLAM->m_pdIMUData=pBuffList->dIMUData;
while(pOpenNI->GetOneFrame()!=0)
{
Sleep(10);
}
pCSLAM->m_bFrameReady=false;
while (!pCSLAM->m_bFrameReady)
{
Sleep(5);
}
pBuffList->bReady=true;
pBuffList=pBuffList->pstNext;
pCSLAM->m_nLeftBuffLen++;
Sleep(30);
}

pCSLAM->m_nFirstFrame=1;
while (1)
{
while (pCSLAM->m_nLeftBuffLen<MAX_STORE_BUFF_LEN)
{
if (!pBuffList->bReady&&!pBuffList->bUse)
{
//printf("List num reci:  %d  \n",pBuffList->nIdx);
pCSLAM->m_pstRGBDDataCurWrite=pBuffList;
pCSLAM->m_pucRGBBuff=pBuffList->pucRGB;
pCSLAM->m_pusDepth=pBuffList->pusDepth;
pCSLAM->m_pdIMUData=pBuffList->dIMUData;
while(pOpenNI->GetOneFrame()!=0)
{
Sleep(10);
}
pCSLAM->m_bFrameReady=false;
while (!pCSLAM->m_bFrameReady)
{
Sleep(5);
}
pBuffList->bReady=true;
pBuffList=pBuffList->pstNext;
pCSLAM->m_nLeftBuffLen++;
}
}
Sleep(30);
}
}


UINT CSLAM::ThreadRGBDDynamicManage(LPVOID lpParam)
{
int nRtn=0;
CSLAM *pCSLAM=(CSLAM *)lpParam;
COpenNI *pOpenNI=(COpenNI *)pCSLAM->m_stClassPtrs.pOpenNi;
int i,nCurrentBuffLen=0;
pCSLAM->m_pstRGBDDataReady=pCSLAM->m_pstRGBDDataListHead;

while (!pCSLAM->m_nFirstFrame)
{
Sleep(10);
}
while (1)
{
if (pCSLAM->m_bIWanaOneFrame)
{
if (pCSLAM->m_pstRGBDDataReady->bReady)
{

//pCSLAM->m_bIWanaOneFrame=false;
pCSLAM->m_pstRGBDDataReady->bUse=1;
//printf("List num mana1:  %d  \n",pCSLAM->m_pstRGBDDataReady->nIdx);
pCSLAM->m_pstRGBDDataCurRead=pCSLAM->m_pstRGBDDataReady;
//printf("List num mana2:  %d  \n",pCSLAM->m_pstRGBDDataCurRead->nIdx);
pCSLAM->m_pstRGBDDataReady=pCSLAM->m_pstRGBDDataReady->pstNext;
//printf("List num mana3:  %d  \n",pCSLAM->m_pstRGBDDataCurRead->nIdx);
pCSLAM->m_nLeftBuffLen--;
pCSLAM->m_bIWanaOneFrame=false;
}
}
Sleep(5);
}
}

void CSLAM::CallBack_SLAM_IMU(unsigned char*pucRGB,unsigned short *pusDepth, char *pcIMUData,void *pContext)
{
int i,j;
m_SLAMPC->points.clear();
CSLAM *pSLAM=(CSLAM *)g_PCLASS;
CMapBuilder* pMapbuilder = (CMapBuilder*)pSLAM->m_stClassPtrs.pMapBuilder;
int nStt,nLen1,nLen2,nPointsNUm,nNodeNum,nNodeID;
float x,y,z,fEle;
unsigned char r,g,b;
unsigned char *pucTmp;
pucTmp=pucRGB;
memcpy(&nStt,pucTmp,4);
if (nStt==0)
{
pucTmp+=4;
memcpy(&nLen1,pucTmp,4);
pucTmp+=4;
nPointsNUm=nLen1/15;

for (i=0;i<nPointsNUm;i++)
{
pcl::PointXYZRGB p;
memcpy(&x,pucTmp,4);
pucTmp+=4;
memcpy(&y,pucTmp,4);
pucTmp+=4;
memcpy(&z,pucTmp,4);
pucTmp+=4;
memcpy(&r,pucTmp,1);
pucTmp+=1;
memcpy(&g,pucTmp,1);
pucTmp+=1;
memcpy(&b,pucTmp,1);
pucTmp+=1;
p.x=x;
p.y=y;
p.z=z;
p.r=r;
p.g=g;
p.b=b;
m_SLAMPC->points.push_back(p);
}
memcpy(&nLen2,pucTmp,4);
nNodeNum=nLen2/68;
pucTmp+=4;

for (i=0;i<nNodeNum;i++)
{
memcpy(&nNodeID,pucTmp,4);
pucTmp+=4;
m_id_of_node2.push_back(nNodeID);
vector<float> vcMat;
for (j=0;j<16;j++)
{
memcpy(&fEle,pucTmp,4);
pucTmp+=4;
vcMat.push_back(fEle);
}
m_matrix_of_nodeF.push_back(vcMat);
}
lock_to_write.Lock();
boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > pPc(new pcl::PointCloud<pcl::PointXYZRGB>());
m_SLAMPC.swap(pPc);
pMapbuilder->Node_To_Cells_Back.insert(make_pair(*(m_id_of_node2.rbegin()),pPc));
pMapbuilder->updateAllNodesBack(m_matrix_of_nodeF,m_id_of_node2);
pMapbuilder->m_id_of_node_Back = m_id_of_node2;
m_bReadyforMB = true;
lock_to_write.Unlock();
}
m_cCallBackState((char) nStt);
}


void CSLAM::SLAMRun()
{
ID_RGBDSlam=0;
g_PCLASS=(void*)this;
CreateThread(NULL,0,(LPTHREAD_START_ROUTINE)ThreadRGBDSlam,this,0,ID_RGBDSlam);
}



void CSLAM::SLAMUninit()
{

}

*/















/*
int CSLAM::CreateRGBDDataBuffList(int nLen)
{
int i;
struct RGBDData *pList=NULL;
m_pstRGBDDataListHead=new struct RGBDData;
m_pstRGBDDataListHead->nIdx=0;
m_pstRGBDDataListHead->bReady=0;
m_pstRGBDDataListHead->bUse=0;
m_pstRGBDDataListHead->pucRGB=new unsigned char[640*480*3];
m_pstRGBDDataListHead->pusDepth=new unsigned short [640*480];

memset(m_pstRGBDDataListHead->pucRGB,0,640*480*3);
memset(m_pstRGBDDataListHead->pusDepth,0,640*480*2);

pList=m_pstRGBDDataListHead;
for (i=0;i<nLen-1;i++)
{
struct RGBDData *pTmp=new struct RGBDData;
pTmp->nIdx=i+1;
pTmp->bUse=0;
pTmp->bReady=0;
pTmp->pucRGB=new unsigned char[640*480*3];
pTmp->pusDepth=new unsigned short [640*480];

memset(pTmp->pucRGB,0,640*480*3);
memset(pTmp->pusDepth,0,640*480*2);
pList->pstNext=pTmp;
pList=pTmp;
}

pList->pstNext=m_pstRGBDDataListHead;
return 1;
}

int CSLAM::DeleteRGBDDataBuffList(int nLen)
{
int i;
struct RGBDData *pList=m_pstRGBDDataListHead;
struct RGBDData *pTmp=NULL;
for (i=0;i<nLen-1;i++)
{
pTmp=pList;
pList=pList->pstNext;
delete [] pTmp->pucRGB;
delete [] pTmp->pusDepth;
delete pTmp;
}
delete [] pList->pucRGB;
delete [] pList->pusDepth;
delete pList;
return 0;
}


void CSLAM::SLAMInit(IoT_CallBackFuncSet *pstCallBackFuncSet)
{
int i;
//	pstCallBackFuncSet->callBack_XYZRGB=CallBack_XYZRGB;
pstCallBackFuncSet->callBack_RGB24_Depth=CallBack_RGB24_Depth;
pstCallBackFuncSet->pXYZRGBContext=this;
m_nLeftBuffLen=0;
CreateRGBDDataBuffList(RGBD_DATA_BUFF_LEN);
m_pstRGBDDataCurRead=m_pstRGBDDataListHead;
m_cRGBDSlam.IoTRobot_RGBDSLAM_Init();
m_bFinishFrame=1;
m_uiFrameNum=0;
m_nFirstFrame=0;


m_pucRGBBuff=new unsigned char[640*480*3];
m_pusDepth=new unsigned short[640*480];

m_pfVertex=new float[640*480*3];
}
void CSLAM::SLAMUninit()
{
DeleteRGBDDataBuffList(RGBD_DATA_BUFF_LEN);
}

int CSLAM::TestPC(unsigned short *psDepth,float *pfPC)
{
int i,j,k,m;
m=0;
k=0;
for (i=-240;i<240;i++)
{
for (j=-320;j<320;j++)
{
if (psDepth[m]!=0)
{
pfPC[k+2]=1.0*(float)psDepth[m]/1000.0;
pfPC[k]=(float)j*0.001904*pfPC[k+2];
pfPC[k+1]=(float)i*0.001904*pfPC[k+2];
}
else
{
pfPC[k+2]=-1000000;
pfPC[k]=-1000000;
pfPC[k+1]=-100000;
}



if (pfPC[k+1]>0)
{
int fff=100;
}
k+=3;
m++;

}
}
return 0;
}





void CSLAM::CallBack_RGB24_Depth(unsigned char *pucRGB24,unsigned short *pusDepth,void *pContext)
{
//printf("SLAM callback run!!!! \n");
//	if (m_bFinishFrame)
{
m_bFinishFrame=0;
g_bStartToRun=1;
memcpy(m_pucRGBBuff,pucRGB24,640*480*3);
memcpy(m_pusDepth,pusDepth,640*480*2);
m_bFrameReady=true;

}
}
void CSLAM::CallBack_SLAM_IMU(unsigned char*pucRGB,unsigned short *pusDepth, char *pcIMUData,void *pContext)
{
int i,j;
m_SLAMPC->points.clear();
CSLAM *pSLAM=(CSLAM *)g_PCLASS;
CMapBuilder* pMapbuilder = (CMapBuilder*)pSLAM->m_stClassPtrs.pMapBuilder;
int nStt,nLen1,nLen2,nPointsNUm,nNodeNum,nNodeID;
float x,y,z,fEle;
unsigned char r,g,b;
unsigned char *pucTmp;
pucTmp=pucRGB;
memcpy(&nStt,pucTmp,4);
if (nStt==0)
{
pucTmp+=4;
memcpy(&nLen1,pucTmp,4);
pucTmp+=4;
nPointsNUm=nLen1/15;

for (i=0;i<nPointsNUm;i++)
{
pcl::PointXYZRGB p;
memcpy(&x,pucTmp,4);
pucTmp+=4;
memcpy(&y,pucTmp,4);
pucTmp+=4;
memcpy(&z,pucTmp,4);
pucTmp+=4;
memcpy(&r,pucTmp,1);
pucTmp+=1;
memcpy(&g,pucTmp,1);
pucTmp+=1;
memcpy(&b,pucTmp,1);
pucTmp+=1;
p.x=x;
p.y=y;
p.z=z;
p.r=r;
p.g=g;
p.b=b;
m_SLAMPC->points.push_back(p);
}
memcpy(&nLen2,pucTmp,4);
nNodeNum=nLen2/68;
pucTmp+=4;

for (i=0;i<nNodeNum;i++)
{
memcpy(&nNodeID,pucTmp,4);
pucTmp+=4;
m_id_of_node2.push_back(nNodeID);
vector<float> vcMat;
for (j=0;j<16;j++)
{
memcpy(&fEle,pucTmp,4);
pucTmp+=4;
vcMat.push_back(fEle);
}
m_matrix_of_nodeF.push_back(vcMat);
}
lock_to_write.Lock();
//cout<<"obtain pc of Node: "<<*m_id_of_node2.rbegin()<<endl;
boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > pPc(new pcl::PointCloud<pcl::PointXYZRGB>());
m_SLAMPC.swap(pPc);
// make buff to store 
pMapbuilder->Node_To_Cells_Back.insert(make_pair(*(m_id_of_node2.rbegin()),pPc));
//pMapbuilder->Node_To_Cells.insert(make_pair(*(m_id_of_node2.rbegin()),pPc));
//pMapbuilder->update1Node(*m_matrix_of_nodeF.rbegin(),*m_id_of_node2.rbegin());
pMapbuilder->updateAllNodesBack(m_matrix_of_nodeF,m_id_of_node2);
//pMapbuilder->updateAllNodes(m_matrix_of_nodeF,m_id_of_node2);
//pMapbuilder->m_id_of_node = m_id_of_node2;
pMapbuilder->m_id_of_node_Back = m_id_of_node2;
m_bReadyforMB = true;
lock_to_write.Unlock();
}
m_cCallBackState((char) nStt);
}

*/
/*
void CSLAM::CallBack_SLAM_IMU(unsigned char*pucRGB,unsigned short *pusDepth, char *pcIMUData,void *pContext)
{
m_bFinishFrame=0;
g_bStartToRun=1;
memcpy(m_pucRGBBuff,pucRGB,640*480*3);
memcpy(m_pusDepth,pusDepth,640*480*2);
memcpy(m_pdIMUData,pcIMUData,48);
m_bFrameReady=true;
}*/
/*
void CSLAM::getImagesandDepthMetaData(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> >& point_cloud,
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

UINT CSLAM::ThreadRGBDSlam(LPVOID lpParam)
{
int nRtn=0;
CSLAM *pCSLAM=(CSLAM *)lpParam;
CPoint_Cloud  *pPointCould=(CPoint_Cloud *)pCSLAM->m_stClassPtrs.pPointCloud;
CMapBuilder * pMapBuilder = (CMapBuilder*)pCSLAM->m_stClassPtrs.pMapBuilder;
COpenNI *pOpenNI=(COpenNI *)pCSLAM->m_stClassPtrs.pOpenNi;
C3DMap* p3DMap = (C3DMap*)pCSLAM->m_stClassPtrs.p3Dmap;
//int nRunTimeIdx=0;
DWORD *pulRunTimeId=NULL;
DWORD ulRunTime[6];
bool bProcessOneFrame;

while (!pCSLAM->m_nFirstFrame)
{
Sleep(10);
}
// 1. Initialize OpenNI Grabber and Mapbuilder
//cout<<"before OpenNIViewer！！"<<endl;
//SimpleOpenNIViewer v;
//v.startframe();
//v.start();

//cout<<"after OpenNIViewer!!"<<endl;

//// Get OpenNI visual image
//static unsigned rgb_array_size = 0;
//static boost::shared_array<unsigned char> rgb_array(0);
//static unsigned char* rgb_buffer = 0;
//rgb_array_size = 480*640*3; // size of each frame
//rgb_array.reset(new unsigned char [rgb_array_size]);
//rgb_buffer = rgb_array.get();


//// Get OpenNI depth image
//static unsigned depth_array_size = 0;
//static boost::shared_array<unsigned char> depth_array(0);
//static unsigned char* depth_buffer = 0;
//depth_array_size = 480*640*2;
//depth_array.reset(new unsigned char [depth_array_size]);
//depth_buffer = depth_array.get();

while(1)
while (1)
{
pCSLAM->m_bIWanaOneFrame=1;

while (pCSLAM->m_bIWanaOneFrame)
{
Sleep(5);
}

boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > point_cloud(new pcl::PointCloud<pcl::PointXYZRGB > ());

if(m_bReadyforMB)
{
// if Manually update Map
if(pCSLAM->m_bManualMap)
{
if(pCSLAM->m_bUpdateManually)
{
pCSLAM->m_bUpdateManually = false;
m_bReadyforMB = false;
lock_to_write.Lock();
std::cout<<"Manually update Map, not move!!"<<std::endl;
pMapBuilder->getdatafrombuf();
lock_to_write.Unlock();
}
else
continue;
}
else{
m_bReadyforMB = false;
lock_to_write.Lock();
pMapBuilder->getdatafrombuf();
lock_to_write.Unlock();
}
while (1)
{
if(!p3DMap->m_bOneRenderFrameReady)
{
p3DMap->valid_flag.reset();
p3DMap->m_global_cell_map->points.clear();
pMapBuilder->fromNodestoCell(); // Update All nodes to Cell
p3DMap->RenderFrame();
break;
}
else
boost::this_thread::yield();
}

}


while(v.getCloudPoint(point_cloud)){
//printf("Lis%t num use:  %d  \n",pCSLAM->m_pstRGBDDataCurRead->nIdx);
//printf("pCSLAM->m_nLeftBuffLen  :   %d  \n",pCSLAM->m_nLeftBuffLen);
g_ulRunTimeBuff[m_nCount][1]=::GetTickCount();
//boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > point_cloud(new pcl::PointCloud<pcl::PointXYZRGB > ());
Eigen::Matrix4f final_transformation;

// get depth_image and image_matadata
//pCSLAM->getImagesandDepthMetaData(point_cloud,pCSLAM->m_pstRGBDDataCurRead->pucRGB, pCSLAM->m_pstRGBDDataCurRead->pusDepth);

pCSLAM->getImagesandDepthMetaData(point_cloud,pCSLAM->m_pucRGBBuff, pCSLAM->m_pusDepth);


nRtn=pCSLAM->m_cRGBDSlam.IoTRobot_RGBDSLAM_RunOneFrame(pCSLAM->m_pucRGBBuff,
pCSLAM->m_pusDepth,point_cloud,final_transformation,
&g_ulRunTimeBuff[m_nCount][2],pCSLAM->m_pstRGBDDataCurRead->dIMUData,0,pCSLAM->rebuild_cells,pCSLAM->robotpath_update,pCSLAM->id_of_node);
//	pCSLAM->TestPC(pCSLAM->m_pstRGBDDataCurRead->pusDepth,pCSLAM->m_pfVertex);
//	pCSLAM->m_cbPointCould(pCSLAM->m_pstRGBDDataCurRead->pucRGB,pCSLAM->m_pfVertex,NULL);

//printf("SLAM DLL PASS!!!!   \n");
g_ulRunTimeBuff[m_nCount][3]=::GetTickCount();

m_cCallBackState((char) nRtn);
if (nRtn==0)
{
while (pPointCould->m_bNewFrame)
{
Sleep(1);
}

// rebuild whole cells
if(pCSLAM->rebuild_cells) 
{
pCSLAM->rebuild_cells = false;
pMapBuilder->transformAllNodes(pCSLAM->id_of_node,pCSLAM->robotpath_update);
pMapBuilder->refresh_cells = true;
}
g_ulRunTimeBuff[m_nCount][4]=::GetTickCount();
pPointCould->id_of_node = *(pCSLAM->id_of_node.rbegin());
cout<<"id of this node is:"<<pPointCould->id_of_node<<endl;
pPointCould->NewFrameCloudIn(point_cloud,&final_transformation);		
g_ulRunTimeBuff[m_nCount][7]=m_uiFrameNum;
m_nCount++;
m_uiFrameNum++;



if (m_nCount==LOG_ARRAY_LEN)
{
m_nCount=0;
}
bProcessOneFrame=true;
}
g_ulRunTimeBuff[m_nCount][6]=::GetTickCount();
pCSLAM->m_pstRGBDDataCurRead->bUse=false;
pCSLAM->m_pstRGBDDataCurRead->bReady=false;
Sleep(100);
}
}
return 0;
}*/
/*

void CSLAM::CallBack_RGB24_Depth(unsigned char *pucRGB24,unsigned short *pusDepth,void *pContext)
{
//printf("SLAM callback run!!!! \n");
//	if (m_bFinishFrame)
{
m_bFinishFrame=0;
g_bStartToRun=1;
memcpy(m_pucRGBBuff,pucRGB24,640*480*3);
memcpy(m_pusDepth,pusDepth,640*480*2);
m_bFrameReady=true;

}
}

UINT CSLAM::ThreadRGBDSlam(LPVOID lpParam)
{
int nRtn=0;
CSLAM *pCSLAM=(CSLAM *)lpParam;
CPoint_Cloud  *pPointCould=(CPoint_Cloud *)pCSLAM->m_stClassPtrs.pPointCloud;
COpenNI *pOpenNI=(COpenNI *)pCSLAM->m_stClassPtrs.pOpenNi;
//int nRunTimeIdx=0;
DWORD *pulRunTimeId=NULL;
DWORD ulRunTime[6];
bool bProcessOneFrame;


//pulRunTimeId=m_ulRunTimeBuff;
while(pOpenNI->GetOneFrame()!=0)
{
Sleep(10);
}

while (1)
{
if (m_bFrameReady)
{
printf("get time log Start!!!!   \n");
if (pCSLAM->m_nFirstFrame==1)
{
g_ulRunTimeBuff[0][0]=0;
pCSLAM->m_nFirstFrame=0;
}
else
{
if (bProcessOneFrame)
{
if(m_nCount==0)
{
g_ulRunTimeBuff[0][0]=g_ulRunTimeBuff[LOG_ARRAY_LEN-1][6];
}
else g_ulRunTimeBuff[m_nCount][0]=g_ulRunTimeBuff[m_nCount-1][6];
}
else
{
g_ulRunTimeBuff[m_nCount][0]=g_ulRunTimeBuff[m_nCount][6];
}
}

printf("get time log PASS!!!!   \n");
bProcessOneFrame=0;
g_ulRunTimeBuff[m_nCount][1]=::GetTickCount();
m_bFinishFrame=1;
m_bFrameReady=0;

printf("SLAM DLL Start!!!!   \n");
boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> point_cloud(new pcl::PointCloud<pcl::PointXYZRGB > ());
Eigen::Matrix4f final_transformation;
nRtn=pCSLAM->m_cRGBDSlam.IoTRobot_RGBDSLAM_RunOneFrame(m_pucRGBBuff,m_pusDepth,point_cloud,final_transformation,&g_ulRunTimeBuff[m_nCount][2]);

printf("SLAM DLL PASS!!!!   \n");
g_ulRunTimeBuff[m_nCount][3]=::GetTickCount();


printf("SLAM222222222222 DLL Start!!!!   \n");
m_cCallBackState((char) nRtn);
if (nRtn==0)
{
while (pPointCould->m_bNewFrame)
{
Sleep(1);
}
g_ulRunTimeBuff[m_nCount][4]=::GetTickCount();
pPointCould->NewFrameCloudIn(point_cloud,&final_transformation);

g_ulRunTimeBuff[m_nCount][7]=m_uiFrameNum;
m_nCount++;
m_uiFrameNum++;


if (m_nCount==LOG_ARRAY_LEN)
{
m_nCount=0;
}
bProcessOneFrame=true;
}
g_ulRunTimeBuff[m_nCount][6]=::GetTickCount();

printf("SLAM222222222222 DLL end!!!!   \n");
pOpenNI->GetOneFrame();
}
Sleep(10);
}

return 0;
}

*/

/*
void CSLAM::SLAMRun()
{
ID_RGBDSlam=0;
ID_RGBDDynamicManage=0;
ID_RGBDDynamicReceive=0;
g_PCLASS=(void*)this;
CreateThread(NULL,0,(LPTHREAD_START_ROUTINE)ThreadRGBDSlam,this,0,ID_RGBDSlam);
CreateThread(NULL,0,(LPTHREAD_START_ROUTINE)ThreadRGBDDynamicManage,this,0,ID_RGBDDynamicManage);
CreateThread(NULL,0,(LPTHREAD_START_ROUTINE)ThreadRGBDDynamicReceive,this,0,ID_RGBDDynamicReceive);
}*/



/*void CSLAM::CallBack_SLAM_IMU(unsigned char*pucRGB,unsigned short *pusDepth, char *pcIMUData,void *pContext)
{
int i,j;
int nDataTotalLen,nAccDatalen=0;
//m_SLAMPC->points.clear();
CSLAM *pSLAM=(CSLAM *)g_PCLASS;
CMapBuilder* pMapbuilder = (CMapBuilder*)pSLAM->m_stClassPtrs.pMapBuilder;
pSLAM->m_SLAMPC->points.clear();

int nStt,nLen1,nLen2,nPointsNUm,nNodeNum,nNodeID,nSendNodeNum;
float x,y,z,fEle;
unsigned char r,g,b;
unsigned char *pucTmp;
static char cCBStt[104];
pucTmp=pucRGB;

float fWrongFlag=-99999;
memcpy(&nDataTotalLen,pcIMUData,4);
memcpy(&nStt,pucTmp,4);
memcpy(cCBStt,pucTmp,4);
nAccDatalen+=4;
if (nStt==0)
{
//点云信息
pucTmp+=4;
nAccDatalen+=4;
memcpy(&nLen1,pucTmp,4);
nAccDatalen+=nLen1;
pucTmp+=4;
nPointsNUm=nLen1/15;

for (i=0;i<nPointsNUm;i++)
{
pcl::PointXYZRGB p;
memcpy(&x,pucTmp,4);
pucTmp+=4;
memcpy(&y,pucTmp,4);
pucTmp+=4;
memcpy(&z,pucTmp,4);
pucTmp+=4;
memcpy(&r,pucTmp,1);
pucTmp+=1;
memcpy(&g,pucTmp,1);
pucTmp+=1;
memcpy(&b,pucTmp,1);
pucTmp+=1;
p.x=x;
p.y=y;
p.z=z;
p.r=r;
p.g=g;
p.b=b;
pSLAM->m_SLAMPC->points.push_back(p);
}
memcpy(&nLen2,pucTmp,4);
nNodeNum=nLen2/68;
pucTmp+=4;

//node信息
nAccDatalen+=4;
nAccDatalen+=nLen2;
for (i=0;i<nNodeNum;i++)
{
memcpy(&nNodeID,pucTmp,4);
pucTmp+=4;
pSLAM->m_id_of_node2.push_back(nNodeID);
vector<float> vcMat;
for (j=0;j<16;j++)
{
memcpy(&fEle,pucTmp,4);
pucTmp+=4;
vcMat.push_back(fEle);
}
pSLAM->m_matrix_of_nodeF.push_back(vcMat);
}

//node 数目
nAccDatalen+=8;

//如果有imu信息


pucTmp+=4;
memcpy(&nSendNodeNum,pucTmp,4);
pucTmp+=4;
memcpy(cCBStt+4,&nSendNodeNum,4);

lock_to_write.Lock();
boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > pPc(new pcl::PointCloud<pcl::PointXYZRGB>());
pSLAM->m_SLAMPC.swap(pPc);
pMapbuilder->Node_To_Cells_Back.insert(make_pair(*(pSLAM->m_id_of_node2.rbegin()),pPc));
pMapbuilder->updateAllNodesBack(pSLAM->m_matrix_of_nodeF,pSLAM->m_id_of_node2);
pMapbuilder->m_id_of_node_Back = pSLAM->m_id_of_node2;

cout<<"last node pose: in IMUCALLBACK"<<endl;
std::map<int, CPose3D>::reverse_iterator it_last_node=pMapbuilder->Id_Cells_New_Pose_Back.rbegin();
memcpy(cCBStt+56,&fWrongFlag,4);
if(it_last_node!=pMapbuilder->Id_Cells_New_Pose_Back.rend())
{
printf("hahhahhahahahhahhahhahahahaahha  \n");
it_last_node->second.output(std::cout);
memcpy(cCBStt+56,&it_last_node->second.m_coords[0],8);
memcpy(cCBStt+64,&it_last_node->second.m_coords[1],8);
memcpy(cCBStt+72,&it_last_node->second.m_coords[2],8);
memcpy(cCBStt+80,&it_last_node->second.roll,8);
memcpy(cCBStt+88,&it_last_node->second.pitch,8);
memcpy(cCBStt+96,&it_last_node->second.yaw,8);
pSLAM->m_cbState(cCBStt);
}
pSLAM->m_bReadyforMB = true;
lock_to_write.Unlock();
}
else
{
memcpy(cCBStt+56,&fWrongFlag,4);
pSLAM->m_cbState(cCBStt);
}

if (nDataTotalLen>nAccDatalen)
{
memcpy(cCBStt+8,pucRGB+nAccDatalen+4,48);
}
//pSLAM->m_cbState(cCBStt);
}
*/