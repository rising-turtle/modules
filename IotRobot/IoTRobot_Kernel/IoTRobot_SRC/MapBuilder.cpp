//#include "stdafx.h"
#include "MapBuilder.h"
#include "3DMap.h"
#include "Point_Cloud.h"


unsigned char *CMapBuilder::m_pucGetImgPtr;
CMapBuilder::CMapBuilder():refresh_cells(false)
{
	m_pucGetImgPtr=0;
}


CMapBuilder::~CMapBuilder()
{

}

void CMapBuilder::GetRGB24Data(unsigned char *pucRGB24)
{
	//CPoint_Cloud::ExternalCall(pucRGB24);
}


int CMapBuilder::NewGlobalPCIn(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > &cloud, int id_of_node)
{
	MergePointCloud2Area(cloud,id_of_node);
	return 0;
}

void CMapBuilder::cloudRegistration(boost::shared_ptr<CPose3D>& pose, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud)
{
	// --------------------------------------------------------------------------
	//  SPECIAL CASE OF HORIZONTAL SCAN: QUICKER IMPLEMENTATION
	// --------------------------------------------------------------------------
	Eigen::Matrix4f	HM;
	pose->getHomogeneousMatrix(HM);
	CPoint_Cloud* pcPointCloud = (CPoint_Cloud*)m_stClassPtrs.pPointCloud;

	pcPointCloud->cloudRegistration(pose,cloud);

	/*using pcl::transformation instead*/ 
//	pcl::transformPointCloud (*cloud, *cloud, HM);
}

void CMapBuilder::MergeLMapwithGMap(pcl::PointCloud<pcl::PointXYZRGB> * global_map,pcl::PointCloud<pcl::PointXYZRGB> * local_map){
	size_t index = global_map->points.size();
	global_map->points.resize(global_map->points.size() + local_map->points.size());
	for(size_t i=0;i<local_map->points.size();i++)
		global_map->points[index+i] = local_map->points[i];
	return ;	
}

// transform all nodes' position
void CMapBuilder::transformAllNodes(std::vector<int>& id_of_node, std::vector<CPose3D>& robotpath_update)
{
	cout<<"This is transform in every Node!!"<<endl;
	for(size_t i=0;i<id_of_node.size();i++)
	{
		transformPointsofNode(id_of_node[i],robotpath_update[i]);
	}
}
// transform pc of this node into a global pc
void CMapBuilder::transformPointsofNode(int id_of_node, CPose3D& pose){

	C3DMap *pc3Dmap=(C3DMap *)m_stClassPtrs.p3Dmap; 

	Node_pc::iterator it_of_node = Node_To_Cells.find(id_of_node);
	if(it_of_node == Node_To_Cells.end())
	{
		cout<<"!!! id_of node "<<id_of_node<<" does not exist in Cells!!!"<<endl;
		return ;
	}
	std::map<int, CPose3D>::iterator it_cell_pose = Id_Cells_Pose.find(id_of_node);
	if(it_cell_pose == Id_Cells_Pose.end())
	{	
		cout<<"!!! id of node "<<id_of_node<<" does not exist in Pose!!!"<<endl;
		return ;
	}
	CPose3D trans_pose = pose - it_cell_pose->second; // trans_pose
	
	trans_pose.output(std::cout);
	it_cell_pose->second = pose;
	boost::shared_ptr<CPose3D> pPose(new CPose3D(trans_pose));
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc = it_of_node->second;
	cloudRegistration(pPose,pc);
	//MergeLMapwithGMap(pc3Dmap->pGlobalDisplay.get(),pc.get());
}



bool CMapBuilder::IsNoisePoint(int index,boost::shared_ptr<pcl::PointXYZRGB>& p){

	C3DMap *pc3Dmap=(C3DMap *)m_stClassPtrs.p3Dmap; 
	CPoint_Cloud *pcPointCloud=(CPoint_Cloud *)m_stClassPtrs.pPointCloud;
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

	/*if(l_x >=0 && l_x <ALL_CELLS){
		sp =pc3Dmap->m_global_cells[l_x];
		if(pc3Dmap->valid_flag[l_x] && sp.get() != NULL)
			if( fabs(sp->rgb - p->rgb)<error_noise)
				return true;
	}
	if(r_x >=0 && r_x <ALL_CELLS){
		sp = pc3Dmap->m_global_cells[r_x];
		if(pc3Dmap->valid_flag[r_x] && sp.get() != NULL)
			if(fabs(sp->rgb - p->rgb)<error_noise)
				return true;
	}
	if(l_y >=0 && l_y <ALL_CELLS){
		sp = pc3Dmap->m_global_cells[l_y];
		if(pc3Dmap->valid_flag[l_y] && sp.get() != NULL)
			if(fabs(sp->rgb - p->rgb)<error_noise)
				return true;
	}
	if(r_y >=0 && r_y <ALL_CELLS){
		sp = pc3Dmap->m_global_cells[r_y];
		if(pc3Dmap->valid_flag[r_y] && sp.get() != NULL)
			if(fabs(sp->rgb - p->rgb)<error_noise)
				return true;
	}*/

	for(int i= index - range_z; i<= index + range_z; i++)
	{
		if( i < 0 || i >=ALL_CELLS  )
			continue;
		sp = pc3Dmap->m_global_cells[i];
		if(pc3Dmap->valid_flag[i] && sp.get()!=NULL)
			if(fabs(sp->rgb - p->rgb) < error_noise)
				return true;
	}
	return false;
}


int CMapBuilder::TMat_RAngle(float *pfTMat,float *pfRAngle)
{
	C3DMap *pc3Dmap=(C3DMap *)m_stClassPtrs.p3Dmap; 
	memcpy(pc3Dmap->m_fTMat,pfTMat,12);
	memcpy(pc3Dmap->m_fRAngle,pfRAngle,12);
	return 0;
}


void CMapBuilder::MergePointCloud2Area(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > &cloud, int id_of_node)
{

	C3DMap *pc3Dmap=(C3DMap *)m_stClassPtrs.p3Dmap; 
	int N = cloud->points.size();
	/*vector<bool> index_point(N,false);
	vector<int> index_set;*/
	
	static bitset<ALL_CELLS> cur_frame;
	cur_frame.reset();

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc(new pcl::PointCloud<pcl::PointXYZRGB>);

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
			if(!IsNoisePoint(index,p))
			{
				if(!cur_frame[index]){
					cur_frame.set(index);
					pc3Dmap->m_global_cells[index] = p;
					pc3Dmap->m_global_cell_map->points.push_back(sp);
					pc->points.push_back(sp);
				}
			}
		}
	}
	cout<<"!!! N of points into Node: "<< pc->points.size()<<endl;
	if(id_of_node != -1)
		Node_To_Cells.insert(make_pair(id_of_node,pc));
	pc3Dmap->valid_flag |= cur_frame; // 


	/*for(size_t i=0,j=0;i<N;i++)
		if(index_point[i])
		{
			pcl::PointXYZRGB& sp = cloud->points[i];
			boost::shared_ptr<pcl::PointXYZRGB> p(new pcl::PointXYZRGB);
			p->x = sp.x; p->y = sp.y; p->z = sp.z; 
			p->r = sp.r; p->g = sp.g; p->b = sp.b;
			int index_cell = index_set[j++];
			if(pc3Dmap->m_global_cells[index_cell].get()== NULL){
				pc3Dmap->m_global_cells[index_cell] = p;
				pc3Dmap->m_global_cell_map->points.push_back(sp);
			}
		}*/
		
	printf("MB Pass  !!!!\n");
	if(id_of_node != -1)
		pc3Dmap->RenderFrame();
}


void CMapBuilder::NewCommand(const IoTRobot_Message MSG)
{
	switch (MSG.cCommand)
	{
	case MAP_BUILDER_MSG_MSG1:
		stMapBuilderMSG.nMsg1=MSG.nParam1;
		break;
	default:
		break;
	}
	//memcpy(&m_stAlgOpt.cAlgorithmFlag,pCmd,ALGORITHM_NUM);
}