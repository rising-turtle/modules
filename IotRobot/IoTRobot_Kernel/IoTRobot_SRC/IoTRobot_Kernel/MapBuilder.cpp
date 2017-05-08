//#include "stdafx.h"
#include "MapBuilder.h"
#include "3DMap.h"
#include "Point_Cloud.h"
#include "CSession.h"

int CMapBuilder::m_iCellSize=2;

unsigned char *CMapBuilder::m_pucGetImgPtr;
CMapBuilder::CMapBuilder():refresh_cells(false),m_pBasicPose(new CPose3D())
{
	m_pucGetImgPtr=0;
	m_num_of_new_nodes=0;
	m_offset_x=0;
	m_offset_y=0;
	m_offset_z=0;
	calcbounder();

	m_SessionStore.LoadConfig("D:\\Work\\SessionFiles");
	m_SessionID=0;
	m_bSessionSwitch=false;
	// these below are for session switch
	m_nfirstLastSession=0;
	m_nendLastSession=0;
	//m_pFeaturesNode=(Node3DDesc**)malloc(sizeof(Node3DDesc*));
	//*m_pFeaturesNode=(Node3DDesc*)malloc(sizeof(Node3DDesc));

	// these below are for Load Sessions
	m_bLoadSessions=false;//true;//false;//true;

	// 记录每个session里面的路径信息
	m_record_trajectory.open("d:\\trajectory.txt");
}
CMapBuilder::~CMapBuilder()
{
	//free(*m_pFeaturesNode);
	//free(m_pFeaturesNode);
	m_record_trajectory.close();
}
void CMapBuilder::Reset()
{
	// Reset Dynamic Display 
	// coordinates translation
	{
		boost::shared_ptr<CPose3D> pTmp(new CPose3D);
		m_pBasicPose.swap(pTmp);
	}
	m_num_of_new_nodes=0;
	m_offset_x=0;
	m_offset_y=0;
	m_offset_z=0;
	calcbounder();

	// Reset Session Switch
	m_bSessionSwitch=false;

	// 2012/3/5_ZH 
	// here we do not want to override previous sessions in our current experiment
	//m_SessionID=0


	//free(*m_pFeaturesNode);
	//free(m_pFeaturesNode);
	m_nfirstLastSession=0;
	m_nendLastSession=0;
	//m_pFeaturesNode=(Node3DDesc**)malloc(sizeof(Node3DDesc*));
	//*m_pFeaturesNode=(Node3DDesc*)malloc(sizeof(Node3DDesc));

	// Reset Data Buf from Client and PC, Pose, Id, 
	Id_Cells_Id.clear();
	Id_Cells_Pose.clear();
	Id_Cells_New_Pose.clear();
	Id_Cells_New_Pose_Back.clear();
	m_id_of_node.clear();
	m_id_of_node_Back.clear();	
	Node_To_Cells.clear();
	Node_To_Cells_Back.clear();
	Id_Node_Feature.clear();
}
void CMapBuilder::GetRGB24Data(unsigned char *pucRGB24)
{
	//CPoint_Cloud::ExternalCall(pucRGB24);
}

void CMapBuilder::calcbounder()
{
	double xyz[3];
	m_pBasicPose->getXYZ(xyz);
	m_lower_x=xyz[0]-L_RX;//+0.04;
	m_lower_y=xyz[1]-L_RY;//+0.04;
	m_lower_z=xyz[2]-L_RZ;//+0.04;
	m_upper_x=xyz[0]+L_RX;
	m_upper_y=xyz[1]+L_RY;
	m_upper_z=xyz[2]+L_RZ;

	// for debug
	/*cout<<"new bounder: x "<<m_lower_x<<"-"<<m_upper_x \
		<<";"<<m_lower_y<<"-"<<m_upper_y \
		<<";"<<m_lower_z<<"-"<<m_upper_z<<endl;*/
}
void CMapBuilder::TranslateArea(CPose3D& trans)
{
	C3DMap *pc3Dmap=(C3DMap *)m_stClassPtrs.p3Dmap; 
	//*(this->m_pBasicPose)+=trans;
	*(this->m_pBasicPose)=trans;
	calcoffset();
	calcbounder();
	// clear points in cell and PC
	//pc3Dmap->valid_flag.reset();
	//boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > p_tmp_pc(new pcl::PointCloud<pcl::PointXYZRGB>);
	//pc3Dmap->m_global_cell_map.swap(p_tmp_pc);
	//pc3Dmap->FromPC2Cell(p_tmp_pc);
}

// Calculate offset based on current Basic Point
void CMapBuilder::calcoffset()
{
	double xyz[3];
	m_pBasicPose->getXYZ(xyz);
	m_offset_x = (xyz[0]*100);// + S_RX); //> >2;/// CELLSIZE;
	m_offset_x>>=2;
	m_offset_y = (xyz[1]*100);// + S_RY); //> >2;/// CELLSIZE;
	m_offset_y>>=2;
	m_offset_z = (xyz[2]*100);// + S_RZ); //> >2;/// CELLSIZE;
	m_offset_z>>=2;

	//for debug
	//cout<<"current Basic Point:"<<endl;
	//m_pBasicPose->output(std::cout);
	//cout<<"offset:(x,y,z) "<<m_offset_x<<" "<<m_offset_y<<" "<<m_offset_z<<endl;
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

	if(l_x >=0 && l_x <ALL_CELLS){
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
	}

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
	
	static boost::dynamic_bitset<> cur_frame;
	cur_frame.resize(ALL_CELLS,false);

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
	//cout<<"!!! N of points into Node: "<< pc->points.size()<<endl;
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
		
	//printf("MB Pass  !!!!\n");
	if(id_of_node != -1)
		pc3Dmap->RenderFrame();
}
bool CMapBuilder::IsexceedBounder(CPose3D& curPose) // Whether robot almost exceed Boundary
{	
	double cxyz[3];
	curPose.getXYZ(cxyz);
	static double tor_x=1.0;	//These should be parameterized
	static double tor_y=0.2;
	static double tor_z=1.0;
	if(fabs(cxyz[0]-m_upper_x)<tor_x || fabs(cxyz[0]-m_lower_x)<tor_x \
		|| fabs(cxyz[1]-m_upper_y)<tor_y || fabs(cxyz[1]-m_lower_y)<tor_y\
		|| fabs(cxyz[2]-m_upper_z)<tor_z || fabs(cxyz[2]-m_lower_z)<tor_z)
		return true;
	return false;
}
void CMapBuilder::fromNodestoCell()
{
	C3DMap *pc3Dmap=(C3DMap *)m_stClassPtrs.p3Dmap; 
	vector<int>::iterator it_id_node = m_id_of_node.begin();
	int last_node = -1;
	for(; it_id_node!=m_id_of_node.end(); it_id_node++)
	{
		if(last_node == *it_id_node)
			continue;

		Node_pc::iterator it_of_node = Node_To_Cells.find(*it_id_node);
		if(it_of_node == Node_To_Cells.end())
		{
			cout<<"!!! id_of node "<<*it_id_node<<" does not exist in Cells!!!"<<endl;
			return ;
		}
		std::map<int, CPose3D>::iterator it_cell_pose = Id_Cells_Pose.find(*it_id_node);
		if(it_cell_pose == Id_Cells_Pose.end())
		{	
			cout<<"!!! id of node "<<*it_id_node<<" does not exist in Pose!!!"<<endl;
			return ;
		}
		std::map<int, CPose3D>::iterator it_new_cell_pose = Id_Cells_New_Pose.find(*it_id_node);
		if(it_new_cell_pose == Id_Cells_New_Pose.end())
		{
			cout<<"!!! id of node "<<*it_id_node<<" does not exit in New_Pose!!!"<<endl;
			continue;
		}
		CPose3D transpose = it_new_cell_pose->second - it_cell_pose->second;
		
		//cout<<"node "<<*it_id_node<<": "<<endl;
		//transpose.output(std::cout);

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc = it_of_node->second;
		#define ZERO_THRESH 1e-4
		double xyz[3],rpy[3];
		transpose.getXYZ(xyz);
		transpose.getrpy(rpy);
		if(fabs(rpy[0])<ZERO_THRESH && fabs(rpy[1])<ZERO_THRESH && fabs(rpy[2])<ZERO_THRESH \
			&& fabs(xyz[0])<ZERO_THRESH && fabs(xyz[1])<ZERO_THRESH && fabs(xyz[2])<ZERO_THRESH)
		{
			//cout<<"node "<<*it_id_node<<" no change!"<<endl;
		}
		else
		{
			//cout<<"node "<<*it_id_node<<": "<<endl;
			//transpose.output(std::cout);
			it_cell_pose->second = it_new_cell_pose->second;
			boost::shared_ptr<CPose3D> pPose(new CPose3D(transpose));
			cloudRegistration(pPose,pc);
		}
		//it_cell_pose->second = it_new_cell_pose->second;
		//boost::shared_ptr<CPose3D> pPose(new CPose3D(transpose));
		//cloudRegistration(pPose,pc);

		pc3Dmap->FromPC2Cell(pc);

		last_node = *it_id_node;
	}
}

void CMapBuilder::findMaxAndMin(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > &cloud,float& l_x,\
								float& u_x,float& l_y,float& u_y,float& l_z,float& u_z)	// to obtain max/min in a cloud
{	for(size_t i=0;i<cloud->points.size();i++)
	{
		pcl::PointXYZRGB& sp = cloud->points[i];
		LOWER_ASS(l_x,sp.x);
		LOWER_ASS(l_y,sp.y);
		LOWER_ASS(l_z,sp.z);

		UPPER_ASS(u_x,sp.x);
		UPPER_ASS(u_y,sp.y);
		UPPER_ASS(u_z,sp.z);
	}
}

void CMapBuilder::fromNodestoDynamicCell()
{
	C3DMap *pc3Dmap=(C3DMap *)m_stClassPtrs.p3Dmap; 
	vector<int>::iterator it_id_node = m_id_of_node.begin();
	int last_node = -1;
	//pc3Dmap->m_pcDynamicArea->m_valid_flag.reset();
	for(; it_id_node!=m_id_of_node.end(); it_id_node++)
	{
		if(last_node == *it_id_node)
			continue;

		Node_pc::iterator it_of_node = Node_To_Cells.find(*it_id_node);
		if(it_of_node == Node_To_Cells.end())
		{
			cout<<"!!! id_of node "<<*it_id_node<<" does not exist in Cells!!!"<<endl;
			return ;
		}
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc = it_of_node->second;
		//cout<<"id_of_node="<<*it_id_node;
		//pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc1 = it_of_node->second;
		//cout<<" pc.size="<<pc1->points.size()<<endl;
		pc3Dmap->m_pcDynamicArea->FromPC2Cell(pc);

		last_node = *it_id_node;
	}
}
void CMapBuilder::TransfromSessionNodes2Cell(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > & in_pc,\
								boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > & out_pc)
{
	C3DMap *pc3Dmap=(C3DMap *)m_stClassPtrs.p3Dmap; 
	CMapBuilder* pMapbuilder=(CMapBuilder*)m_stClassPtrs.pMapBuilder;
	//int N = in_pc->points.size();
	out_pc->points.insert(out_pc->points.end(),in_pc->points.begin(),in_pc->points.end());
	cout<<"out_pc size()="<<out_pc->points.size()<<endl;
}
// Load session from direcotry 
void CMapBuilder::LoadAllSessions(string dir, vector<boost::shared_ptr<CSession> >& sessions)
{
	CAreaStore m_LoadSession;
	m_LoadSession.LoadConfig(dir);
	
	// get store header list
	AreaStoreHdrList hdrList;
	m_LoadSession.GetStoreHdrList(hdrList);

	// file2area
	for(int i=0; i<1/*hdrList.size()*/; i++)
	{
		boost::shared_ptr<CSession> tmp_session(new CSession);
		tmp_session->m_fhdr=hdrList[i];
		//pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc(new pcl::PointCloud<pcl :: PointXYZRGB>());
		m_LoadSession.File2AreaMap(tmp_session->m_pc, hdrList[i].id);
		//Area3DDescMap descMap;
		m_LoadSession.File2Area3DDesc(tmp_session->m_descMap, hdrList[i].id);
		//AreaPathList pathList;
		m_LoadSession.File2AreaPath(tmp_session->m_pathList, hdrList[i].id);
		sessions.push_back(tmp_session);
	}
	for(size_t i=0;i<sessions.size();i++)
	{
		//sessions[i].GeneratePlanes2(); //
		//sessions[i].RenderPlanes();
		sessions[i]->GeneratePlanes3();
		//sessions[i].GeneratePlanes4();
	}
	cout<<"add "<<sessions.size()<<" sessions to display!"<<endl;
}

// record last number of first node in last session
void CMapBuilder::SwitchSession(int beginid,int endid)
{
	C3DMap* p3DMap = (C3DMap*)this->m_stClassPtrs.p3Dmap;
	Area3DDescMap descMap;
	AreaPathList pathList;
	pcl::PointCloud<pcl :: PointXYZRGB>::Ptr pcSession(new 	pcl::PointCloud<pcl :: PointXYZRGB>);

	// 1. Dump pc, path list, feature list, into file
	Node_pc::iterator it_node_pc=Node_To_Cells.find(beginid);//Node_To_Cells.begin();
	std::map<int, CPose3D>::iterator it_node_pose=Id_Cells_Pose.find(beginid);//Id_Cells_Pose.begin();
	Area3DDescMap::iterator it_node_feature=Id_Node_Feature.find(beginid);//Id_Node_Feature.begin();

	// 将pose信息写入文件当中
	std::map<int, CPose3D>::iterator _node_pose=Id_Cells_Pose.find(beginid);//Id_Cells_Pose.begin();
	while(_node_pose->first!=endid)
	{
		_node_pose->second.output(this->m_record_trajectory);
		_node_pose++;
	}

	while(it_node_pc->first!=endid)
	{
		if(it_node_pc==Node_To_Cells.end() || it_node_pose==Id_Cells_Pose.end() || it_node_feature==Id_Node_Feature.end())
		{
			cout<<"error in SwitchSession!!"<<endl;
			return;
		}
		// Merge pc
		pcSession->points.insert(pcSession->points.end(),it_node_pc->second->points.begin(),it_node_pc->second->points.end());
		cout<<"pcSession size()="<<pcSession->points.size()<<endl;
		// Merge pose
		NodePose tmpPose;
		double rpy[3],xyz[3];
		tmpPose.id=it_node_pc->first;
		it_node_pose->second.getXYZ(xyz);
		it_node_pose->second.getrpy(rpy);
		tmpPose.pose.x=xyz[0]; tmpPose.pose.y=xyz[1]; tmpPose.pose.z=xyz[2];
		tmpPose.pose.roll=rpy[0]; tmpPose.pose.pitch=rpy[1]; tmpPose.pose.yaw=rpy[2];
		pathList.push_back(tmpPose);
		// Merge descriptor
		descMap.insert(*it_node_feature);
		it_node_pc++;
		it_node_pose++;
		it_node_feature++;
	}

	// Test area map to file
	AreaStoreHdr Area_hdr = {0};
	Area_hdr.id = (unsigned long)m_SessionID;
	Area_hdr.size = pcSession->points.size();
	m_SessionStore.AreaMap2File(pcSession, Area_hdr);

	// Test area descriptor to file		
	Area3DDescStoreHdr Area3DDesc_hdr={0};
	Area3DDesc_hdr.id = (unsigned long)m_SessionID;
	Area3DDesc_hdr.size = descMap.size();
	m_SessionStore.Area3DDesc2File(descMap, Area3DDesc_hdr);

	// Test area robot path to file
	AreaPathStoreHdr Path_hdr={0};
	Path_hdr.id = (unsigned long)m_SessionID;
	Path_hdr.size = pathList.size();
	m_SessionStore.AreaPath2File(pathList, Path_hdr);

	m_SessionID++;

	// display this session!
	boost::shared_ptr<CSession> curSession(new CSession);
	curSession->SetPointCloud(pcSession);
	curSession->SetBoundary(p3DMap->m_abs_lower_x,p3DMap->m_abs_lower_y,p3DMap->m_abs_lower_z,\
		p3DMap->m_abs_upper_x,p3DMap->m_abs_upper_y,p3DMap->m_abs_upper_z);
	curSession->GeneratePlanes3();
	p3DMap->m_Sessions.push_back(curSession);
	cout<<"Render last Session!"<<endl;

	// 2. Register bounding box, check sum, ref-pose 
	
	// 3. Delete these info in CMapBuilder
	// delete Node-info in this session
	Node_pc::iterator it_node_pc_begin=Node_To_Cells.find(beginid);//Node_To_Cells.begin();
	std::map<int, CPose3D>::iterator it_node_pose_begin=Id_Cells_Pose.find(beginid);//Id_Cells_Pose.begin();
	Area3DDescMap::iterator it_node_feature_begin=Id_Node_Feature.find(beginid);//Id_Node_Feature.begin();

	Node_pc::iterator it_node_pc_end=Node_To_Cells.find(endid);//Node_To_Cells.begin();
	std::map<int, CPose3D>::iterator it_node_pose_end=Id_Cells_Pose.find(endid);//Id_Cells_Pose.begin();
	Area3DDescMap::iterator it_node_feature_end=Id_Node_Feature.find(endid);//Id_Node_Feature.begin();

	Node_To_Cells.erase(it_node_pc_begin,it_node_pc_end);
	Id_Cells_Pose.erase(it_node_pose_begin,it_node_pose_end);
	Id_Node_Feature.erase(it_node_feature_begin,it_node_feature_end);

	//// delete ids between beginid and endid
	//vector<int>::iterator it_node_id_begin=m_id_of_node.begin();
	//vector<int>::iterator it_node_id_end=m_id_of_node.begin();
	//std::sort(m_id_of_node.begin(),m_id_of_node.end());
	//for(int i=0;i<m_id_of_node.size();i++)
	//{
	//	if(m_id_of_node[i]<beginid)
	//	{
	//		it_node_id_begin++;
	//	}
	//	if(m_id_of_node[i]<=endid)
	//	{
	//		it_node_id_end++;
	//	}
	//}
	//if(*it_node_id_begin<beginid)
	//	it_node_id_begin++;
	//it_node_id_end--;
	//m_id_of_node.erase(it_node_id_begin,it_node_id_end);

	// 4. Update current bounding box for dynamic whole map

	p3DMap->ResetBound();

	int last_node=-1;
	for(size_t i=0;i<m_id_of_node.size();i++)
	{
		if(i==last_node)
			continue;
		Node_pc::iterator it_node_pc=Node_To_Cells.find(m_id_of_node[i]);
		if(it_node_pc==Node_To_Cells.end())
		{
			cout<<"error in Switch Session! Step 4!"<<endl;
			return;
		}
		boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > &cloud=it_node_pc->second;

		for(size_t j=0;j<cloud->points.size();j++)
		{
			pcl::PointXYZRGB& sp = cloud->points[j];

			LOWER_ASS(p3DMap->m_abs_lower_x,sp.x);
			LOWER_ASS(p3DMap->m_abs_lower_y,sp.y);
			LOWER_ASS(p3DMap->m_abs_lower_z,sp.z);

			UPPER_ASS(p3DMap->m_abs_upper_x,sp.x);
			UPPER_ASS(p3DMap->m_abs_upper_y,sp.y);
			UPPER_ASS(p3DMap->m_abs_upper_z,sp.z);
		}
	}

	m_nfirstLastSession=endid;
	cout<<"switch session end!"<<endl;
}
void CMapBuilder::getdatafrombuf()
{
	CMapBuilder* pMapbuilder = (CMapBuilder*)this;
	// make buff to store 
	//pMapbuilder->Node_To_Cells_Back.insert(make_pair(*(m_id_of_node2.rbegin()),pPc));

	m_num_of_new_nodes=0;
	int last_node=-1;
	Node_pc::iterator it_node = Node_To_Cells_Back.begin();

	// if session has switched, then firstly dump last session into disk
	if((Id_Cells_New_Pose_Back.begin())->first!=m_nfirstLastSession)
	{
		cout<<"session switches at Node: "<<it_node->first<<endl;
		m_bSessionSwitch=true;
		m_nendLastSession=(Id_Cells_New_Pose_Back.begin())->first;
	}

	for(;it_node!=Node_To_Cells_Back.end();it_node++)
	{
		if(it_node->first == last_node)
			continue;
		last_node =it_node->first;
		m_num_of_new_nodes++;	// record num of new nodes to be inserted 
		Node_To_Cells.insert(*it_node);
		Id_Cells_Pose.insert(make_pair(it_node->first,CPose3D()));
	}
	std::map<int, CPose3D>::iterator it_pose = pMapbuilder->Id_Cells_New_Pose_Back.begin();
	for(;it_pose!=Id_Cells_New_Pose_Back.end();it_pose++)
		Id_Cells_New_Pose[it_pose->first] = it_pose->second;
	pMapbuilder->m_id_of_node = m_id_of_node_Back;

	Node_To_Cells_Back.clear();
	Id_Cells_New_Pose_Back.clear();
	m_id_of_node_Back.clear();
}
void CMapBuilder::update1Node(vector<float>& matrix, int id_of_node)
{
	Eigen::Matrix4f transformation;
	int index = 0;
	/*	for(size_t j=0;j<4;j++)
			for(size_t k=0;k<4;k++)
				transformation(j,k)=matrix[index++];*/
	this->Id_Cells_Pose.insert(make_pair(id_of_node,CPose3D(/*transformation*/)));
}
void CMapBuilder::updateAllNodesBack(vector<vector<float> >& m_matrix_of_node,vector<int>& m_id_of_node)
{
	Eigen::Matrix4f transformation;
	for(size_t i=0;i<m_matrix_of_node.size(); i++)
	{
		int index = 0;
		vector<float> &r_matrix = m_matrix_of_node[i];
		for(size_t j=0;j<4;j++)
			for(size_t k=0;k<4;k++)
				transformation(j,k)=r_matrix[index++];
		//output received robotpath at Server_end
		CPose3D pose(transformation);
		//std::cout<<"receive node: "<<m_id_of_node[i]<<std::endl;
		//pose.output(std::cout);
		this->Id_Cells_New_Pose_Back.insert(make_pair(m_id_of_node[i],CPose3D(transformation)));
	}
}
void CMapBuilder::updateAllNodes(vector<vector<float> >& m_matrix_of_node,vector<int>& m_id_of_node)
{
	Eigen::Matrix4f transformation;
	for(size_t i=0;i<m_matrix_of_node.size(); i++)
	{
		int index = 0;
		vector<float> &r_matrix = m_matrix_of_node[i];
		for(size_t j=0;j<4;j++)
			for(size_t k=0;k<4;k++)
				transformation(j,k)=r_matrix[index++];
		//output received robotpath at Server_end
		CPose3D pose(transformation);
		std::cout<<"receive node: "<<m_id_of_node[i]<<std::endl;
		pose.output(std::cout);
		this->Id_Cells_New_Pose.insert(make_pair(m_id_of_node[i],CPose3D(transformation)));
	}
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