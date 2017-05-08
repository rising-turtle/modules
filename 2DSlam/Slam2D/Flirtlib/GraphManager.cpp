#include "GraphManager.h"
#include <sensorstream/CarmenLog.h>
#include <geometry/point.h>
#include <feature/InterestPoint.h>
#include "FlirterNode.h"
#include "ZHPolar_Match.h"

CGraphManager::CGraphManager():m_pPSM(new CPolarMatch("LMS151"))
{}
CGraphManager::~CGraphManager(){}

#define LASER_SICK_NUM		361
#define M_PI 3.141592654
#define D2R(d) (d*M_PI/180.0) 
#define R2D(r) (r*180.0/M_PI)
#define MIN_ANGLE_D 0
#define MAX_ANGLE_D 180
#define MIN_ANGLE_R 0
#define MAX_ANGLE_D M_PI
#define LASER_BEARING_D	0.5
#define LASER_BEARING_R 0.008726646261
#define MAX_LASER_RANGE 50.0

#define DIS_THRESHOLD 1.50	// max distance translation 1.5m


bool CGraphManager::runSicklog()
{
	// firstly we only match two frames
	if(m_log.size()<=0){
		cout<<"log is empty!"<<endl;
		return false;
	}

	for(int i=0;i<m_log.size();i++){
		LaserReading* pcurRead = dynamic_cast<LaserReading*> (m_log[i]);
		// create Flirter Node 
		CFliterNode * pcurNode = new CFliterNode(pcurRead);
		if(pcurNode==NULL || pcurNode->m_featurePoints.size()<=0){
			cout<<"failed to create node at frame: "<<i+1<<endl;
			continue;
		}
		// this is first frame
		if(m_graph.size()==0){
			//pcurNode->m_pose=OrientedPoint2D(0,0,0);
			pcurNode->m_relpose = pcurNode->m_pose;
		}
		else{
			CFliterNode * prefNode = (*m_graph.rbegin()).second;
			OrientedPoint2D transform;
			//if(!pcurNode->matchNodePairGlobal(prefNode,transform))
			if(!pcurNode->matchNodePairLocal(prefNode,transform))
			{
				cout<<"failed to match "<<i+1<<" with "<<i<<endl;
				continue;
			}
		}
		m_graph.insert(make_pair(m_graph.size(),pcurNode));
	}
	return true;
}

bool CGraphManager::readSicklog(string logfile, std::vector<AbstractReading*>& log)
{
	std::ifstream infile(logfile.c_str());
	if(!infile.is_open()){
		cout<<"failed to read file!"<<endl;
		return false;
	}
	char line[8192];
	std::vector<double> phi(LASER_SICK_NUM);
//	std::vector<double> cphi(LASER_SICK_NUM);
//	std::vector<double> sphi(LASER_SICK_NUM);
	for(int i=0;i<LASER_SICK_NUM;i++){
		phi[i]	=	 MIN_ANGLE_R + i*LASER_BEARING_R;
//		cphi[i] = cos(phi[i]);
//		sphi[i] = sin(phi[i]);
	}
	
	std::vector<double> rho(LASER_SICK_NUM);
	std::vector<double> remission;
	OrientedPoint2D laserPose,robotPose;
	while(infile.getline(line,8192))
	{
		strtok(line, " ");
		int N = (int)(atof(strtok(NULL," ")));
		if(N!=LASER_SICK_NUM){
			cout<<"laser number is not right:  file: "<<N<<" Laser: "<<LASER_SICK_NUM<<endl;
			return false;
		}
		for(int i=0;i<N;i++){
			float tmp= atof(strtok(NULL," "));
			if(tmp==0) rho[i] = MAX_LASER_RANGE;
			else rho[i] = tmp;
		}

		LaserReading* frame = new LaserReading(phi,rho);
		
		frame->setMaxRange(MAX_LASER_RANGE);
		//frame->setRemission(remission);
		//frame->setLaserPose(laserPose);
		log.push_back(frame);
	}
	return true;
}

bool CGraphManager::recordGTCarmon(string logfile, string outfile) // 提取文件中GT数据
{
	if(!readlog(logfile)){
		cout<<"failed to read data from log: "<<logfile<<endl;
		return false;
	}
	ofstream trajectory(outfile.c_str());
	if(!trajectory.is_open()){
		cout<<"failed to open file: "<<outfile<<endl;
		return false;
	}
	for(int i=0;i<m_log.size();i++){
		LaserReading* pcur = dynamic_cast<LaserReading*>(m_log[i]);
		trajectory<<pcur->getLaserPose()<<endl;	
	}
	trajectory.close();
	return true;
}

// read log from ground truth
bool CGraphManager::readlog(string logfile){
	std::ifstream infile(logfile.c_str());
	if(!infile.is_open()){
		cout<<"failed to open file: "<<logfile<<endl;
		return false;
	}
	CarmenLogReader reader;
	reader.readLog(infile,m_log);
	return (m_log.size()>0);
}


bool CGraphManager::runlog(int run_num,int start_frame){
	if(m_log.size()<=0){
		cout<<"log is empty!"<<endl;
		return false;
	}
	if(run_num < 0)
		run_num = m_log.size()+1;

	for(int i=0;i+start_frame<m_log.size() && i<run_num;i++){
		LaserReading* pcurRead = dynamic_cast<LaserReading*> (m_log[i+start_frame]);
		// create Flirter Node 
		CFliterNode * pcurNode = new CFliterNode(pcurRead);
		if(pcurNode==NULL || pcurNode->m_featurePoints.size()<=0){
			cout<<"failed to create node at frame: "<<i+1<<endl;
			continue;
		}
		// this is first frame
		if(m_graph.size()==0){
			//pcurNode->m_pose=OrientedPoint2D(0,0,0);
			//pcurNode->m_relpose = pcurNode->m_pose;
			pcurNode->m_relpose = OrientedPoint2D(0,0,0);
		}
		else{
			CFliterNode * prefNode = (*m_graph.rbegin()).second;
			OrientedPoint2D transform;
			//if(!pcurNode->matchNodePairGlobal(prefNode,transform))
			if(!pcurNode->matchNodePairLocal(prefNode,transform))
			{
				cout<<"failed to match "<<i+1<<" with "<<i<<endl;
				//continue;
			}
			if(IsNoisyMove(transform)){
				cout<<"Noisy move! "<<i+1<<" to "<<i<<endl;
				//pcurNode->m_relpose = pcurNode->m_pose - prefNode->m_pose;
				pcurNode->m_relpose = prefNode->m_pose.ominus(pcurNode->m_pose);
				//continue;
			}
		}
		m_graph.insert(make_pair(m_graph.size(),pcurNode));
	}
	return true;
}



// read our carmon log file
bool CGraphManager::readOurlog(string logfile){
	return m_pPSM->readCarmon(logfile,m_pPSM->m_pParam->pm_laser_name);
}
bool CGraphManager::runOurlog(int run_num){
	if(run_num <0)
		run_num = m_pPSM->m_SickScans.size()+1;
	int cnt=0;
	PMScan * ls;
	

	ofstream out_psm("y:\\log\\onlyPSM.log");
	ofstream out_flirt("y:\\log\\onlyflirt.log");

	// PSM + ICP
	while(cnt<m_pPSM->m_SickScans.size() && cnt<run_num){
		ls = m_pPSM->m_SickScans[cnt];
		cnt++;
		CFliterNode * pcurNode = new CFliterNode(ls);
		if(m_graph.size()==0){
			m_graph.insert(make_pair(0,pcurNode));
			continue;
		}
		CFliterNode * prefNode = (*m_graph.rbegin()).second;
		if(!pcurNode->matchNodeFrontend(prefNode)){
			cout<<"PSM+ICP failed!"<<endl;
			delete pcurNode;
			continue;
		}
		cout<<"PSM successful to match "<<cnt<<" with "<<cnt-1<<endl;
		pcurNode->m_pose = prefNode->m_pose.oplus(pcurNode->m_relpose);
		out_psm<<pcurNode->m_pose<<endl;
		m_graph.insert(make_pair(m_graph.size(),pcurNode));
	}
	
	// FLIRT 
	cnt=0;
	map<int,CFliterNode*>::iterator it = m_graph.begin();
	map<int,CFliterNode*>::iterator it_last=it;
	it++;
	while(it!=m_graph.end()){
		CFliterNode* prefNode = it_last->second;
		CFliterNode* pcurNode = it->second;
		cnt++;
		pcurNode->m_pose = OrientedPoint2D(0,0,0);
		OrientedPoint2D transform;
		// FLIRT 
		//if(!pcurNode->matchNodePairGlobal(prefNode,transform))
		if(!pcurNode->matchNodePairLocal(prefNode,transform))
		{
			cout<<"failed to match "<<cnt<<" with "<<cnt-1<<endl;
			
		}else{
			pcurNode->m_pose = prefNode->m_pose.oplus(transform);
			out_flirt<<pcurNode->m_pose<<endl;
			cout<<"FLIRT successful to match "<<cnt<<" with "<<cnt-1<<endl;
		}
		it_last = it;
		it++;
	}
	
	out_psm.close();
	out_flirt.close();
	return true;
}

void CGraphManager::recordTrajectory(string outfile)
{
	ofstream trajectory(outfile.c_str());
	if(trajectory.is_open()){
		map<int, CFliterNode*>::iterator it = m_graph.begin();
		int index=0;
		bool firstPose=true;
		OrientedPoint2D lastpose,curpose;
		OrientedPoint2D original;
		while(it!=m_graph.end()){
			if(firstPose){
				//curpose = it->second->m_pose;
				original = it->second->m_pose;
				curpose = it->second->m_pose;// - original;
				firstPose = false;
			}
			else{
					curpose = lastpose.oplus((it->second->m_relpose));
			}
			// record also relative pose
			trajectory/*<<original.ominus(it->second->m_pose)<<" "*/<<curpose<<endl;
			//lastpose = it->second->m_pose - original;
			lastpose=curpose;
			it++;	
			index++;
		}
		trajectory.close();
	}
}


bool CGraphManager::IsNoisyMove(OrientedPoint2D& transform)
{
	if(fabs(transform.x) >= DIS_THRESHOLD || \
		fabs(transform.y) >= DIS_THRESHOLD )
		return true;
	return false;
}

namespace{
int findSimilarElement(const Point2D& keyp, vector<Point2D>& pset)
{
	double min_dis = 1e17;
	double dis;
	int ret = -1;
	for(int i=0;i<pset.size();i++)
	{
		dis = fabs(keyp.x - pset[i].x) + fabs(keyp.y-pset[i].y);
		if(dis<min_dis){
			min_dis = dis;
			ret = i;
		}
	}
	return ret;
}
};

void CGraphManager::testMultiFlirt(int run_num)
{
	if(run_num<0)
		run_num = m_pPSM->m_SickScans.size();
	int cnt=0;
	PMScan* ls; 

	ofstream outf("y:\\log\\exp1.log");
	
	// using ICP or PSM set position
	while(cnt<m_pPSM->m_SickScans.size() && cnt<run_num)
	{
		ls = m_pPSM->m_SickScans[cnt];
		cnt++;

		CFliterNode* pcurNode = new CFliterNode(ls);
		if(m_graph.size() == 0)
		{
			m_graph.insert(make_pair(0,pcurNode));
			continue;
		}
		CFliterNode* prefNode = m_graph.rbegin()->second;
		try{
			pcurNode->matchNodeFrontend(prefNode);
			pcurNode->m_pose = prefNode->m_pose.oplus(pcurNode->m_relpose);
		}catch(int err){
			cout<<"failed in Frontend Match!"<<endl;
			continue;
		}
		m_graph.insert(make_pair(cnt,pcurNode));
	}

	map<int,CFliterNode*>::iterator it = m_graph.begin();
	while(it!=m_graph.end()){
		outf<<it->second->m_pose<<endl;
		it++;
	}
	// for the last pose
	ofstream lastpose("y:\\log\\lastpose.log");

	CFliterNode* plastNode = m_graph.rbegin()->second;
	it = m_graph.begin();
	it++;
	vector<OrientedPoint2D> poseSet;
	poseSet.push_back(plastNode->m_pose);

	while(it!=m_graph.end()){
		OrientedPoint2D trans;
		if(plastNode->matchNodePairLocal(it->second,trans)){
			cout<<"successful to match with "<<it->first<<" Node!"<<endl;
			poseSet.push_back(it->second->m_pose.oplus(trans));
		}
		it++;
	}
	for(int i=0;i<poseSet.size();i++)
		lastpose<<poseSet[i]<<" "<<i<<endl;
}

void CGraphManager::testRelTrans()
{
	if(m_log.size()<=0){
		cout<<"log is empty!"<<endl;
		return ;
	}

	for(int i=0;i</*m_log.size()*/100;i++){
		LaserReading* pcurRead = dynamic_cast<LaserReading*> (m_log[i]);
		// create Flirter Node 
		CFliterNode * pcurNode = new CFliterNode(pcurRead);
		if(pcurNode==NULL || pcurNode->m_featurePoints.size()<=0){
			cout<<"failed to create node at frame: "<<i+1<<endl;
			continue;
		}
		// this is first frame
		if(m_graph.size()==0){
			//pcurNode->m_pose=OrientedPoint2D(0,0,0);
			pcurNode->m_relpose = pcurNode->m_pose;
		}
		else{
			CFliterNode * prefNode = (*m_graph.rbegin()).second;
			OrientedPoint2D transform;
			if(!pcurNode->matchNodePairGlobal(prefNode,transform))
			//if(!pcurNode->matchNodePairLocal(prefNode,transform))
			{
				cout<<"failed to match "<<i+1<<" with "<<i<<endl;
				continue;
			}
			//if(IsNoisyMove(transform)){
			//cout<<"Noisy move! "<<i+1<<" to "<<i<<endl;
			//pcurNode->m_relpose = pcurNode->m_pose - prefNode->m_pose;
			////continue;
			//}
		}
		m_graph.insert(make_pair(m_graph.size(),pcurNode));
	}

	ofstream trajectory("d:\\exprdata\\glbtrajectory1.log");
	if(trajectory.is_open()){
		map<int, CFliterNode*>::iterator it = m_graph.begin();
		bool firstPose=true;
		OrientedPoint2D lastpose,curpose;
		while(it!=m_graph.end()){
			if(firstPose){
				curpose = it->second->m_pose;
				firstPose = false;
			}
			else
				//curpose = lastpose.oplus(it->second->m_relpose);
				curpose = lastpose + it->second->m_relpose;

			// record also relative pose
			trajectory<<it->second->m_pose<<" "<<curpose<<endl;
			//lastpose = curpose;
			lastpose=it->second->m_pose;
			it++;		
		}
		trajectory.close();
	}

	return ;
}

void CGraphManager::testPointTransform()
{
	if(m_log.size()<=0)
		return ;
	LaserReading* pread = dynamic_cast<LaserReading*>(m_log[0]);
	CFliterNode* node=new CFliterNode(pread);
	
	std::vector<Point2D> world = pread->getWorldCartesian();
	std::vector<Point2D> local = pread->getCartesian();

	int lindex,gindex;
	for(int i=0;i<node->m_featurePointsLocal.size();i++){
		const Point2D& lp = node->m_featurePointsLocal[i]->getPosition();
		const Point2D& gp = node->m_featurePoints[i]->getPosition();
		lindex = findSimilarElement(lp,local);
		gindex = findSimilarElement(gp,world);
		cout<<"lp:("<<lp.x<<","<<lp.y<<"),"<< "local: ("<<local[lindex].x<<","<<local[lindex].y<<")"<<endl;
		cout<<"gp:("<<gp.x<<","<<gp.y<<"),"<< "world: ("<<world[gindex].x<<","<<world[gindex].y<<")"<<endl;
	}

	delete node;
}

void CGraphManager::testLocalGlobalTransform()
{
	int N=100;
	if(m_log.size()<N)
		N = m_log.size();
	OrientedPoint2D transforml, transformg;
	vector<CFliterNode*> node_set(N);
	for(int i=0;i<N;i++)
	{
		LaserReading* pread = dynamic_cast<LaserReading*>(m_log[i]);
		node_set[i]=new CFliterNode(pread);
	}
	
	bool first=true;
	CFliterNode* pre;
	CFliterNode* cur;
	for(int i=0;i<node_set.size();i++){
		cur=node_set[i];
		if(first){
			first = false;
		}
		else
		{
			cur->matchNodePairLocal(pre,transforml);
			cur->matchNodePairGlobal(pre,transformg);
		}
		pre = cur;
		cout<<"local: "<<pre->m_pose.oplus(transforml)<<endl;
		cout<<"global: "<<pre->m_pose+transformg<<endl;
	}

	for(int i=0;i<node_set.size();i++){
		delete node_set[i];
		node_set[i]=NULL;
	}
}