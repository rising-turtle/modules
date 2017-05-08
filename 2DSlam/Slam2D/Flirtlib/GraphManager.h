#ifndef GRAPH_MANAGER_H
#define GRAPH_MANAGER_H

#include "preheader.h"
#include "FlirterNode.h"

class AbstractReading;
class CPolarMatch;

// contains all the pose-node in a graph , 
// optimize this graph using Hogman or g2o
class CGraphManager{
public:
	CGraphManager();
	~CGraphManager();

	bool recordGTCarmon(string logfile, string outfile); // 提取文件中GT数据

	// read Sick laser data
	bool readSicklog(string logfile,std::vector<AbstractReading*>& log);
	bool runSicklog();

	// read log file using groundtruth
	bool readlog(string logfile);
	bool runlog(int run_num=-1,int start_frame=0);

	// read our carmon log file
	bool readOurlog(string logfile);
	bool runOurlog(int run_num=-1);

	// record trajectory
	void recordTrajectory(string outfile);
	bool IsNoisyMove(OrientedPoint2D& transform);

public:
	CPolarMatch* m_pPSM; // using PSM to frontend match
public:
	std::vector<AbstractReading*> m_log;	// read log files
	map<int, CFliterNode*> m_graph; // contains the trajectory of robot
	vector<bool> m_noisy;	// whether this match is success
public:
	// those are for debug
	void testRelTrans();
	void testLocalGlobalTransform();
	void testPointTransform();

	void testMultiFlirt(int run_num=10); // experiment 1
};




#endif