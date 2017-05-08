#pragma  once
//#include "Point_Cloud.h"
//#include <boost/shared_ptr.hpp>
#include "InternalDefine.h"
#include <vector>
#include "AreaStore.h"
#include "CSession.h"
//#include "pcl/registration/ia_ransac.h"
//#include "3DMap.h"

class CPose3D;
class CMapBuilder
{
public:

	CMapBuilder();
	~CMapBuilder();

	IoTRobot_MapBuilderMSG stMapBuilderMSG;
	void NewCommand(const IoTRobot_Message MSG);



	ClassPtrs m_stClassPtrs;
	static unsigned char *m_pucGetImgPtr;
	void GetRGB24Data(unsigned char *pucRGB24);

	//CPoint_Cloud m_cPointColud; 
	void MergePointCloud2Area(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > &cloud, int id_of_node = 0);
	int NewGlobalPCIn(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > &cloud, int id_of_node = 0);
	int TMat_RAngle(float *pfTMat,float *pfRAngle);
	//static C3DMap m_c3Dmap;

	//////////////////////////////////////////////////////////////////////////
	// added at 2011/12/13 by ZH 
	// for dynamic display points observed by moving robot
	// Main 
	int m_offset_x;		// Offset along x-axis
	int m_offset_y;		// Offset along y-axis
	int m_offset_z;		// Offset along z-axis

	double m_upper_x,m_lower_x;  // Bounder along x-axis
	double m_upper_y,m_lower_y;	// Bounder along y-axis
	double m_upper_z,m_lower_z;	// Bounder along z-axis

	void calcoffset();		// Calculate offset based on current Basic Point
	void calcbounder();		// Calculate bounder box 
	boost::shared_ptr<CPose3D> m_pBasicPose;	// Basic Position of Area
	bool IsexceedBounder(CPose3D& ); // Whether robot almost exceed Boundary
	void dumpAreatoDisk();	//TODO // Dump Area into disk
	void TranslateArea(CPose3D& );

	// Index from (x,y,z)
	inline int getIndexCell(float& x,float& y, float& z){
		if(_isnan(x) || _isnan(y) || _isnan(z))
			return -1;
		if(x>=m_upper_x || y>=m_upper_y || z>=m_upper_z \
			|| x<m_lower_x || y<m_lower_y || z<m_lower_z )
			return -1;
		int lx = floor(( x*100 + S_RX)+0.5);
		lx>>=CMapBuilder::m_iCellSize;//divide by cell_size
		int ly = floor(( y*100 + S_RY)+0.5); 
		ly>>=CMapBuilder::m_iCellSize;//divide by cell_size
		int lz = floor(( z*100 + S_RZ)+0.5); 
		lz>>=CMapBuilder::m_iCellSize;//divide by cell_size

		// Calc offset
		lx-=m_offset_x;
		ly-=m_offset_y;
		lz-=m_offset_z;

		if(lx >= X_CELL || ly>= Y_CELL || lz >= Z_CELL \
			|| lx<0 || ly<0 || lz<0)
		{
			return -1;
		}
		return (lx*X_STEP + ly*Y_STEP + lz);
	}
	
	// added at 2011/12/13 by ZH 
	//////////////////////////////////////////////////////////////////////////
	
	bool IsNoisePoint(int index,boost::shared_ptr<pcl::PointXYZRGB>& p);

	//// Added at 2011/11/30 by ZH
	// Connects Nodes with Cells
	typedef map<int,pcl::PointCloud<pcl::PointXYZRGB>::Ptr > Node_pc;
	Node_pc Node_To_Cells;
	void transformPointsofNode(int id_of_node, CPose3D& pose);
	void transformAllNodes(std::vector<int>& , std::vector<CPose3D>&);
	void cloudRegistration(boost::shared_ptr<CPose3D>& pose, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud);
	void MergeLMapwithGMap(pcl::PointCloud<pcl::PointXYZRGB> * global_map,pcl::PointCloud<pcl::PointXYZRGB> * local_map);
	void updateAllNodes(vector<vector<float> >&, vector<int>& id_of_node);
	void updateAllNodesBack(vector<vector<float> >& ,vector<int>& m_id_of_node);
	void fromNodestoCell();
	void fromNodestoDynamicCell(); // to dynamically display the whole area
	void findMaxAndMin(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> >& cloud,float& l_x,float& u_x,float& l_y,float& u_y,float& l_z,float& u_z);
	void update1Node(vector<float>& matrix, int id_of_node);
	void getdatafrombuf();
	// buf to store client's write
	Node_pc Node_To_Cells_Back;
	std::map<int, CPose3D> Id_Cells_New_Pose_Back;
	vector<int> m_id_of_node_Back;

	typedef bitset<ALL_CELLS> Cells_id;					// index of cell number
	typedef struct Cell_Element{
		int index_of_cell;
		boost::shared_ptr<pcl::PointXYZRGB> _p;
		struct Cell_Element():_p(new pcl::PointXYZRGB){};
		struct Cell_Element(int index,boost::shared_ptr<pcl::PointXYZRGB>& p):index_of_cell(index),_p(p){};
	}cell_info;
	//typedef	std::map<int,pcl::PointXYZRGB> Cells_set;	// Cells set from a Point Cloud
	typedef vector<cell_info> Cells_set;
	std::map<int, Cells_set> Id_Cells_Set;				// From Id of Node to Cells Set
	std::map<int, Cells_id> Id_Cells_Id;				// From Id of Node to Cells Id
	std::map<int, CPose3D> Id_Cells_Pose;				// From Id of Node to Cells Pose
	std::map<int, CPose3D> Id_Cells_New_Pose;			// 
	Area3DDescMap Id_Node_Feature;			// Record Features of each Node
	int id_of_cur_node;
	vector<int> m_id_of_node;

	// save features of each node
	Node3DDesc ** m_pFeaturesNode;

	// transform Cells according to responding Node
	void tarnsformofCells(int id_of_Node, CPose3D& pose);
	// whether to refresh all cell-map
	bool refresh_cells;

	int m_num_of_new_nodes; // record number of new nodes insert into Cell

	CAreaStore m_SessionStore;
	bool m_bSessionSwitch;
	int m_SessionID;
	// record last number of first node in last session
	void SwitchSession(int beginid,int endid);
	void TransfromSessionNodes2Cell(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > &,\
		boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > &);
	bool m_bLoadSessions;	// Load Sessions from disk? 
	void LoadAllSessions(string,vector<boost::shared_ptr<CSession> >& );
	int m_nfirstLastSession;
	int m_nendLastSession;

	// it will change with CellSize when Dynamic cell_size change
	static int m_iCellSize;
	//// \\Added at 2011/11/30 by ZH
};