#pragma  once
//#include "Point_Cloud.h"
//#include <boost/shared_ptr.hpp>
#include "InternalDefine.h"
#include <vector>

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


	inline int getIndexCell(float& x,float& y, float& z)
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
	int id_of_cur_node;
	vector<int> m_id_of_node;

	// transform Cells according to responding Node
	void tarnsformofCells(int id_of_Node, CPose3D& pose);
	// whether to refresh all cell-map
	bool refresh_cells;

	//// \\Added at 2011/11/30 by ZH

};