#include "PCL_Openni.h"
#include "IoTRobot_RGBDSLAM_Interface.h"
#include "CPose3D.h"

#include "ReadGroundTrues.h"

typedef void (*SLAMClientCallBack_OK)(unsigned char*pucData,int nDataLen,void *pContext);
typedef void (*SLAMClientCallBack_RGB24_Depth)(unsigned char*pucData,unsigned short *pusDepth,void *pContext);


#define LOG_ARRAY_LEN 20

#define RX 1200 // X [-6 6]
#define RY 400	// Y [-2 2]
#define RZ 1200 // Z [-6 6]

#define CELLSIZE 4
#define X_CELL RX/CELLSIZE // 400
#define Y_CELL RY/CELLSIZE // 120
#define Z_CELL RZ/CELLSIZE // 400

#define X_STEP Y_CELL*Z_CELL // 120*400
#define Y_STEP Z_CELL // 400

#define ALL_CELLS X_CELL*Y_CELL*Z_CELL // 



typedef struct _XYZPt{
	float x; 
	float y; 
	float z;
} XYZPt;
// Descriptor is CV::MAT(N,64,float), N depends on the number of features.(We use ONLY non-extended surf descriptor)
// 3D Descriptor is the combination of 3D pose and descrpitor of features.
// The serialization of 3D Descriptor is as follows:
// (x0,y0,z0),(0,0,float),(0,1,float),...,(0,63,float); (x1,y1,z1),(1,0,float),(1,1,float),...,(1,63,float);... ;(xN-1,yN-1,zN-1),(N-1,0,float),(N-1,1,float),...,(N-1,63,float);
typedef struct _Feature3DDesc{
	XYZPt xyz;
	float desc[64];
} Feature3DDesc;

typedef struct _Node3DDesc{
	int id; // node ID
	int n; // the number of features
	Feature3DDesc desc3DList[0];
} Node3DDesc;

class IoTRobot_RGBDSLAM_Client
{
public:
	IoTRobot_RGBDSLAM_Client();
	~IoTRobot_RGBDSLAM_Client();

	int SLAMInit();
	int SLAMRun();
	int SLAMStop();
	int SLAMUnint();


	int SLAMParamsSetting(void *pParam);
	int resetSession(CPose3D pose);


	SLAMClientCallBack_OK m_cbSLAMOK;
	void *m_pSLAMOKContext;
	SLAMClientCallBack_RGB24_Depth m_cbRGBDData;
	void *m_pRGBDDataContext;

	double m_dCurPos[6];
protected:
private:
	
	LPDWORD ID_SLAM;
	HANDLE m_hSLAM;
	LPDWORD ID_PCData;
	HANDLE m_hPCData;
	static UINT ThreadSLAM(LPDWORD lpParam);
	static UINT ThreadGetPCData(LPDWORD lpParam);


	IoTRobot_RGBDSLAM_Interfaace m_CRGBDSLAM; 
	ReadGroundTrues m_CReadGroundTrues;  // 读取 groundtruth 的点云数据

	SimpleOpenNIViewer m_COpenNIViewer;
	unsigned char *m_pucRGB;
	unsigned short *m_pusDepth;

	bool m_rebuild_cells;

	DWORD m_ulRunTimeBuff[LOG_ARRAY_LEN][8];
	bool m_bStopSLAM;
	bool m_bStopSLAMClient;

	boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > m_PC;
	boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > m_subPC;

	bool m_bNewSyncDataArrive;
	bitset<ALL_CELLS> m_valid_flag;
	bool IsNoisePoint(int index,boost::shared_ptr<pcl::PointXYZRGB>& p);
	int getIndexCell(float& x,float& y, float& z);
	void MergePointCloud2Area
		(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > &cloud, int id_of_node,unsigned char *pucDataHead,int *pnPCLen);

	void getImagesandDepthMetaData(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> >& point_cloud,
		unsigned char* rgbbuf,
		unsigned short* depthbuf);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_global_cell_map; // to display cells
	std::vector<boost::shared_ptr<pcl::PointXYZRGB> > m_global_cells; // all the discrete cells 5*5*5
	std::vector<int> m_id_of_node;					// id of node
	std::vector<Eigen::Matrix4f> m_matrix_of_node;				// pose of node

	vector<CPose3D> m_robotpath_update;
	vector<int> m_id_of_node2;
	typedef map<int,pcl::PointCloud<pcl::PointXYZRGB>::Ptr > Node_pc;
	Node_pc Node_To_Cells;


	vector<vector<float> >m_matrix_of_nodeF;

	unsigned char *m_pucSLAMData;
	int m_nSLAMDataLen;
	map<string,string> m_SLAMParams;

	Node3DDesc** m_pNodeFeatures;

	// Reset session
	bool m_bResetSession;
	CPose3D m_SessionPose;
	
	//?
	unsigned char *m_pucSparsePC;
	int m_nSparsePCDataLen;
};