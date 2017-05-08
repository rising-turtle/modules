#include "Point_Cloud.h"

#include <boost/shared_ptr.hpp>
//#include <opencv2/core/core.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Core>
#include "InternalDefine.h"


#include "3DMap.h"
class CMapBuilder
{
public:

	CMapBuilder();
	~CMapBuilder();

	ClassPtrs m_stClassPtrs;
	static unsigned char *m_pucGetImgPtr;
	void GetRGB24Data(unsigned char *pucRGB24);

	CPoint_Cloud m_cPointColud; 
	void MergePointCloud2Area(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > &cloud);
	int NewGlobalPCIn(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > &cloud);
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

};