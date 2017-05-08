#ifndef _CPOINTS_SET
#define _CPOINTS_SET

#include "..\KDTreeCapable.h"
#include <vector>
#include <deque>
#include <limits>
#include <cmath>
#include <stdlib.h>
#include <ctime>

using namespace std;

static const double pi = 3.141592654;
static const double FMIN = 1e-10;

class CPose3D{
public:
	CPose3D();
	CPose3D::CPose3D(const double x,const double y,const double z, const double yaw=0, const double pitch=0, const double roll=0);
	CPose3D(CPose3D& );
	CPose3D&  operator= (CPose3D& );
	~CPose3D(){};
	
	/*Calculate coordinates after CPose3D translation*/
	void composePoint(double lx,double ly,double lz, float &gx, float &gy, float &gz ) ; //using Eular angles to calculate translation
	void composePointQ(double lx,double ly,double lz, float &gx, float &gy, float &gz);//using Quaternion to calculate translation
	inline float square(float x){return (x*x);}
	inline float distance3DTo(float _x,float _y,float _z){return (float)(sqrt(square(m_coords[0]-_x)+square(m_coords[1]-_y)+square(m_coords[0]-_z)));}
	double m_coords[3];
	double x(){return m_coords[0];}
	double y(){return m_coords[1];}
	double z(){return m_coords[2];}
	double yaw(){return m_yaw;}
	double roll(){return m_roll;}
	double pitch(){return m_pitch;}
public:
	double m_Rot[3][3];//Rotation Matrix
	mutable double m_yaw,m_roll,m_pitch; // Eular angles
	double _q[4];//Quaternion 
	bool m_ypr_uptodate; 

	inline double wrapToPi(double deg){return ((deg*pi)/180.0);}
	void rebuildRotationMatrix();
	void CalculateQ();//Calculate Q[4];
	void CalculateEularFromQ(); // Calculate Eular angles from Q[4]
	void setFromValues(const double	x0,const double	y0,const double	z0,
										const double yaw= 0,	const double pitch=0,const double roll=0);
};

class CPoint3D{
public:
	CPoint3D():x(0),y(0),z(0){}
	CPoint3D(CPoint3D& o){this->x = o.get_x();this->y = o.get_y();this->z = o.get_z();}
	CPoint3D(const CPose3D& o){this->x = (float)o.m_coords[0];this->y = (float)o.m_coords[1];this->z =(float)o.m_coords[2];}
	CPoint3D& operator=(CPoint3D& o){this->x = o.get_x();this->y = o.get_y();this->z = o.get_z();}
	CPoint3D& operator*=(double d){this->x*=(float)d; this->y*=(float)d;this->z*=(float)d; return (*this);}
	~CPoint3D(){}
	inline float square(float x){return x*x;}
	inline double	distance3DTo(float _x,float _y,float _z){return sqrt(square(x-_x)+square(y-_y)+square(z-_z));}

	inline void x_incr(float dx){this->x+=dx;}
	inline void y_incr(float dy){this->y+=dy;}
	inline void z_incr(float dz){this->z+=dz;}
	inline float get_x(){return this->x;}
	inline float get_y(){return this->y;}
	inline float get_z(){return this->z;}
	inline void set_x(float tx){this->x = tx;}
	inline void set_y(float ty){this->y = ty;}
	inline void set_z(float tz){this->z = tz;}
public:
	float x,y,z;
};



	/** A structure for holding correspondences between two sets of points or points-like entities in 2D or 3D.
		  */
		struct  TMatchingPair
		{
			TMatchingPair() :
				this_idx(0), other_idx(0),
				this_x(0),this_y(0),this_z(0),
				other_x(0),other_y(0),other_z(0),
				errorSquareAfterTransformation(0)
			{
			}

			TMatchingPair( unsigned int _this_idx,unsigned int _other_idx, float _this_x, float _this_y,float _this_z, float _other_x,float _other_y,float _other_z ) :
					this_idx(_this_idx), other_idx(_other_idx),
					this_x(_this_x),this_y(_this_y),this_z(_this_z),
					other_x(_other_x),other_y(_other_y),other_z(_other_z),
					errorSquareAfterTransformation(0)
			{
			}

			unsigned int	this_idx;
			unsigned int	other_idx;
			float			this_x,this_y,this_z;
			float			other_x,other_y,other_z;
			float			errorSquareAfterTransformation;

		};

class CPointsSet : public KDTreeCapable{
public:
	CPointsSet(size_t n=4000,double x=2,double y=10,double z=-5,double yaw=75,double roll=40, double pitch=37);
	~CPointsSet();
			/** Auxiliary variables used in "getLargestDistanceFromOrigin"
		   * \sa getLargestDistanceFromOrigin
		   */
		 mutable float	m_largestDistanceFromOrigin;

		 /** Auxiliary variables used in "getLargestDistanceFromOrigin"
		   * \sa getLargestDistanceFromOrigin
		   */
		 mutable bool	m_largestDistanceFromOriginIsUpdated;

		 void mark_as_modified() const; //!< Called only by this class or children classes, set m_largestDistanceFromOriginIsUpdated=false and such.

		/** @name Virtual methods that MUST be implemented by children classes of KDTreeCapable
			@{ */
		/** Must return the number of data points */
		 virtual size_t kdtree_get_point_count() const{return this->size();};

		/** Must fill out the data points in "data", such as the i'th point will be stored in (data[i][0],...,data[i][nDims-1]). */
		virtual void kdtree_fill_point_data(ANNpointArray &data, const int nDims) const;

		size_t size() const{return x.size();}

	/*Dump all the points into files*/
	void DumpTofile(const char*);

	/*Calculate the nearest points_set */
	/*standard*/
	void  computeMatchingWith3D(
    CPose3D	 &otherMapPose,
    float maxDistForCorrespondence, float maxAngularDistForCorrespondence,
     CPoint3D &angularDistPivotPoint, std::deque<TMatchingPair>& correspondences,
    float &correspondencesRatio, float *sumSqrDist);
	/*Gravity*/
    void computeMatchingWith3Dgravity(
    CPose3D	 &otherMapPose,
    float maxDistForCorrespondence, float maxAngularDistForCorrespondence,
     CPoint3D &angularDistPivotPoint, std::deque<TMatchingPair>& correspondences,
    float &correspondencesRatio, float *sumSqrDist);

	inline float square(float x){return (x*x);}

	public:
		class gravity_point{
		public:
		gravity_point(float x,float y, float z):gx(x),gy(y),gz(z){}	
		~gravity_point(){}
		public:
			float gx,gy,gz;
			std::vector<float> mx,my,mz;
			public:
			   inline void insert(float x,float y, float z){
					mx.push_back(x);
					my.push_back(y);
					mz.push_back(z);
				}
			 	void calculate(float *out_x,float *out_y,float *out_z){
					float sx,sy,sz;
					sx=sy=sz = 0.0;
					size_t cnt = mx.size();
					for(size_t i=0;i<cnt;i++)
						{
						sx+=mx[i];
						sy+=my[i];
						sz+=mz[i];
					}
					*out_x = sx/(float)cnt;
					*out_y = sy/(float)cnt;
					*out_z = sz/(float)cnt;
				}
				 bool hasPoint(float x,float y, float z)
				{
					if((fabs(x-gx)<=FMIN) && (fabs(y-gy)<=FMIN) &&(fabs(z-gz)<=FMIN))
						return true;
					return false;
				}
		};

		 int HasPoint(	std::vector<gravity_point>& ga,float x, float y, float z) const // check whether the gravity point has already been included
			{
			if(ga.size()==0) return -1;
			for(unsigned int i =0;i<ga.size();i++)
				if(ga[i].hasPoint(x,y,z))
					return i;
			return -1;
		}

private:
	vector<float> x,y,z; // original points set
	vector<float> tx,ty,tz; // translated points set after rotation and displacement
	size_t _N; 

		/*Randomly construct  the points*/
		void RandomGenerate(size_t num);

		/*Spatial translation according to <dx,dy,dz,raw,roll,>*/
		void TranslateOriginalPoints();
	explicit CPointsSet(CPointsSet& o);
	CPointsSet& operator= (const CPointsSet&);
public:
	/*CPose3D used to calculate translation of points*/
		CPose3D _pose;
};
	
	
#endif