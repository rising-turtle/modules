#ifndef CPOSE3D_H
#define CPOSE3D_H
#include <Eigen/Core>
#include <cmath>
#include <iostream>

#define pi 3.141592654
#define R2D(r) ((r)/pi*180)
#define equal_thresh 1e-5

using namespace std;


class CPoint3D{
public:
	CPoint3D(){x=0;y=0;z=0;}
	~CPoint3D(){}

	inline void x_incr(double _x){ x+=_x;}
	inline void y_incr(double _y){ y+=_y;}
	inline void z_incr(double _z){ z+=_z;}

	void operator*=(double factor){
		x*=factor; y*=factor; z*=factor;
		return ;
	}

	double x,y,z;
};



class CPose3D{
public:
	CPose3D():yaw(0),pitch(0),roll(0){m_coords[0]=m_coords[1]=m_coords[2]=0;rebuildRotationMatrix();}
	//CPose3D(const double x,const double  y,const double  z,const double  yaw=0, const double  pitch=0, const double roll=0);
	CPose3D(Eigen::Matrix4f &transformation ){
		UpdatePosewithMatrix4f(transformation);
	}
	CPose3D(double _yaw,double _pitch,double _roll,double x,double y, double z):yaw(_yaw),pitch(_pitch),roll(_roll)
	{
		m_coords[0]=x;m_coords[1]=y;m_coords[2]=z;
		rebuildRotationMatrix();
	}
	CPose3D(double _q[4]){m_coords[0]=m_coords[1]=m_coords[2]=0;
	for(size_t i=0;i<4;i++) q[i] = _q[i];
		rotationMatrixNoResize();
		getYawPitchRoll(yaw,pitch,roll);		
	}
	CPose3D(const CPose3D& _pose){
		m_coords[0]= _pose.m_coords[0];m_coords[1]=_pose.m_coords[1];m_coords[2] = _pose.m_coords[2];
		yaw = _pose.yaw; roll = _pose.roll; pitch = _pose.pitch;
		rebuildRotationMatrix();
	}
	~CPose3D(){}

	CPose3D operator+(const CPose3D& ); // 
	CPose3D operator+=(const CPose3D& ); // 
	CPose3D& operator = (const CPose3D&);
	inline bool operator ==(const CPose3D& pose){
		if( fabs(m_coords[0] - pose.m_coords[0]) < equal_thresh && fabs(m_coords[1] - pose.m_coords[1]) < equal_thresh && fabs(m_coords[2] - pose.m_coords[2]) <equal_thresh \
			&& fabs(yaw-pose.yaw) < equal_thresh && fabs(roll - pose.roll) < equal_thresh && fabs(pitch - pose.pitch) < equal_thresh)
			return true;
		return false;
	}

	inline void getXYZ(double xyz[3]){xyz[0] = m_coords[0];xyz[1]=m_coords[1];xyz[2]=m_coords[2];}
	inline void getrpy(double rpy[3]){rpy[0] = roll; rpy[1] = pitch; rpy[2] = yaw;}

	ostream& output(ostream& out)
	{
		out<<"(x,y,z)" <<"("<<m_coords[0]<<","<<m_coords[1]<<","<<m_coords[2]<<")" <<endl;
		out<<"(r,p,y)" <<"("<<R2D(roll)<<","<<R2D(pitch)<<","<<R2D(roll)<<")"<<endl;
		return out;
	}

	/*6dof (x,y,z,yaw,roll,pitch)*/
	double m_coords[3]; // location
	double yaw,roll,pitch; // posture
	double q[4]; //quaternion 
 
	Eigen::Matrix3f m_ROT; // Rotation Matrix

public:

	/*Calculate posture angles from Rotation matrix*/
	void	getYawPitchRoll( double &yaw, double &pitch, double &roll );

	/** Rebuild the homog matrix from the angles. */
	void  rebuildRotationMatrix();

	/**  Makes "this = A (+) B"; this method is slightly more efficient than "this= A + B;" since it avoids the temporary object.
	*  \note A or B can be "this" without problems.
	*/
	void composeFrom(const CPose3D& A, const CPose3D& B );

	inline void  getHomogeneousMatrix(Eigen::Matrix4f & out_HM ) const
	{
		for(size_t i=0;i<3; i++)
			for(size_t j=0;j<3;j++)
				out_HM(i,j) = m_ROT(i,j);
		for (int i=0;i<3;i++) out_HM(i,3)=m_coords[i];
		out_HM(3,0)=out_HM(3,1)=out_HM(3,2)=0.; out_HM(3,3)=1.;
	}
	//Update CPose3D using Matrix4f
	inline void UpdatePosewithMatrix4f(Eigen::Matrix4f &transformation){
		m_ROT = transformation.block<3,3>(0, 0);
		Eigen::Vector3f translation = transformation.block<3,1>(0, 3);
		m_coords[0] = translation(0); m_coords[1] = translation(1); m_coords[2] = translation(2);
		getYawPitchRoll(yaw,pitch,roll);
	}

	// rotate point
	void rotatePoint( double lx, double ly, double lz, double &gx,double &gy,double &gz ){
		const double t2 = q[0]*q[1]; const double t3 = q[0]*q[2]; const double t4 = q[0]*q[3]; const double t5 =-q[1]*q[1]; const double t6 = q[1]*q[2];
		const double t7 = q[1]*q[3]; const double t8 =-q[2]*q[2]; const double t9 = q[2]*q[3]; const double t10=-q[3]*q[3];
		gx = 2*((t8+ t10)*lx+(t6 - t4)*ly+(t3+t7)*lz)+lx;
		gy = 2*((t4+  t6)*lx+(t5 +t10)*ly+(t9-t2)*lz)+ly;
		gz = 2*((t7-  t3)*lx+(t2 + t9)*ly+(t5+t8)*lz)+lz;
	}

	// build rotation matrix with q[4]
	inline void  rotationMatrixNoResize() 
	{
		m_ROT(0,0)=q[0]*q[0]+q[1]*q[1]-q[2]*q[2]-q[3]*q[3];		m_ROT(0,1)=2*(q[1]*q[2] -q[0]*q[3]);			m_ROT(0,2)=2*(q[3]*q[1]+q[0]*q[2]);
		m_ROT(1,0)=2*(q[1]*q[2]+q[0]*q[3]);				m_ROT(1,1)=q[0]*q[0]-q[1]*q[1]+q[2]*q[2]-q[3]*q[3];	    m_ROT(1,2)=2*(q[2]*q[3]-q[0]*q[1]);
		m_ROT(2,0)=2*(q[3]*q[1]-q[0]*q[2]);				m_ROT(2,1)=2*(q[2]*q[3]+q[0]*q[1]);				        m_ROT(2,2)=q[0]*q[0]-q[1]*q[1]-q[2]*q[2]+q[3]*q[3];
	}

	inline void normalizeQ(){
		q[0] = 1; q[1] = 0; q[2] = 0; q[3] = 0;
	}
};

#endif