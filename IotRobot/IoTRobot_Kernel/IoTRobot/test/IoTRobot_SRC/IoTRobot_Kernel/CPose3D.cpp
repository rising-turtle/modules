#include "CPose3D.h"

#define square(x) ((x)*(x))

using std::cout;
using std::endl;

/*---------------------------------------------------------------
getYawPitchRoll
---------------------------------------------------------------*/
void  CPose3D::getYawPitchRoll( double &yaw, double &pitch, double &roll ) 
{
	/*if( fabs(sqrt(square(m_ROT(0,0))+square(m_ROT(1,0))+square(m_ROT(2,0))) - 1 ) < 1e-5) cout<< "Homogeneous matrix is not orthogonal & normalized!: "<<endl;
	if( fabs(sqrt(square(m_ROT(0,1))+square(m_ROT(1,1))+square(m_ROT(2,1))) - 1 ) < 1e-5) cout<< "Homogeneous matrix is not orthogonal & normalized!: "<<endl;
	if(	fabs(sqrt(square(m_ROT(0,2))+square(m_ROT(1,2))+square(m_ROT(2,2))) - 1 ) < 1e-5) cout<< "Homogeneous matrix is not orthogonal & normalized!: "<<endl;*/

	// Pitch is in the range [-pi/2, pi/2 ], so this calculation is enough:
	pitch =  atan2( (double)(- m_ROT(2,0)), (double)hypot( m_ROT(0,0),m_ROT(1,0) ) ); //asin( - m_ROT(2,0) );

	// Roll:
	if ( (fabs(m_ROT(2,1))+fabs(m_ROT(2,2)))<10*std::numeric_limits<double>::epsilon() )
	{
		//Gimbal lock between yaw and roll. This one is arbitrarily forced to be zero.
		//Check http://reference.mrpt.org/svn/classmrpt_1_1poses_1_1_c_pose3_d.html. If cos(pitch)==0, the homogeneous matrix is:
		//When sin(pitch)==1:
		//  /0  cysr-sycr cycr+sysr x\   /0  sin(r-y) cos(r-y)  x\.
		//  |0  sysr+cycr sycr-cysr y| = |0  cos(r-y) -sin(r-y) y|
		//  |-1     0         0     z|   |-1    0         0     z|
		//  \0      0         0     1/   \0     0         0     1/
		//
		//And when sin(pitch)=-1:
		//  /0 -cysr-sycr -cycr+sysr x\   /0 -sin(r+y) -cos(r+y) x\.
		//  |0 -sysr+cycr -sycr-cysr y| = |0 cos(r+y)  -sin(r+y) y|
		//  |1      0          0     z|   |1    0          0     z|
		//  \0      0          0     1/   \0    0          0     1/
		//
		//Both cases are in a "gimbal lock" status. This happens because pitch is vertical.

		roll = 0.0;
		if (pitch>0) yaw=atan2((double)m_ROT(1,2),(double)m_ROT(0,2));
		else yaw=atan2((double)(-m_ROT(1,2)),(double)(-m_ROT(0,2)));
	}
	else
	{
		roll = atan2( (double)(m_ROT(2,1)), (double)m_ROT(2,2) );
		// Yaw:
		yaw = atan2( (double)(m_ROT(1,0)), (double)(m_ROT(0,0)) );
	}
}
void CPose3D::rebuildRotationMatrix(){
	const double	cy = cos(yaw);
	const double	sy = sin(yaw);
	const double	cp = cos(pitch);
	const double	sp = sin(pitch);
	const double	cr = cos(roll);
	const double	sr = sin(roll);

	EIGEN_ALIGN16  double rot_vals[] = {
		cy*cp,      cy*sp*sr-sy*cr,     cy*sp*cr+sy*sr,
		sy*cp,      sy*sp*sr+cy*cr,     sy*sp*cr-cy*sr,
		-sp,        cp*sr,              cp*cr
	};
	for(size_t i=0;i<3;i++)
		for(size_t j=0;j<3;j++)
			m_ROT(i,j) = rot_vals[i*3 + j];
}
/**  Makes "this = A (+) B"; this method is slightly more efficient than "this= A + B;" since it avoids the temporary object.
*  \note A or B can be "this" without problems.
*/
/*---------------------------------------------------------------
this = A + B
---------------------------------------------------------------*/
void CPose3D::composeFrom(const CPose3D& A, const CPose3D& B )
{
	//Was: m_HM.multiply( A.m_HM, B.m_HM );
	m_ROT=  A.m_ROT* B.m_ROT ;

	// The translation part HM(0:3,3)
	if (this==&B)
	{
		// we need to make a temporary copy of the vector:
		const double*  B_coords = B.m_coords;
		for (int r=0;r<3;r++)
			m_coords[r] = A.m_coords[r] + A.m_ROT(r,0)*B_coords[0]+A.m_ROT(r,1)*B_coords[1]+A.m_ROT(r,2)*B_coords[2];
	}
	else
	{
		for (int r=0;r<3;r++)
			m_coords[r] = A.m_coords[r] + A.m_ROT(r,0)*B.m_coords[0]+A.m_ROT(r,1)*B.m_coords[1]+A.m_ROT(r,2)*B.m_coords[2];
	}

	//m_ypr_uptodate=false;
}
//
///** Rotate a 3D point (lx,ly,lz) -> (gx,gy,gz) as described by this quaternion
//*/
//void CPose3D::rotatePoint( double lx, double ly, double lz, double &gx,double &gy,double &gz ) 
//{
//	const double t2 = q[0]*q[1]; const double t3 = q[0]*q[2]; const double t4 = q[0]*q[3]; const double t5 =-q[1]*q[1]; const double t6 = q[1]*q[2];
//	const double t7 = q[1]*q[3]; const double t8 =-q[2]*q[2]; const double t9 = q[2]*q[3]; const double t10=-q[3]*q[3];
//	gx = 2*((t8+ t10)*lx+(t6 - t4)*ly+(t3+t7)*lz)+lx;
//	gy = 2*((t4+  t6)*lx+(t5 +t10)*ly+(t9-t2)*lz)+ly;
//	gz = 2*((t7-  t3)*lx+(t2 + t9)*ly+(t5+t8)*lz)+lz;
//}

CPose3D CPose3D::operator+(const CPose3D& current){
	CPose3D   ret;
	ret.composeFrom(*this,current);
	return ret;
}

CPose3D CPose3D::operator+=(const CPose3D& current ){
	composeFrom(*this,current);
	return *this;
}
CPose3D& CPose3D::operator =(const CPose3D& v){
	if(this == &v){
		return *this;
	}
	for(size_t i=0;i<3;i++)
		this->m_coords[i] = v.m_coords[i];
	this->yaw = v.yaw; this->pitch = v.pitch; this->roll = v.roll;
	this->m_ROT = v.m_ROT;
	return *this;
}