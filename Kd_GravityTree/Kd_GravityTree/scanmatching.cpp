#include "CPointsSet.h"
#include "..\KDTreeCapable.h"
#include "CEigenMetrix.h"
#include <cmath>

#define CLAMP(x , min , max) ((x) > (max) ? (max) : ((x) < (min) ? (min) : x))

template<class T>
inline T square(T x){return (x*x);}



//*---------------------------------------------------------------
//	leastSquareErrorRigidTransformation6D
//  ---------------------------------------------------------------*/
bool  leastSquareErrorRigidTransformation6D(
	 deque<TMatchingPair>	&in_correspondences,
	CPose3D							&out_transformation,
	double								&out_scale,
	const bool 							forceScaleToUnity)
{

	CPoint3D cL, cR;
	double S[3][3] = {0,0,0,0,0,0,0,0,0};
	CEigenM44 N;
	//CEigenM44 Z, D;

	//vector<double> v;
	const size_t nMatches = in_correspondences.size();
	double s; // Scale

	// Compute the centroid
	 deque<TMatchingPair>::const_iterator	itMatch;

	for(itMatch = in_correspondences.begin(); itMatch != in_correspondences.end(); itMatch++)
	{
		cL.x_incr( itMatch->other_x );
		cL.y_incr( itMatch->other_y );
		cL.z_incr( itMatch->other_z );

		cR.x_incr( itMatch->this_x );
		cR.y_incr( itMatch->this_y );
		cR.z_incr( itMatch->this_z );
	}
	const double F = 1.0/nMatches;
	cL *= F;
	cR *= F;

	deque<TMatchingPair>			auxList( in_correspondences );
	deque<TMatchingPair>::iterator auxIt;
	// Substract the centroid
	for( auxIt = auxList.begin(); auxIt != auxList.end(); auxIt++ )
	{
		auxIt->other_x -= cL.get_x();
		auxIt->other_y -= cL.get_y();
		auxIt->other_z -= cL.get_z();

		auxIt->this_x -= cR.get_x();
		auxIt->this_y -= cR.get_y();
		auxIt->this_z -= cR.get_z();
	}

	// Compute the S matrix of products
	//S.setSize(3,3);
	//S.fill(0);
	for( auxIt = auxList.begin(); auxIt != auxList.end(); auxIt++ )
	{
		S[0][0] += auxIt->other_x * auxIt->this_x;
		S[0][1] += auxIt->other_x * auxIt->this_y;
		S[0][2] += auxIt->other_x * auxIt->this_z;

		S[1][0] += auxIt->other_y * auxIt->this_x;
		S[1][1] += auxIt->other_y * auxIt->this_y;
		S[1][2] += auxIt->other_y * auxIt->this_z;

		S[2][0] += auxIt->other_z * auxIt->this_x;
		S[2][1] += auxIt->other_z * auxIt->this_y;
		S[2][2] += auxIt->other_z * auxIt->this_z;
	}

	//N.setSize(4,4);
	//N.fill(0);

	N.SetValue(0,0, S[0][0]+ S[1][1] + S[2][2]);
	N.SetValue(0,1, S[1][2] - S[2][1]);
	N.SetValue(0,2,S[2][0] - S[0][2]);
	N.SetValue(0,3, S[0][1] - S[1][0]);

	N.SetValue(1,0,S[1][2] - S[2][1]);
	N.SetValue(1,1, S[0][0]- S[1][1] - S[2][2]);
	N.SetValue(1,2,S[0][1]+ S[1][0]);
	N.SetValue(1,3, S[2][0] + S[0][2]);

	N.SetValue(2,0, S[2][0] - S[0][2]);
	N.SetValue(2,1, S[0][1]+ S[1][0]);
	N.SetValue(2,2,-S[0][0] + S[1][1] - S[2][2]);
	N.SetValue(2,3, S[1][2] + S[2][1]);

	N.SetValue(3,0, S[0][1] - S[1][0]);
	N.SetValue(3,1, S[2][0] + S[0][2]);
	N.SetValue(3,2,S[1][2] + S[2][1]);
	N.SetValue(3,3,-S[0][0] - S[1][1] + S[2][2]);

	// q is the quaternion correspondent to the greatest eigenvector of the N matrix (last column in Z)
	//N.eigenVectors( Z, D );
	//Z.extractCol( Z.getColCount()-1, v );
	double v[4]; double val;
	N.EigenVectorMax(v,val);

	//CPose3DQuat q;
	double q[7];
	
	for(unsigned int i = 0; i < 4; i++ )			// Set out_transformation [rotation]
			q[i+3] = v[i];

	// Compute scale
	double	num = 0.0;
	double	den = 0.0;
	for( auxIt = auxList.begin(); auxIt != auxList.end(); auxIt++ )
	{
		den += square<double>(auxIt->other_x) + square<double>(auxIt->other_y) + square<double>(auxIt->other_z);
		num += square<double>(auxIt->this_x) + square<double>(auxIt->this_y) + square<double>(auxIt->this_z);
	}
	s = sqrt( num/den );

	// Enforce scale to be 1
	out_scale = s;
	if (forceScaleToUnity)
		s = 1.0;

	CPoint3D pp, aux;
	//q.composePoint( cL.x, cL.y, cL.z, pp.x, pp.y, pp.z );
	pp*=s;


	/*Calculate Eular angle from q[4]
		From http://www.cppblog.com/heath/archive/2009/12/13/103127.html
	*/
	CPose3D _t;
	double w =out_transformation._q[0] = _t._q[0] = q[3];
	double x = out_transformation._q[1] = _t._q[1] = q[4];
	double y = out_transformation._q[2] = _t._q[2] = q[5];
	double z = out_transformation._q[3] = _t._q[3] = q[6];
	
	_t.m_roll = out_transformation.m_roll      = atan2(2 * (w * x + y * z) , 1 - 2 * (x * x + y * y));
	_t.m_pitch = out_transformation.m_pitch   = asin(CLAMP(2 * (w * y - z * x) , -1.0f , 1.0f));
	_t.m_yaw = out_transformation.m_yaw     = atan2(2 * (w * z + x * y) , 1 - 2 * (y * y + z * z));

	_t.composePointQ(cL.x, cL.y, cL.z, pp.x, pp.y, pp.z);

	// Set out_transformation [traslation]
	out_transformation.m_coords[0] = cR.x - pp.x;	// X
	out_transformation.m_coords[1] = cR.y - pp.y;	// Y
	out_transformation.m_coords[2] = cR.z - pp.z;	// Z

	out_transformation.rebuildRotationMatrix();//Update prediction pose3D

	return true;
}
/*---------------------------------------------------------------
			leastSquareErrorRigidTransformation in 6D
  ---------------------------------------------------------------*/
	// Algorithm:
	// 0. Preliminary
	//		pLi = { pLix, pLiy, pLiz }
	//		pRi = { pRix, pRiy, pRiz }
	// -------------------------------------------------------
	// 1. Find the centroids of the two sets of measurements:
	//		cL = (1/n)*sum{i}( pLi )		cL = { cLx, cLy, cLz }
	//		cR = (1/n)*sum{i}( pRi )		cR = { cRx, cRy, cRz }
	//
	// 2. Substract centroids from the point coordinates:
	//		pLi' = pLi - cL					pLi' = { pLix', pLiy', pLiz' }
	//		pRi' = pRi - cR					pRi' = { pRix', pRiy', pRiz' }
	//
	// 3. For each pair of coordinates (correspondences) compute the nine possible products:
	//		pi1 = pLix'*pRix'		pi2 = pLix'*pRiy'		pi3 = pLix'*pRiz'
	//		pi4 = pLiy'*pRix'		pi5 = pLiy'*pRiy'		pi6 = pLiy'*pRiz'
	//		pi7 = pLiz'*pRix'		pi8 = pLiz'*pRiy'		pi9 = pLiz'*pRiz'
	//
	// 4. Compute S components:
	//		Sxx = sum{i}( pi1 )		Sxy = sum{i}( pi2 )		Sxz = sum{i}( pi3 )
	//		Syx = sum{i}( pi4 )		Syy = sum{i}( pi5 )		Syz = sum{i}( pi6 )
	//		Szx = sum{i}( pi7 )		Szy = sum{i}( pi8 )		Szz = sum{i}( pi9 )
	//
	// 5. Compute N components:
	//		N =	[ Sxx+Syy+Szz	Syz-Szy			Szx-Sxz			Sxy-Syx		 ]
	//			    [ Syz-Szy		Sxx-Syy-Szz		Sxy+Syx			Szx+Sxz		 ]
	//				[ Szx-Sxz		Sxy+Syx			-Sxx+Syy-Szz	Syz+Szy		 ]
	//				[ Sxy-Syx		Szx+Sxz			Syz+Szy			-Sxx-Syy+Szz ]
	//
	// 6. Rotation represented by the quaternion eigenvector correspondent to the higher eigenvalue of N
	//
	// 7. Scale computation (symmetric expression)
	//		s = sqrt( sum{i}( square(abs(pRi')) / sum{i}( square(abs(pLi')) ) )
	//
	// 8. Translation computation (distance between the Right centroid and the scaled and rotated Left centroid)
	//		t = cR-sR(cL)

