#include <iostream>
#include <cmath>
#include "KDTreeCapable.h"
#include "Kd_GravityTree/CPointsSet.h"
#include "Kd_GravityTree/CEigenMetrix.h"
using namespace std;

extern bool  leastSquareErrorRigidTransformation6D(
	 deque<TMatchingPair>& in_correspondences,
	CPose3D							&out_transformation,
	double								&out_scale,
	const bool 							forceScaleToUnity);

inline double wrapToPi(double p){return (p/180)*pi;}
inline int PiTowrap(double p){return (int)(p*180/pi);}

void DisplayPose3D(CPose3D& o){
	cout<<"(x: "<<o.m_coords[0]<<", y: "<<o.m_coords[1]<<", z: "<<o.m_coords[2]<<" )"<<endl;
	cout<<"(yaw: "<<PiTowrap(o.m_yaw)<<", pitch: "<<PiTowrap(o.m_pitch)<<", roll: "<<PiTowrap(o.m_roll)<<" )"<<endl;
	cout<<"Q[4] = { "<<o._q[0]<<", "<<o._q[1]<<", "<<o._q[2]<<", "<<o._q[3]<<" }"<<endl;
}

void cicp()
{
		CPointsSet h; 
		CPose3D gaussPdf,lastMeanPose;
		deque<TMatchingPair> correspondences;
		
		gaussPdf.setFromValues(1.7,12.3,-3.5,70.5,25,42);
		gaussPdf.CalculateQ();
		/*ICP - parameters*/
		float umbral_dist = 0.1;
		float umbral_ang = (float)(5.0/180.0)*pi;
		float correspondencesRatio;
		bool keepApproaching = true;
		float smallestThresholdDist = 0.01;
		int nIterations = 0;
		float ALFA = 0.75;
		int maxIterations = 100;


		int nCorrespondences;
		do
		{
			CPoint3D  pivotPoint( gaussPdf );

			// ------------------------------------------------------
			//		Find the matching (for a points map)
			// ------------------------------------------------------
			h.computeMatchingWith3D(
					gaussPdf,			// The other map pose
					umbral_dist,			// Distance threshold
					umbral_ang,				// Angular threshold
					pivotPoint,				// Pivot point for angular measurements
					correspondences,		// Output
					correspondencesRatio,	// Ratio
					NULL				// MSE
					);

			nCorrespondences = correspondences.size();

			cout<<"MatchingPairs :" <<nCorrespondences<<endl;

			if ( !nCorrespondences )
			{
				// Nothing we can do !!
				keepApproaching = false;
			}
			else
			{
				// Compute the estimated pose, using Horn's method.
				// ----------------------------------------------------------------------
				double transf_scale;
				leastSquareErrorRigidTransformation6D( correspondences, gaussPdf, transf_scale, true );// true - We don't care scale-affection

				//cout << gaussPdf->mean << " scale: " << transf_scale << " corrs: " << correspondences.size() << endl;

				// If matching has not changed, decrease the thresholds:
				// --------------------------------------------------------
				keepApproaching = true;
				if	(!(fabs(lastMeanPose.x()-gaussPdf.x())>1e-6 ||
					fabs(lastMeanPose.y()-gaussPdf.y())>1e-6 ||
					fabs(lastMeanPose.z()-gaussPdf.z())>1e-6 ||
					fabs(wrapToPi(lastMeanPose.yaw()-gaussPdf.yaw()))>1e-6 ||
					fabs(wrapToPi(lastMeanPose.pitch()-gaussPdf.pitch()))>1e-6 ||
					fabs(wrapToPi(lastMeanPose.roll()-gaussPdf.roll()))>1e-6 ))
				{
					umbral_dist		*= ALFA;
					umbral_ang		*= ALFA;
					if (umbral_dist < smallestThresholdDist )
						keepApproaching = false;
				}

				lastMeanPose = gaussPdf;

			}	// end of "else, there are correspondences"

			// Next iteration:
			nIterations++;

			if (nIterations >=maxIterations && umbral_dist>smallestThresholdDist)
			{
				umbral_dist		*= ALFA;
			}

		} while	( (keepApproaching && nIterations<maxIterations) ||
					(nIterations >= maxIterations && umbral_dist>smallestThresholdDist) );

		/*Display results of ICP*/
		cout<<"Iteration times"<<nIterations<<endl;
		cout<<"original transformation Pose3D:"<<endl;
		DisplayPose3D(h._pose);
		cout<<"ICP 's transformation Pose3D: "<<endl;
		gaussPdf.CalculateEularFromQ();
		DisplayPose3D(gaussPdf);
	return ;

}

void main()
{
	cicp();
	//CPointsSet t;
	//t.DumpTofile("D:\\MyProjects\\Kd_GravityTree\\Kd_GravityTree\\log.xlsx");
	
	getchar();
	return;
}

//double a[16] = {214.9368,2.5384,-0.0861,-2.7065, \
//								 2.5384,22.7197,52.3896,131.69, \
//								-0.0861,52.3896,-128.734,41.427,\
//								-2.7065,131.69,41.427,-108.922};
// vecs[] = {1,0.011,0.001,-0.004};
//	CEigenM44 M(a,16);
//	double v[4],val;
//	M.EigenVectorMax(v,val);