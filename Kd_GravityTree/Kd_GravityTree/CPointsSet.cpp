#include "CPointsSet.h"
#include <fstream>
#include <iostream>

#define CLAMP(x , min , max) ((x) > (max) ? (max) : ((x) < (min) ? (min) : x))
/*CPose3D Realization*/
CPose3D::CPose3D()
	: m_ypr_uptodate(true), m_yaw(0),m_pitch(0),m_roll(0)
{
	m_coords[0] =
	m_coords[1] =
	m_coords[2] = 0;
	for(int i=0;i<4;i++)
		_q[i] = 0;
	_q[0] =1;
	for(int i=0; i<3 ;i ++)
		 for(int j=0; j<3 ;j++)
			 m_Rot[i][j] = 1.0;
}
CPose3D::CPose3D(const double x,const double y,const double z, const double yaw, const double pitch, const double roll)
	:  m_ypr_uptodate(false)
{
	setFromValues(x,y,z,yaw,pitch,roll);
}
CPose3D::CPose3D(CPose3D& o){
		this->m_coords[0] = o.m_coords[0];
		this->m_coords[1] = o.m_coords[1];
		this->m_coords[2] = o.m_coords[2];
		this->m_yaw = o.m_yaw;
		this->m_roll = o.m_roll;
		this->m_pitch = o.m_pitch;
		this->m_ypr_uptodate = o.m_ypr_uptodate;
		for(int i=0;i<4;i++)
			this->_q[i] = o._q[i];
			for(int i=0; i<3 ;i ++)
				 for(int j=0; j<3 ;j++)
					 m_Rot[i][j] = o.m_Rot[i][j];
}
CPose3D& CPose3D::operator = (CPose3D& o){
	if(this!=&o){
		this->m_coords[0] = o.m_coords[0];
		this->m_coords[1] = o.m_coords[1];
		this->m_coords[2] = o.m_coords[2];
		this->m_yaw = o.m_yaw;
		this->m_roll = o.m_roll;
		this->m_pitch = o.m_pitch;
		for(int i=0;i<4;i++)
			this->_q[i] = o._q[i];
		this->m_ypr_uptodate = o.m_ypr_uptodate;
			for(int i=0; i<3 ;i ++)
				 for(int j=0; j<3 ;j++)
					 m_Rot[i][j] = o.m_Rot[i][j];
	}
	return (*this);
}
void  CPose3D::setFromValues(
	const double		x0,
	const double		y0,
	const double		z0,
	const double		yaw,
	const double		pitch,
	const double		roll)
{
	m_coords[0] = x0;
	m_coords[1] = y0;
	m_coords[2] = z0;
	this->m_yaw = wrapToPi(yaw);
	this->m_pitch = wrapToPi(pitch);
	this->m_roll = wrapToPi(roll);

	m_ypr_uptodate = true;

	rebuildRotationMatrix();
	CalculateQ();
}


/*---------------------------------------------------------------
 Set the pose from 3D point and yaw/pitch/roll angles, in radians.
---------------------------------------------------------------*/
void  CPose3D::rebuildRotationMatrix()
{
	const double	cy = cos(m_yaw);
	const double	sy = sin(m_yaw);
	const double	cp = cos(m_pitch);
	const double	sp = sin(m_pitch);
	const double	cr = cos(m_roll);
	const double	sr = sin(m_roll);

	 const double rot_vals[][3] = {
		cy*cp,      cy*sp*sr-sy*cr,     cy*sp*cr+sy*sr,
		sy*cp,      sy*sp*sr+cy*cr,     sy*sp*cr-cy*sr,
		-sp,        cp*sr,              cp*cr
		};
	 for(int i=0; i<3 ;i ++)
		 for(int j=0; j<3 ;j++)
			 m_Rot[i][j] = rot_vals[i][j];
}
/*Interchangeble rotation between Eular angles and Quternion*/
void CPose3D::CalculateQ()//Calculate Q[4];
{
	double cx = cos(m_roll*0.5);double cy = cos(m_pitch);double cz= cos(m_yaw*0.5);
	double sx = sin(m_roll*0.5);double sy = sin(m_pitch*0.5); double sz = sin(m_yaw*0.5);

	_q[0] = cx*cy*cz + sx*sy*sz;
	_q[1] = sx*cy*cz - cx*sy*sz;
	_q[2] = cx*sy*cz + sx*cy*sz;
	_q[3] = cx*cy*sz - sx*sy*cz;

}
void CPose3D::CalculateEularFromQ() // Calculate Eular angles from Q[4]
{
	double w =_q[0];
	double x = _q[1] ;
	double y = _q[2] ;
	double z = _q[3] ;
	
	m_roll      = atan2(2 * (w * x + y * z) , 1 - 2 * (x * x + y * y));
	//m_roll = atan(2*(w*x + y*z)/(1 - 2 * (x * x + y * y)));
	m_pitch   = asin(CLAMP(2 * (w * y - z * x) , -1.0f , 1.0f));
	//m_yaw = atan(2 * (w * z + x * y)/( 1 - 2 * (y * y + z * z)));
	m_yaw     = atan2(2 * (w * z + x * y) , 1 - 2 * (y * y + z * z));
}
/*---------------------------------------------------------------
		composePoint
---------------------------------------------------------------*/
void CPose3D::composePoint(double lx,double ly,double lz, float &gx, float &gy, float &gz ) 
{
			/*const double	cy = cos(m_yaw);
			const double	sy = sin(m_yaw);
			const double	cp = cos(m_pitch);
			const double	sp = sin(m_pitch);
			const double	cr = cos(m_roll);
			const double	sr = sin(m_roll);*/

			gx=(float)(m_Rot[0][0]*lx+m_Rot[0][1]*ly+m_Rot[0][2]*lz+m_coords[0]);
			gy=(float)(m_Rot[1][0]*lx+m_Rot[1][1]*ly+m_Rot[1][2]*lz+m_coords[1]);
			gz=(float)(m_Rot[2][0]*lx+m_Rot[2][1]*ly+m_Rot[2][2]*lz+m_coords[2]);
}

void CPose3D::composePointQ(double lx,double ly,double lz, float &gx, float &gy, float &gz ) 
{
			const double t2 = _q[0]*_q[1]; const double t3 = _q[0]*_q[2]; const double t4 = _q[0]*_q[3]; 
			const double t5 =-_q[1]*_q[1]; const double t6 = _q[1]*_q[2]; const double t7 = _q[1]*_q[3];
			const double t8 =-_q[2]*_q[2]; const double t9 = _q[2]*_q[3]; const double t10=-_q[3]*_q[3];
			gx = 2*((t8+ t10)*lx+(t6 - t4)*ly+(t3+t7)*lz)+lx;
			gy = 2*((t4+  t6)*lx+(t5 +t10)*ly+(t9-t2)*lz)+ly;
			gz = 2*((t7-  t3)*lx+(t2 + t9)*ly+(t5+t8)*lz)+lz;
}
/*CPointsSet Relization*/
CPointsSet::CPointsSet(size_t n, double x, double y, double z,  double yaw,  double pitch,  double roll):x(),y(),z(),_N(n),
m_largestDistanceFromOrigin(0),_pose(x,y,z,yaw,pitch,roll)
{
	mark_as_modified();
	this->RandomGenerate(_N);
	this->TranslateOriginalPoints();
}
CPointsSet::~CPointsSet(){}

void CPointsSet::mark_as_modified() const
{
	m_largestDistanceFromOriginIsUpdated=false;
	kdtree_mark_as_outdated();
}

void CPointsSet::kdtree_fill_point_data(ANNpointArray &data, const int nDims) const
{
		const size_t N = this->size();

	if (nDims==2)
	{
		const float  *x_ptr = &x[0];
		const float  *y_ptr = &y[0];
		for(size_t i=0;i<N;i++)
		{
			data[i][0]= *x_ptr++;
			data[i][1]= *y_ptr++;
		}
	}
	else
	{
		const float  *x_ptr = &x[0];
		const float  *y_ptr = &y[0];
		const float  *z_ptr = &z[0];
		for(size_t i=0;i<N;i++)
		{
			data[i][0]= *x_ptr++;
			data[i][1]= *y_ptr++;
			data[i][2]= *z_ptr++;
		}
	}
}

void CPointsSet::RandomGenerate(size_t num){
	tx.clear();ty.clear();tz.clear();
	srand(unsigned(time(NULL)));
	/* Randomly generate num double points
	within space: { (x,y,z) | x[-6,+6]  y[-4,+4] z[0,20] }*/
	float fx,fy,fz;
	for(size_t i=0;i<num; i++){
			fx = (((float)(rand()%120000)*0.0001 + (float)(rand()%12000)*0.001 + (float)(rand()%1200)*0.01 + (float)(rand()%120)*0.1 ))/4.0 - 6;
			fy = (((float)(rand()%80000)*0.0001 + (float)(rand()%8000)*0.001 + (float)(rand()%800)*0.01 + (float)(rand()%80)*0.1 ))/4.0 - 4;
			fz = (((float)(rand()%200000)*0.0001 + (float)(rand()%20000)*0.001 + (float)(rand()%2000)*0.01 + (float)(rand()%200)*0.1 ))/4.0;
			tx.push_back(fx);
			ty.push_back(fy);
			tz.push_back(fz);
	}
	return ; 
}

void CPointsSet::TranslateOriginalPoints(){
	
	float ox,oy,oz;
	x.clear();y.clear();z.clear();
	for(size_t i=0; i<_N;i++)
	{
		this->_pose.composePointQ(tx[i],ty[i],tz[i],ox,oy,oz);
		this->x.push_back(ox);
		this->y.push_back(oy);
		this->z.push_back(oz);
	}

}
void CPointsSet::DumpTofile(const char* s)
{
		ofstream f(s);
		unsigned int i;
		if(f.is_open()){
			f<<"this cloud"<<endl;
			for(i=0;i<this->x.size();i++)
				f<<"tx: "<<x[i]<<" ty: "<<y[i]<<" tz: "<<z[i]<<"\n";
			f<<"other cloud"<<endl;
			for(i=0;i<this->x.size();i++)
				f<<"x: "<<tx[i]<<" y: "<<ty[i]<<" z: "<<tz[i]<<"\n";
			f.close();
		}
}

//// ------------------------------------------------------
//			//		Find the matching (for a points map)
//			// ------------------------------------------------------
//			m1->computeMatchingWith3D(
//					m2,						// The other map
//					gaussPdf->mean,			// The other map pose
//					umbral_dist,			// Distance threshold
//					umbral_ang,				// Angular threshold
//					pivotPoint,				// Pivot point for angular measurements
//					correspondences,		// Output
//					correspondencesRatio,	// Ratio
//					NULL,					// MSE
//					onlyKeepTheClosest,
//					onlyUniqueRobust );


/*---------------------------------------------------------------
				computeMatchingWith3D
---------------------------------------------------------------*/
void  CPointsSet::computeMatchingWith3Dgravity(
     CPose3D							&otherMapPose,
    float									maxDistForCorrespondence,
    float									maxAngularDistForCorrespondence,
     CPoint3D 							&angularDistPivotPoint,
     std::deque<TMatchingPair>&						correspondences,
    float									&correspondencesRatio,
    float									*sumSqrDist)
{
	size_t					nLocalPoints = this->size();
	size_t					nGlobalPoints = this->size();
	float					_sumSqrDist=0;
	size_t					_sumSqrCount = 0;
	size_t					nOtherMapPointsWithCorrespondence = 0;	// Number of points with one corrs. at least

	float					local_x_min= std::numeric_limits<float>::max(), local_x_max= -std::numeric_limits<float>::max();
	float					global_x_min=std::numeric_limits<float>::max(), global_x_max= -std::numeric_limits<float>::max();
	float					local_y_min= std::numeric_limits<float>::max(), local_y_max= -std::numeric_limits<float>::max();
	float					global_y_min=std::numeric_limits<float>::max(), global_y_max= -std::numeric_limits<float>::max();
	float					local_z_min= std::numeric_limits<float>::max(), local_z_max= -std::numeric_limits<float>::max();
	float					global_z_min=std::numeric_limits<float>::max(), global_z_max= -std::numeric_limits<float>::max();

	double					maxDistForCorrespondenceSquared;

	unsigned int			globalIdx,localIdx;

	vector<float>					x_locals,y_locals,z_locals;

	float 		*x_locals_it,*y_locals_it,*z_locals_it;
	const float *x_other_it,*y_other_it,*z_other_it,*x_global_it,*y_global_it,*z_global_it;

	// No correspondences initially:
	correspondences.clear();
	correspondencesRatio = 0;

	// Hay mapa global?
	if (!nGlobalPoints) return;  // No

	// Hay mapa local?
	if (!nLocalPoints)  return;  // No

	// Solo hacer matching si existe alguna posibilidad de que
	//  los dos mapas se toquen:
	// -----------------------------------------------------------

	// Transladar y rotar ya todos los puntos locales
	x_locals.resize(nLocalPoints);
	y_locals.resize(nLocalPoints);
	z_locals.resize(nLocalPoints);

	float x_local,y_local,z_local;

	for (   localIdx=0,
		    x_locals_it=&x_locals[0],
			y_locals_it=&y_locals[0],
			z_locals_it=&z_locals[0],
			x_other_it=&this->tx[0],
			y_other_it=&this->ty[0],
			z_other_it=&this->tz[0];
			localIdx<nLocalPoints;
			localIdx++)
	{
		// Translate and rotate each point in the "other" map:
		float  x_other = *x_other_it++;
		float  y_other = *y_other_it++;
		float  z_other = *z_other_it++;

		otherMapPose.composePoint( x_other,y_other,z_other,  x_local,y_local,z_local);


		*x_locals_it++ = x_local;
		*y_locals_it++ = y_local;
		*z_locals_it++ = z_local;

		// Find the bounding box:
		local_x_min = min(local_x_min,x_local);
		local_x_max = max(local_x_max,x_local);
		local_y_min = min(local_y_min,y_local);
		local_y_max = max(local_y_max,y_local);
		local_z_min = min(local_z_min,z_local);
		local_z_max = max(local_z_max,z_local);
	}

	// Find the bounding box:
	for (   globalIdx=0,
		    x_global_it=&x[0],
		    y_global_it=&y[0],
		    z_global_it=&z[0];
			globalIdx<nGlobalPoints;
			globalIdx++)
	{
		float global_x = *x_global_it++;
		float global_y = *y_global_it++;
		float global_z = *z_global_it++;

		global_x_min = min(global_x_min,global_x);
		global_x_max = max(global_x_max,global_x);
		global_y_min = min(global_y_min,global_y);
		global_y_max = max(global_y_max,global_y);
		global_z_min = min(global_z_min,global_z);
		global_z_max = max(global_z_max,global_z);
	}

	// Solo hacer matching si existe alguna posibilidad de que
	//  los dos mapas se toquen:
	if (local_x_min>global_x_max ||
		local_x_max<global_x_min ||
		local_y_min>global_y_max ||
		local_y_max<global_y_min) return;	// No hace falta hacer matching,
											//   porque es de CERO.

			std::vector<gravity_point> ga;
	// Loop for each point in local map:
	// --------------------------------------------------
	for ( localIdx=0,
			x_locals_it=&x_locals[0],
			y_locals_it=&y_locals[0],
			z_locals_it=&z_locals[0],
			x_other_it=&this->tx[0],
			y_other_it=&this->ty[0],
			z_other_it=&this->tz[0];
			localIdx<nLocalPoints;
			x_locals_it++,y_locals_it++,z_locals_it++,x_other_it++,y_other_it++,z_other_it++,localIdx++ )
	{
		// For speed-up:
		x_local = *x_locals_it;
		y_local = *y_locals_it;
		z_local = *z_locals_it;

//		bool thisLocalHasCorr = false;

		{
			// Find all the matchings in the requested distance:
			TMatchingPair		p,closestPair;


			// KD-TREE implementation

			// Use a KD-tree to look for the nearnest neighbor of:
			//   (x_local, y_local, z_local)
			// In "this" (global/reference) points map.

			float tmp_x,tmp_y,tmp_z;
			p.this_idx = kdTreeClosestPoint3D(
				x_local,  y_local, z_local,  // Look closest to this guy
				tmp_x, tmp_y, tmp_z, // save here the closest match
				p.errorSquareAfterTransformation // save here the min. distance squared
				);

			p.this_x = tmp_x; p.this_y = tmp_y; p.this_z = tmp_z;
			// Compute max. allowed distance:
			maxDistForCorrespondenceSquared = square(
						maxAngularDistForCorrespondence * angularDistPivotPoint.distance3DTo(x_local,y_local,z_local) +
						maxDistForCorrespondence );

			// Distance below the threshold??
			if ( p.errorSquareAfterTransformation < maxDistForCorrespondenceSquared )
			{
				   int index = HasPoint(ga,p.this_x, p.this_y, p.this_z);
			       if(-1!=index)//already included
			       	{
				      ga[index].insert(*x_other_it,*y_other_it,*z_other_it);
				}
				else
				{
					gravity_point  tp(p.this_x, p.this_y, p.this_z);
					tp.insert(*x_other_it,*y_other_it,*z_other_it);
					ga.push_back(tp);
					// At least one:
					nOtherMapPointsWithCorrespondence++;

					// Accumulate the MSE:
					_sumSqrDist+= p.errorSquareAfterTransformation;
					_sumSqrCount++;
				}

			
			}


		} // End of test_match
	} // For each local point

	//now we calculate the corresponding gravity_points
	for(size_t i = 0;i<ga.size();i++)
	{
		TMatchingPair	p;
		p.this_x = ga[i].gx; p.this_y = ga[i].gy; p.this_z = ga[i].gz;
		ga[i].calculate(&p.other_x, &p.other_y,&p.other_z);
		correspondences.push_back(p);
	}
	ga.clear();

	// If requested, copy sum of squared distances to output pointer:
	// -------------------------------------------------------------------
	if (sumSqrDist)
	{
		if (_sumSqrCount)
				*sumSqrDist = _sumSqrDist / static_cast<double>(_sumSqrCount);
		else	*sumSqrDist = 0;
	}

	// The ratio of points in the other map with corrs:
	//printf("In this 30 bucket %d points has been corresponded!\n",nOtherMapPointsWithCorrespondence);
	correspondencesRatio = nOtherMapPointsWithCorrespondence / static_cast<float>(nLocalPoints);

}

/*---------------------------------------------------------------
				computeMatchingWith3D
---------------------------------------------------------------*/
void  CPointsSet::computeMatchingWith3D(
     CPose3D							&otherMapPose,
    float									maxDistForCorrespondence,
    float									maxAngularDistForCorrespondence,
     CPoint3D 							&angularDistPivotPoint,
     std::deque<TMatchingPair>&						correspondences,
    float									&correspondencesRatio,
    float									*sumSqrDist)
{
	size_t					nLocalPoints = this->size();
	size_t					nGlobalPoints = this->size();
	float					_sumSqrDist=0;
	size_t					_sumSqrCount = 0;
	size_t					nOtherMapPointsWithCorrespondence = 0;	// Number of points with one corrs. at least

	float					local_x_min= std::numeric_limits<float>::max(), local_x_max= -std::numeric_limits<float>::max();
	float					global_x_min=std::numeric_limits<float>::max(), global_x_max= -std::numeric_limits<float>::max();
	float					local_y_min= std::numeric_limits<float>::max(), local_y_max= -std::numeric_limits<float>::max();
	float					global_y_min=std::numeric_limits<float>::max(), global_y_max= -std::numeric_limits<float>::max();
	float					local_z_min= std::numeric_limits<float>::max(), local_z_max= -std::numeric_limits<float>::max();
	float					global_z_min=std::numeric_limits<float>::max(), global_z_max= -std::numeric_limits<float>::max();

	double					maxDistForCorrespondenceSquared;

	unsigned int			globalIdx,localIdx;

	vector<float>					x_locals,y_locals,z_locals;

	float 		*x_locals_it,*y_locals_it,*z_locals_it;
	const float *x_other_it,*y_other_it,*z_other_it,*x_global_it,*y_global_it,*z_global_it;

	// No correspondences initially:
	correspondences.clear();
	correspondencesRatio = 0;

	// Hay mapa global?
	if (!nGlobalPoints) return;  // No

	// Hay mapa local?
	if (!nLocalPoints)  return;  // No

	// Solo hacer matching si existe alguna posibilidad de que
	//  los dos mapas se toquen:
	// -----------------------------------------------------------

	// Transladar y rotar ya todos los puntos locales
	x_locals.resize(nLocalPoints);
	y_locals.resize(nLocalPoints);
	z_locals.resize(nLocalPoints);

	float x_local,y_local,z_local;

	for (   localIdx=0,
		    x_locals_it=&x_locals[0],
			y_locals_it=&y_locals[0],
			z_locals_it=&z_locals[0],
			x_other_it=&this->tx[0],
			y_other_it=&this->ty[0],
			z_other_it=&this->tz[0];
			localIdx<nLocalPoints;
			localIdx++)
	{
		// Translate and rotate each point in the "other" map:
		float  x_other = *x_other_it++;
		float  y_other = *y_other_it++;
		float  z_other = *z_other_it++;

		otherMapPose.composePointQ( x_other,y_other,z_other,  x_local,y_local,z_local);


		*x_locals_it++ = x_local;
		*y_locals_it++ = y_local;
		*z_locals_it++ = z_local;

		// Find the bounding box:
		local_x_min = min(local_x_min,x_local);
		local_x_max = max(local_x_max,x_local);
		local_y_min = min(local_y_min,y_local);
		local_y_max = max(local_y_max,y_local);
		local_z_min = min(local_z_min,z_local);
		local_z_max = max(local_z_max,z_local);
	}

	// Find the bounding box:
	for (   globalIdx=0,
		    x_global_it=&x[0],
		    y_global_it=&y[0],
		    z_global_it=&z[0];
			globalIdx<nGlobalPoints;
			globalIdx++)
	{
		float global_x = *x_global_it++;
		float global_y = *y_global_it++;
		float global_z = *z_global_it++;

		global_x_min = min(global_x_min,global_x);
		global_x_max = max(global_x_max,global_x);
		global_y_min = min(global_y_min,global_y);
		global_y_max = max(global_y_max,global_y);
		global_z_min = min(global_z_min,global_z);
		global_z_max = max(global_z_max,global_z);
	}

	// Solo hacer matching si existe alguna posibilidad de que
	//  los dos mapas se toquen:
	if (local_x_min>global_x_max ||
		local_x_max<global_x_min ||
		local_y_min>global_y_max ||
		local_y_max<global_y_min) return;	// No hace falta hacer matching,
											//   porque es de CERO.

	// Loop for each point in local map:
	// --------------------------------------------------
	for ( localIdx=0,
			x_locals_it=&x_locals[0],
			y_locals_it=&y_locals[0],
			z_locals_it=&z_locals[0],
			x_other_it=&this->tx[0],
			y_other_it=&this->ty[0],
			z_other_it=&this->tz[0];
			localIdx<nLocalPoints;
			x_locals_it++,y_locals_it++,z_locals_it++,x_other_it++,y_other_it++,z_other_it++,localIdx++ )
	{
		// For speed-up:
		x_local = *x_locals_it;
		y_local = *y_locals_it;
		z_local = *z_locals_it;

//		bool thisLocalHasCorr = false;

		{
			// Find all the matchings in the requested distance:
			TMatchingPair		p,closestPair;


			// KD-TREE implementation

			// Use a KD-tree to look for the nearnest neighbor of:
			//   (x_local, y_local, z_local)
			// In "this" (global/reference) points map.

			float tmp_x,tmp_y,tmp_z;
			p.this_idx = kdTreeClosestPoint3D(
				x_local,  y_local, z_local,  // Look closest to this guy
				tmp_x, tmp_y, tmp_z, // save here the closest match
				p.errorSquareAfterTransformation // save here the min. distance squared
				);

			p.this_x = tmp_x; p.this_y = tmp_y; p.this_z = tmp_z;
			// Compute max. allowed distance:
			maxDistForCorrespondenceSquared = square(
						maxAngularDistForCorrespondence * angularDistPivotPoint.distance3DTo(x_local,y_local,z_local) +
						maxDistForCorrespondence );

			// Distance below the threshold??
			if ( p.errorSquareAfterTransformation < maxDistForCorrespondenceSquared )
			{
				 // Save all the correspondences??
				//p.this_z = z[p.this_idx];
				//p.other_idx = localIdx;
				p.other_x = *x_other_it;
				p.other_y = *y_other_it;
				p.other_z = *z_other_it;

				// save the correspondence:
				correspondences.push_back( p );

				// At least one:
				nOtherMapPointsWithCorrespondence++;

					// Accumulate the MSE:
					_sumSqrDist+= p.errorSquareAfterTransformation;
					_sumSqrCount++;
				}	
		} // End of test_match
	} // For each local point


	// If requested, copy sum of squared distances to output pointer:
	// -------------------------------------------------------------------
	if (sumSqrDist)
	{
		if (_sumSqrCount)
				*sumSqrDist = _sumSqrDist / static_cast<double>(_sumSqrCount);
		else	*sumSqrDist = 0;
	}

	// The ratio of points in the other map with corrs:
	//printf("In this 30 bucket %d points has been corresponded!\n",nOtherMapPointsWithCorrespondence);
	correspondencesRatio = nOtherMapPointsWithCorrespondence / static_cast<float>(nLocalPoints);

}
