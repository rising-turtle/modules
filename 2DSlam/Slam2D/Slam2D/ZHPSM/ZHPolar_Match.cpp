#include "ZHPolar_Match.h"
#include <iostream>
#include <cmath>
#include <fstream>

#define SQ(x) ((x)*(x))

using namespace std;
namespace{
	// obtain the param from the name of laser
	Base_PARAM* getParam(std::string laser_name){
		static std::map<std::string, Base_PARAM* (*)() > _hash;

		if(_hash.size()==0){ // first time
			_hash[_BEARING_361] = ObtainParam<_PM_SICK_LMS511>;
			_hash[_BEARING_181] = ObtainParam<_PM_SICK_LMS211>;
			_hash[_BEARING_541] = ObtainParam<_PM_SICK_LMS151>;
		}
		if(_hash.find(laser_name)==_hash.end()){
			return NULL;
		}
		return (*_hash[laser_name])();
	}
}
CPolarMatch::CPolarMatch(string laser_name):
m_bReady(false)
{
	m_pParam = getParam(laser_name);
	if(m_pParam == NULL){
		cout<<"laser is not right!"<<endl;
		m_bReady = false;
	}
	else
		m_bReady = true;
	if(m_bReady)
	{
		pm_fi = new PM_TYPE[m_pParam->pm_l_points];
		pm_si = new PM_TYPE[m_pParam->pm_l_points];
		pm_co = new PM_TYPE[m_pParam->pm_l_points];
		pm_init();
	}
}

CPolarMatch::~CPolarMatch()
{
	delete []pm_fi;
	delete []pm_si;
	delete []pm_co;
	clearRecord();
}

void CPolarMatch::clearRecord(){
	for(int i=0;i<m_SickScans.size();i++)
		if(m_SickScans[i]!=NULL)
		{
			delete m_SickScans[i];
			m_SickScans[i] = NULL;
		}
	m_SickScans.clear(); // this will not release corresponding memory 
}

void CPolarMatch::pm_init() // init bearing variables
{
	if(m_bReady){
		for ( int i=0;i<m_pParam->pm_l_points;i++ )
		{
			pm_fi[i] = ( ( float ) i ) *m_pParam->pm_dfi + m_pParam->pm_fi_min;
			pm_si[i] = sinf ( pm_fi[i] );
			pm_co[i] = cosf ( pm_fi[i] );
		}
	}
}

bool CPolarMatch::readCarmon(string filename, string laser){
	if(!m_bReady){
		cout<<"Laser is not right initialized!"<<endl;
		return false;
	}

	ifstream inf(filename.c_str());
	if(!inf.is_open()){
		cout<<"failed to open file: "<<filename<<endl;
		return false;
	}

	// clear previous records
	clearRecord();
	
	// --- main loop ---
	PMScan ls(m_pParam->pm_l_points);
	ls.rx = 0;
	ls.ry = 0;
	ls.t = 0;
	ls.th = 0;

	char line[8192];
	char sensorName[255];
	//string tag;
	double timestamp;
	double start, fov, resolution, maxRange, accuracy;
	int laserType, remissionMode, num_points;

	bool bFirst = true;

	string delim(" ");
	if(laser=="")	// ���������Լ�������
	{
		laser = m_pParam->pm_laser_name;
		delim=string("\t");
	}

	while(inf.getline(line,8192)){
		string tag(strtok(line,delim.c_str()));
		if(tag!=laser)
			continue;
		laserType = (int)(atof(strtok(NULL,delim.c_str())));	
		start = atof(strtok(NULL,delim.c_str()));
		fov = atof(strtok(NULL,delim.c_str()));
		resolution = atof(strtok(NULL,delim.c_str()));
		maxRange = atof(strtok(NULL,delim.c_str()));
		accuracy = atof(strtok(NULL,delim.c_str()));
		remissionMode = atoi(strtok(NULL,delim.c_str()));
		num_points = atoi(strtok(NULL,delim.c_str()));

		if(bFirst){
			bFirst = false;
			m_pParam->pm_fi_min = start;
			m_pParam->pm_fi_max = start + fov*PM_D2R;
			//m_pParam->pm_dfi = resolution;
			pm_init();
		}

		// for laser data in intel-lab.log contains 180 0-179 while SICK 200 181 0-180
		if(num_points !=m_pParam->pm_l_points-1 && num_points != m_pParam->pm_l_points){
			cout<<"ERROR READING DATA!"<<endl;
			return false;
		}
		int cnt=0;
		for(int i=0;i<num_points;i++){
			++cnt;
			ls.r[i] = atof(strtok(NULL,delim.c_str()))*100.0; // from [m] 2 [cm]
			ls.x[i] = ls.r[i] * pm_co[i];
			ls.y[i] = ls.r[i] * pm_si[i];
			ls.bad[i] = 0;
			if(ls.r[i]<PM_MIN_RANGE){
				ls.r[i]=m_pParam->pm_max_range+1; //set it to a value larger than the max
			}
		}
	
		if(delim.c_str()==string(" ")) // ����ground truth���ݣ���odometry data
		{	
			// remission ������
			strtok(NULL,delim.c_str());
			// ��ʵ������laser������
			ls.rx = atof(strtok(NULL,delim.c_str()));
			ls.ry = atof(strtok(NULL,delim.c_str()));
			ls.th = atof(strtok(NULL,delim.c_str()));
		}
		if(cnt!=num_points){
			cout<<"not enough points in file!"<<endl;
			return false;
		}
		if(num_points == m_pParam->pm_l_points-1){
			ls.r[num_points] = m_pParam->pm_max_range+1;
			ls.bad[num_points] = 0;
		}
		m_SickScans.push_back(new PMScan(ls));
	}
	
	return (m_SickScans.size()>0);
}

bool CPolarMatch::readFlirt(string filename)
{
	return readCarmon(filename,string("ROBOTLASER1"));
}

bool CPolarMatch::readSICK(string filename)
{
	if(!m_bReady){
		cout<<"Laser is not right initialized!"<<endl;
		return false;
	}
	ifstream inf(filename.c_str());
	if(!inf.is_open()){
		cout<<"failed to open file: "<<filename<<endl;
		return false;
	}

	int N;
	char line[8192];
	PMScan* pScan = new PMScan(m_pParam->pm_l_points);
	pScan->rx = 0;
	pScan->ry = 0;
	pScan->th = 0;
	pScan->t = 0;
	while(inf.getline(line,8192)){
		strtok(line, " ");
		N = atof(strtok(NULL," "));
		if(N > m_pParam->pm_l_points)
		{
			cout<<"laser points are more than expected!"<<endl;
			N = m_pParam->pm_l_points;
		}
		for(int i=0;i<N;i++){
			pScan->r[i] = atof(strtok(NULL," "))*100.0; //from [m] 2 [cm]
			pScan->x[i] = pScan->r[i] * pm_co[i];
			pScan->y[i] = pScan->r[i] * pm_si[i];
			if(pScan->r[i]<PM_MIN_RANGE){
				pScan->r[i] = m_pParam->pm_max_range + 1;
			}
			pScan->bad[i] = 0;
		}
		m_SickScans.push_back(new PMScan(*pScan));
	}
	return m_SickScans.size()>0;
}

void CPolarMatch::runFlirtFile(string filename, int run_num)
{
	if(!readFlirt(filename)){
		cout<<"failed to read file: "<<filename<<endl;
		return ;
	}
	runlog(run_num);
	//runlogImproved(run_num);
}

// run PSM using SICK log 
void CPolarMatch::runSICKFile(string filename,int run_num)
{
	if(!readSICK(filename)){
		cout<<"failed to read file: "<<filename<<endl;
		return ;
	}
	runlogImproved(run_num);
}
 
// run our file log like carmen log
void CPolarMatch::runOurFile(string filename, int run_num )
{
	if(!readCarmon(filename)){
		cout<<"failed to read file: "<<filename<<endl;
		return ;
	}
	runlogImproved(run_num);
}


void CPolarMatch::runlog(int run_num,int start_frame) // run slam using log data
{
	if(!m_bReady){
		cout<<"Laser is not initialized!"<<endl;
		return ;
	}
	if(m_SickScans.size()<=0){
		cout<<"No record data is prepared!"<<endl;
		return ;
	}
	// trajectory record
#ifdef NEW_OP
	// ofstream trajectory("d://exprdata//zhnew.log");
	// ofstream trajectory("d://exprdata//fr079_comp//psm.log");
	ofstream trajectory("d://exprdata//intel_comp//psm.log");
#endif
#ifdef VEC_OP
	ofstream trajectory("d://exprdata//zhvec.log");
#endif
	if(!trajectory.is_open())
	{
		cout<<"failed to open output trajectory!"<<endl;
	}

	double slam_t;
	bool bFirst = 1;
	int cnt = 0;
	int err = 0;
	if(run_num < 0)
		run_num = m_SickScans.size()+1;

	// here we just simply using odometry to filter false matching!
	double odometry_thre = 0.6982; // almost 40 // 0.87264; // almost 50 degree

	// main loop match each frame with its precursor
	double rx=0,ry=0,th=0;//robot starts at 0,0,0
	double xx_last=0,yy_last=0,tth_last=0;  
	double orx,ory,oth;
	PMScan* ls;
	PMScan* ls_last;
	while(cnt+start_frame<m_SickScans.size() && cnt <run_num){
		ls = m_SickScans[cnt+start_frame];
		cnt++;

		pm_preprocessScan(ls);
		if(bFirst){
			rx = (ls->rx)*100;
			ry = (ls->ry)*100;
			th = ls->th;
			ls_last = ls;
			bFirst = false;
			trajectory<<ls->rx<<" "<<ls->ry<<" "<<th<<endl;
		}else{

			// ���odometry��ʾ�Ƕȱ仯���󣬾Ͳ���Ҫ��scan-matching�ķ�������λ��
			if(fabs(ls->th-th) > odometry_thre)
			{
				printf("Odometry: Using odometry data at frame: %d\n",cnt+start_frame);
				trajectory<<ls->rx<<" "<<ls->ry<<" "<<ls->th<<endl;
				ls_last = ls; 
				rx = (ls->rx)*100;
				ry = (ls->ry)*100;
				th = ls->th;
				continue;
			}

			// save for using the right odometry value
			orx = ls->rx;
			ory = ls->ry;
			oth = ls->th;

			//matching without the previous result as prior info
			ls->rx = ls_last->rx = 0;
			ls->ry = ls_last->ry = 0;
			ls->th = ls_last->th = 0;

			bool matchFailed = false;

			try{
				pm_psm(ls_last,ls);
			}catch(int err){
				cerr<<"Error caught: psm failed match at: "<<cnt<<endl;
				matchFailed = true;
			}
			if(matchFailed){
				try{
					pm_icp(ls_last,ls);
				}catch(int err){
					cerr<<"Error caught: icp failed match at: "<<cnt<<endl;
					printf("Frame %d is discarded!",cnt);
					continue;
				}
			}			
			
			PM_TYPE err_idx = pm_error_index2(ls_last,ls);
			if(err_idx> 10){
				printf("Error: Using odometry data at frame: %d\n",cnt+start_frame);
				ls_last = ls; 
				rx = orx*100;
				ry = ory*100;
				th = oth;
				trajectory<<orx<<" "<<ory<<" "<<oth<<endl;
				continue;
			}

			// the obtained pose is relative to the reference of ls_last      
			//convert the match result into global pose
			double xx,yy,tth;

			xx = ls->rx*cos(th) - ls->ry*sin(th) + rx;
			yy = ls->rx*sin(th) + ls->ry*cos(th) + ry;
			tth= th + ls->th;

			rx = xx;
			ry = yy;
			th = tth;

			ls_last = ls;

			trajectory<<(xx/100.0)<<" "<<(yy/100.0)<<" "<<tth<<endl;
		}//else
	}//while
	trajectory.close();
	return ;
}

void CPolarMatch::runlogImproved(int run_num,int start_frame)	// rum slam using log data
{
	if(!m_bReady){
		cout<<"Laser is not initialized!"<<endl;
		return ;
	}
	if(m_SickScans.size()<=0){
		cout<<"No record data is prepared!"<<endl;
		return ;
	}
	// trajectory record
#ifdef NEW_OP
	// ofstream trajectory("d://exprdata//intel_comp//psm.log");
	ofstream trajectory("d://exprdata//fr079_comp//psm.log");
#endif
#ifdef VEC_OP
	ofstream trajectory("d://exprdata//zhvec.log");
#endif
	if(!trajectory.is_open())
	{
		cout<<"failed to open output trajectory!"<<endl;
	}

	double slam_t;
	bool bFirst = 1;
	int cnt = 0;
	int err = 0;
	if(run_num < 0)
		run_num = m_SickScans.size()+1;

	// main loop match each frame with its precursor
	double rx=0,ry=0,th=0;//robot starts at 0,0,0
	double xx_last=0,yy_last=0,tth_last=0;  
	PMScan* ls;
	PMScan* ls_last;
	PMScan* ls_ref;
	while(cnt+start_frame<m_SickScans.size() && cnt <run_num){
		ls = m_SickScans[cnt+start_frame];
		cnt++;

		pm_preprocessScan(ls);
		if(bFirst){
			ls->rx = 0;
			ls->ry = 0;
			ls->th = 0;
			ls_last = ls;
			ls_ref = ls;
			bFirst = false;
			trajectory<<"0"<<" "<<"0"<<" "<<"0"<<endl;
		}else{
			
			//matching using the previous result as prior info
			ls->rx = ls_last->rx;
			ls->ry = ls_last->ry;
			ls->th = ls_last->th;
			//matching without prior info
			// ls_ref->rx = 0;ls_ref->ry=0;ls_ref->th=0;

			bool matchFailed = false;

			try{
				pm_psm(ls_ref,ls);
			}catch(int err){
				cerr<<"Error caught: psm failed match at "<<cnt<<endl;
				matchFailed = true;
			}		

			PM_TYPE err_idx = pm_error_index2(ls_last,ls);
			bool refSwitched = false;
			if(matchFailed || err_idx > 5.0)
			{
				bool switch2icp =false;
				refSwitched = true;
				printf(" Switching reference scan.\n");
				ls_ref = ls_last;        
				ls->rx = 0;ls->ry = 0; ls->th = 0;
				ls_ref->rx = 0;ls_ref->ry = 0; ls_ref->th = 0;
				try{
					pm_psm( ls_ref, ls );
				}catch(int err){
					cerr<<"psm failed again! switch to icp!"<<endl;
					switch2icp = true;
				}
				if(switch2icp){
					printf("switch to icp at: %d \n",cnt);
					try{
						pm_icp(ls_ref,ls);
					}catch(int err){
						cerr<<"icp also failed at: "<<cnt<<endl;
						printf("Frame %d is discarded!\n",cnt);
						continue;
					}
				}
				rx = xx_last;
				ry = yy_last;
				th = tth_last;
				ls_ref = ls_last;
			}
			//convert the match result into global pose
			double xx,yy,tth;

			xx = ls->rx*cos(th) - ls->ry*sin(th) + rx;
			yy = ls->rx*sin(th) + ls->ry*cos(th) + ry;
			// tth= norm_a(th + ls->th);
			tth = th + ls->th;
			trajectory<<(xx/100.0)<<" "<<(yy/100.0)<<" "<<tth<<endl;
			// global position
			ls->rx = xx; ls->ry = yy; ls->th = tth;           
			/*	if(refSwitched || cnt == 2)
			pm_plotScanAt(&ls,rx,ry,th,"red",1.0);*/
			// ls->rx = 0;ls->ry=0;ls->th=0;     
			ls_last = ls; 
			xx_last = xx;
			yy_last = yy;
			tth_last = tth;
		}//else
		
	}//while
	trajectory.close();
	return ;
}

/** @brief Prepares a scan for scan matching.

Filters the scan using median filter, finds far away points and segments the scan.
@param ls The scan to be preprocessed.
*/
void CPolarMatch::pm_preprocessScan(PMScan *ls)
{
	pm_median_filter(ls);
	pm_find_far_points(ls);
	pm_segment_scan(ls);
}

/** @brief Match two laser scans using polar scan matching. 

Minimizes the sum of square range residuals through changing lsa->rx, lsa->ry, lsa->th.
The error is minimized by iterating a translation estimation step followed by an
orientation search step.

PSM was not explicitly designed for laser scan matching based odometry where scans with small
pose difference are matched with each other without any prior pose information. However when 
using PSM for this purpose, reduce the values of PM_MAX_ERROR, PM_WEIGHTING_FACTOR to 
reflect the small inter-scan motion. Also by reducing the value of PM_STOP_COND,
larger matching accuracy can be achieved. The currently implemented error estimation 
functions are not useful for laser odometry error estimation.

Limitations: due to the nature of the association rule divergence in a slow rate
may be experienced in rooms where there are not many features to constrain
the solution in all directions. This can occur for examples in corridor-like environments 
including rooms where the room directly in front of the laser is outside of 
the range of the laser range finder.

@param lsr The reference scan.
@param lra The current scan. 
*/
PM_TYPE CPolarMatch::pm_psm ( const PMScan *lsr,PMScan *lsa )
{
	//PMScan    act, ref;
	PMScan act(*lsa);//copies of current and reference scans
	PMScan ref(*lsr);

	PM_TYPE   rx,ry,rth,ax,ay,ath;//robot pos at ref and current scans
	PM_TYPE   t13,t23,LASER_Y = PM_LASER_Y;
	vector<PM_TYPE>   new_r(m_pParam->pm_l_points);//interpolated r at measurement bearings
	vector<int>       new_bad(m_pParam->pm_l_points);//bad flags of the interpolated range readings
	PM_TYPE   C = PM_WEIGHTING_FACTOR;//weighting factor; see dudek00
	int       iter,small_corr_cnt=0;
	PM_TYPE   dx=0,dy=0,dth=0;//match error, current scan corrections
	PM_TYPE   avg_err = 100000000.0;

#ifdef  PM_GENERATE_RESULTS
	double start_tick, dead_tick,end_tick,end_tick2;
	FILE *f;
	f = fopen ( PM_TIME_FILE,"w" );
	dead_tick = 0;
	start_tick =pm_msec();
#endif

	//act = *lsa;
	//ref = *lsr;

	rx =  ref.rx; ry = ref.ry; rth = ref.th;
	ax =  act.rx; ay = act.ry; ath = act.th;

	//transformation of the current scan laser scanner coordinates into the reference
	//laser scanner's coordinate system:
	t13 = sinf ( rth-ath ) *LASER_Y+cosf ( rth ) *ax+sinf ( rth ) *ay-sinf ( rth ) *ry-rx*cosf ( rth );
	t23 = cosf ( rth-ath ) *LASER_Y-sinf ( rth ) *ax+cosf ( rth ) *ay-cosf ( rth ) *ry+rx*sinf ( rth )-LASER_Y;

	ref.rx = 0;   ref.ry = 0;   ref.th = 0;
	act.rx = t13; act.ry = t23; act.th = ath-rth;

	ax = act.rx; ay = act.ry; ath = act.th;
	//from now on act.rx,.. express the laser's position in the reference frame

	iter = -1;
	while ( ++iter < PM_MAX_ITER && small_corr_cnt < 3 ) //Has to be a few small corrections before stopping.
	{
		if ( ( fabsf ( dx ) +fabsf ( dy ) +fabsf ( dth ) ) < PM_STOP_COND )
			small_corr_cnt++;
		else
			small_corr_cnt=0;

#ifdef  PM_GENERATE_RESULTS
		end_tick =pm_msec();
		fprintf ( f,"%i %lf %lf %lf %lf\n",iter,
			end_tick-start_tick-dead_tick ,ax,ay,ath*PM_R2D );
		end_tick2 =pm_msec();
		dead_tick += end_tick2- end_tick;
#endif


#ifdef GR
		dr_erase();
		dr_circle ( ax,ay,5.0,"green" );
		dr_line ( 0,-100,200,-100,"black" );
		dr_line ( 0,-200,200,-200,"black" );
		for(int i=0;i<PM_L_POINTS;i++)
		{
			dr_circle ( ref.r[i]*pm_co[i],ref.r[i]*pm_si[i],4.0,"black" );
			dr_circle ( pm_fi[i]*PM_R2D,ref.r[i]/10.0-100,1,"black" );
		}
#endif

		act.rx = ax;act.ry = ay;act.th = ath;
		pm_scan_project(&act,  &new_r[0], &new_bad[0]);

		//---------------ORIENTATION SEARCH-----------------------------------
		//search for angle correction using crosscorrelation, perform it every second step
		if ( iter%2 == 0 )
		{
			dth = pm_orientation_search(&ref, &new_r[0], &new_bad[0]);
			ath += dth;
			continue;
		}

		//------------------------------------------translation-------------
		//reduce C with time to consider only the best matches
		if ( iter == PM_CHANGE_WEIGHT_ITER )
			C = C/50.0; // weigh far points even less.
#ifdef GR
		for(int i=0;i<PM_L_POINTS;i++)
		{
			dr_circle ( pm_fi[i]*PM_R2D,ref.r[i]/10.0-200,1,"black" );
		}
#endif

		avg_err = pm_translation_estimation(&ref, &new_r[0], &new_bad[0], C, &dx, &dy);
		ax += dx;
		ay += dy;

#ifdef GR
		cout <<"iter "<<iter<<" "<<ax<<" "<<ay<<" "<<ath*PM_R2D<<" "<<dx<<" "<<dy<<endl;
		dr_zoom();
#endif
	}//while iter

#ifdef  PM_GENERATE_RESULTS
	end_tick =pm_msec();
	fprintf ( f,"%i %lf %lf %lf %lf\n",iter,
		end_tick-start_tick-dead_tick ,ax,ay,ath*PM_R2D );
	fclose ( f );
#endif
	//dr_zoom();
	//cout <<"Iterations: "<<iter<<endl;
	lsa->rx =ax;lsa->ry=ay;lsa->th=ath;
	return ( avg_err);
}//pm_psm

/** @brief Estimate the postion of the current scan with respect to a reference scan.

@param ref The reference scan.
@param new_r The interpolated ranges of the current scan.
@param new_bad The tags corresponding to the new_r.
@param C Weighting factor for range residuals.
@param dx Estimated position increment X coordinate is returned here.
@param dy Estimated position increment Y coordinate is returned here.
@return Returns the average range residual.
*/
PM_TYPE CPolarMatch::pm_translation_estimation(const PMScan *ref, const PM_TYPE *new_r, const int *new_bad, PM_TYPE C, PM_TYPE *dx, PM_TYPE *dy)
{
	// do the weighted linear regression on the linearized ...
	// include angle as well
	int i;
	PM_TYPE hi1, hi2,hwi1,hwi2, hw1=0,hw2=0,hwh11=0;
	PM_TYPE hwh12=0,hwh21=0,hwh22=0,w;
	PM_TYPE dr;
	PM_TYPE abs_err = 0;
	int     n = 0;
	for ( i=0;i<m_pParam->pm_l_points;i++ )
	{
		dr = ref->r[i]-new_r[i];
		abs_err += fabsf ( dr );
		//weight calculation
		if ( ref->bad[i]==0 && new_bad[i]==0 && new_r[i]<m_pParam->pm_max_range && new_r[i]>PM_MIN_RANGE && fabsf ( dr ) <PM_MAX_ERROR )
		{
			//        cout <<i<<" "<<dr<<";"<<endl;
			//weighting according to DUDEK00
			w = C/ ( dr*dr+C );
			n++;

			//proper calculations of the jacobian
			hi1 = pm_co[i];//xx/new_r[i];//this the correct
			hi2 = pm_si[i];//yy/new_r[i];

			hwi1 = hi1*w;
			hwi2 = hi2*w;

			//par = (H^t*W*H)^-1*H^t*W*dr
			hw1 += hwi1*dr;//H^t*W*dr
			hw2 += hwi2*dr;

			//H^t*W*H
			hwh11 += hwi1*hi1;
			hwh12 += hwi1*hi2;
			//        hwh21 += hwi2*hi1; //should take adv. of symmetricity!!
			hwh22 += hwi2*hi2;

#ifdef GR
			//deb
			dr_circle ( pm_fi[i]*PM_R2D,new_r[i]/10.0-200,1,"red" );
			dr_circle ( pm_fi[i]*PM_R2D,dr/10.0-200,1,"blue" );
			//          cout <<i<<"\t"<<ref.r[i]<<"\t"<<new_r[i]<<"\t"<<dr<<"\t"<<w<<" "<<act.r[index[i]]
			//          <<" "<<pm_fi[index[i]]<<";"<<endl;
			{
				double x,y;
				x = pm_fi[i]*PM_R2D;
				y = new_r[i]/10.0-200;
				dr_line ( x,y,x+hwi1*dr,y+hwi2*dr,"red" );
				x = new_r[i]*pm_co[i];
				y = new_r[i]*pm_si[i];
				dr_line ( x,y,x+hwi1*dr,y+hwi2*dr,"red" );
			}
#endif

		}//if
	}//for i
	if ( n< m_pParam->pm_min_valid_points/*PM_MIN_VALID_POINTS*/ ) //are there enough points?
	{
		cerr <<"pm_translation_estimation: ERROR not enough points ("<<n<<")"<<endl;
#ifdef GR
		dr_zoom();
#endif
		throw 1;//not enough points
	}

	//calculation of inverse
	PM_TYPE D;//determinant
	PM_TYPE inv11,inv21,inv12,inv22;//inverse matrix

	D = hwh11*hwh22-hwh12*hwh21;
	if ( D<0.001 )
	{
		cerr <<"pm_linearized_match: ERROR determinant to small! "<<D<<endl;
		throw 1;
	}
	inv11 =  hwh22/D;
	inv12 = -hwh12/D;
	inv21 = -hwh12/D;
	inv22 =  hwh11/D;

	*dx = inv11*hw1+inv12*hw2;
	*dy = inv21*hw1+inv22*hw2;
	return(abs_err/n);
}//pm_translation_estimation

/** @brief Performs one iteration of orientation alignment of current scan.

Function estimating the orientation of the current scan represented with range readings
@a new_r tagged with flags @a new_bad with respect to the reference scan @a ref.

This function exploits that if the current and reference scan are taken at the same 
position, an orientation change of the current scan results in a left or right shift 
of the scan ranges.

This function estimates the orientation by finding that shift which minimizes the
difference between the current and ref. scan. The orientation estimate is then
refined using interpolation by fitting a parabole to the maximum and its 
neighbours and finding the maximum. 

@param ref The reference scan.
@param new_r The interpolated ranges of the current scan.
@param new_bad The tags corresponding to the new_r.
@return Returns the rotation of @new_bad in radians which minimize the sum of absolute range residuals.
*/
PM_TYPE CPolarMatch::pm_orientation_search(const PMScan *ref, const PM_TYPE *new_r, const int *new_bad)
{
	int       i;
	int       window = m_pParam->pm_scan_window;//PM_SEARCH_WINDOW;//20;//+- width of search for correct orientation
	PM_TYPE   dth = 0.0;//current scan corrections
	//pm_fi,ref.r - reference points
	PM_TYPE e;
	std::vector<PM_TYPE>  err(m_pParam->pm_l_points); // the error rating	
	std::vector<PM_TYPE> beta(m_pParam->pm_l_points);// angle corresponding to err
	
	const PM_TYPE LARGE_NUMBER = 10000;
	PM_TYPE n;
	int k=0;

	for ( int di=-window;di<=window;di++ )
	{
		n=0;e=0;

		int min_i,max_i;
		if ( di<=0 )
		{min_i = -di;max_i=m_pParam->pm_l_points;}
		else
		{min_i = 0;max_i=m_pParam->pm_l_points-di;}

		///TODO: speed up by unrolling the loop, replace if with multiplication with 0 or 1/
		/// use sse2 instructions...
		for ( i=min_i;i<max_i;i++ ) //searching through the current points
		{
			PM_TYPE delta = fabsf ( new_r[i]-ref->r[i+di] );
//#if PM_LASER == PM_HOKUYO_UTM_30LX
		if(m_pParam->pm_laser_name=="HOKUYO_UTM_30LX")
			//checking delta < PM_MAX_ERROR helps to choose the correct local minimum for the UTM.
			//Without it the solution may be pulled in the wrong direction. Don't remove it.
			///TODO: Find out when is it useful to check if delta < PM_MAX_ERROR - why only for the UTM...
		{	if ( !new_bad[i] && !ref->bad[i+di]  && delta < PM_MAX_ERROR)
			{ 
				e += delta;
				n++;
			}
		}
//#else
		else{
			if ( !new_bad[i] && !ref->bad[i+di] )
//#endif
			{ 
				e += delta;
				n++;
			}
		}
		}//for i

		if(i>=196 && di>=162){
			i=i;
			k=k;
		}

		if(k>65535){
			di=di;
			i=i;
		}

		if ( n > 0 )
			err[k]  = e/n;//don't forget to correct with n!
		else
			err[k]  = LARGE_NUMBER;
		beta[k] = di;
		k++;
	}//for dfi

#ifdef GR
	FILE *fo;
	fo = fopen ( "angles.txt","w" );
	for ( i = 0; i < k; i++ )
	{
		dr_circle ( beta[i],err[i],1.0,"blue" );
		fprintf ( fo,"%f %f\n",beta[i],err[i] );
	}
	fclose ( fo );
#endif

	//now search for the global minimum
	//later I can make it more robust
	//assumption: monomodal error function!
	PM_TYPE emin = LARGE_NUMBER*10.0;
	int   imin=-1;
	for ( i = 0; i < k; i++ )
	{
		if ( err[i] < emin )
		{
			emin = err[i];
			imin = i;
		}
	}

	if ( err[imin]>=LARGE_NUMBER )
	{
		cerr <<"Polar Match: orientation search failed" <<err[imin]<<endl;
		throw 1;
	}
	dth = beta[imin]*m_pParam->pm_dfi;

	//interpolation
	if ( imin >= 1 && imin < ( k-1 ) ) //is it not on the extreme?
	{
		//lets try interpolation
		PM_TYPE D = err[imin-1]+err[imin+1]-2.0*err[imin];
		PM_TYPE d = LARGE_NUMBER;
		if ( fabsf ( D ) >0.01 && err[imin-1]>err[imin] && err[imin+1]>err[imin] )
			d= ( err[imin-1]-err[imin+1] ) /D/2.0;
		//        cout <<"ORIENTATION REFINEMENT "<<d<<endl;
		if ( fabsf ( d ) < 1.0 )
			dth+=d*m_pParam->pm_dfi;
	}//if

#ifdef GR
	cout <<"angle correction[deg]: "<<dth*PM_R2D<<endl;
	dr_zoom();
#endif
	return(dth);
}//pm_orientation_search


/** @brief More quickly calculates an error index expressing the quality of a match.

This function assesses how well is the current scan aligned with the
reference scan. This function has to be called after a scan has been 
matched. The current scan's pose has to be expressed in the reference 
scan's coordinate system.

The current scan is compared to the reference scan by projecting the
current scan where the reference scan was taken and calculating the
average range residuals.

@param lsr The reference scan.
@param lra The current scan. 
@return The average minimum Euclidean distance.
*/
PM_TYPE CPolarMatch::pm_error_index2 ( PMScan *ref,PMScan *cur, int* associatedPoints )
{

	PMScan cur2(*cur);//copies of current and reference scans
	PM_TYPE   rx,ry,rth,ax,ay,ath;//robot pos at ref and current scans
	PM_TYPE   t13,t23,LASER_Y = PM_LASER_Y;
	vector<PM_TYPE>   new_r(m_pParam->pm_l_points);//interpolated r at measurement bearings
	vector<int>       new_bad(m_pParam->pm_l_points);//bad flags of the interpolated range readings 
	PM_TYPE   avg_err = 100000000.0;

	rx =  ref->rx; ry = ref->ry; rth = ref->th;
	ax =  cur->rx;  ay = cur->ry;  ath = cur->th;

	// Transformation of the current scan laser scanner coordinates into the reference
	// laser scanner's coordinate system:
	t13 = sinf ( rth-ath ) *LASER_Y+cosf ( rth ) *ax+sinf ( rth ) *ay-sinf ( rth ) *ry-rx*cosf ( rth );
	t23 = cosf ( rth-ath ) *LASER_Y-sinf ( rth ) *ax+cosf ( rth ) *ay-cosf ( rth ) *ry+rx*sinf ( rth )-LASER_Y;

	//cur2 = *cur;
	cur2.rx = t13; cur2.ry = t23; cur2.th = ath-rth;

	//from now on act.rx,.. express the laser's position in the reference frame
	pm_scan_project( &cur2,  &new_r[0], &new_bad[0] );

	PM_TYPE  e = 0;
	int n = 0;
	for ( int i=0;i < m_pParam->pm_l_points;i++ ) //searching through the current points
	{
		PM_TYPE delta = fabsf ( new_r[i] - ref->r[i] );
		if ( !new_bad[i] && !ref->bad[i] && delta < PM_MAX_ERROR / 2.0)
		{
			e += delta;
			n++;
		}
	}//for i

	if ( n > 0 )
		avg_err = e/n;
	if(associatedPoints != NULL)
	{
		*associatedPoints = n;
	} 
	return avg_err;
}


/** @brief Performs scan projection.

This function enables the comparisson of two scans.
It projects the current (active) scan @a act into the reference scans @a ref
coordinate frame,  using the current scan's pose. As the reference scan
is assumed to be at the origin, its coordinates don't need to be passed along.
Returns in new_r the interpolated range readinds r at the reference scan's
measurement bearings. Returns in new_bad bad flags of the interpolated range 
readings, where occluded readings are tagged.

@param act The current scan.
@param new_r Array of the projected range readings (has to have the correct size).
@param new_bad Information about the validity of the interpolated range readings is returned here.
*/
void CPolarMatch::pm_scan_project(const PMScan *act,  PM_TYPE   *new_r,  int *new_bad)
{
	vector<PM_TYPE> r(m_pParam->pm_l_points);//current scan in ref. coord. syst.
	vector<PM_TYPE> fi(m_pParam->pm_l_points);
	PM_TYPE   x,y;
	int       i;
	PM_TYPE   delta;

	// convert range readings into the reference frame
	// this can be speeded up, by connecting it with the interpolation
	for ( i=0;i<m_pParam->pm_l_points;i++ )
	{
		delta   = act->th + pm_fi[i];
		x       = act->r[i]*cosf ( delta ) + act->rx;
		y       = act->r[i]*sinf ( delta ) + act->ry;
		r[i]    = sqrtf ( x*x+y*y );
		fi[i]   = atan2f ( y,x );
		//handle discontinuity at pi (Angle goes from -pi/1 to 3pi/2 for 360deg. scans)
		if(x<0 && y<0)
			fi[i] += 2.0*M_PI;
		new_r[i]  = 10000;//initialize big interpolated r;
		new_bad[i]= PM_EMPTY;//for interpolated r;

#ifdef GR
		//      dr_circle ( ref.r[i]*pm_co[i],ref.r[i]*pm_si[i],4.0,"black" );
		dr_circle ( x,y,4.0,"red" );
		//      dr_circle ( pm_fi[i]*PM_R2D,ref.r[i]/10.0-100,1,"black" );
		dr_circle ( fi[i]*PM_R2D,r[i]/10.0-100,1,"red" );      
#endif
	}//for i

	//------------------------INTERPOLATION------------------------
	//calculate/interpolate the associations to the ref scan points
	//algorithm ignores crosings at the beginning and end points to make it faster
	for ( i=1;i<m_pParam->pm_l_points;i++ )
	{
		//i points to the angles in the current scan

		// i and i-1 has to be in the same segment, both shouldn't be bad
		// and they should be larger than the minimum angle      
		if ( act->seg[i] != 0 && act->seg[i] == act->seg[i-1] && !act->bad[i] && !act->bad[i-1] ) /* && fi[i]>PM_FI_MIN && fi[i-1]>PM_FI_MIN*/
		{
			//calculation of the "whole" parts of the angles
			int j0,j1;
			PM_TYPE r0,r1,a0,a1;
			bool occluded;
			//This is a crude hack to fix a serious bug here!!!! 
			//At the -pi pi boundary it failed by interpolating throught the whole scan.
			//The affected 360 scans, or Hokuyo scans where the matched scans had
			//more than 60degree orientation difference. 
			if( fabsf(fi[i]-fi[i-1]) >= M_PI ) ///TODO: replace this hack with proper fix where we don't loose points.
				continue;

			if ( fi[i]>fi[i-1] ) //are the points visible?
			{
				//visible
				occluded = false;
				a0  = fi[i-1];
				a1  = fi[i];
				j0  =  (int) ceil ( ( fi[i-1] - m_pParam->pm_fi_min/*PM_FI_MIN*/ ) / m_pParam->pm_dfi /*PM_DFI*/ );
				j1  =  (int) floor ( ( fi[i] - m_pParam->pm_fi_min/*PM_FI_MIN*/ ) / m_pParam->pm_dfi /*PM_DFI*/ );
				r0  = r[i-1];
				r1  = r[i];
			}
			else
			{
				//invisible - still have to calculate to filter out points which
				occluded = true; //are covered up by these!
				//flip the points-> easier to program
				a0  = fi[i];
				a1  = fi[i-1];
				j0  =  (int) ceil ( ( fi[i] - m_pParam->pm_fi_min/*PM_FI_MIN*/ ) / m_pParam->pm_dfi /*PM_DFI*/ );
				j1  =  (int) floor ( ( fi[i-1] - m_pParam->pm_fi_min/*PM_FI_MIN*/ ) / m_pParam->pm_dfi /*PM_DFI*/ );
				//j0  =  (int) ceil ( ( fi[i] - PM_FI_MIN ) /PM_DFI );
				//j1  =  (int) floor ( ( fi[i-1] - PM_FI_MIN ) /PM_DFI );
				r0  = r[i];
				r1  = r[i-1];
			}
			//here j0 is always smaller than j1!

			//interpolate for all the measurement bearings beween j0 and j1
			while ( j0<=j1 ) //if at least one measurement point difference, then ...
			{
				PM_TYPE ri = ( r1-r0 ) / ( a1-a0 ) * ( ( ( PM_TYPE ) j0*m_pParam->pm_dfi+m_pParam->pm_fi_min )-a0 ) +r0;

				//if j0 -> falls into the measurement range and ri is shorter
				//than the current range then overwrite it
				if ( j0>=0 && j0<m_pParam->pm_l_points && new_r[j0]>ri )
				{
					new_r[j0]    = ri;//overwrite the previous reading
					new_bad[j0] &=~PM_EMPTY;//clear the empty flag
					if ( occluded ) //check if it was occluded
						new_bad[j0] = new_bad[j0]|PM_OCCLUDED;//set the occluded flag
					else
						new_bad[j0] = new_bad[j0]&~PM_OCCLUDED;
					//the new range reading also has to inherit the other flags
					new_bad[j0] |= act->bad[i];//superfluos - since act.bad[i] was checked for 0
					new_bad[j0] |= act->bad[i-1];//superfluos - since act.bad[i-1] was checked for 0
					///TODO: Uncomment this? (or leave it as it is a local scan matching approach anyway)
					//if(ri>PM_MAX_RANGE)        //uncomment this later
					//  new_bad[fi0] |= PM_RANGE;
#ifdef GR
					dr_circle ( pm_fi[j0]*PM_R2D,new_r[j0]/10.0-100,1,"yellow" );
#endif
					//dr_zoom();
				}
				j0++;//check the next measurement angle!
			}//while
		}//if act
	}//for i

#ifdef GR
	//show the interpolated measurements:
	for ( i=0;i<PM_L_POINTS;i++ )
	{
		double x,y;
		x = new_r[i]*pm_co[i];
		y = new_r[i]*pm_si[i];
		dr_circle ( x,y,3,"yellow" );
		dr_circle ( fi[i]*PM_R2D,new_r[i]/10.0-100,1,"yellow" );
	}
	dr_zoom();
#endif
}//pm_scan_project


/** @brief Segments scanpoints into groups based on range discontinuities.

By segmenting scans into groups of disconnected sets of points, one can
prevent falsely interpolating points into the free space between disconnected
objects during scan projection.

Segment number 0 is reserved to segments containing only 1 point.

Far away points (r > PM_MAX_RANGE), gaps between groups of 
points - divide segments. The gap between extrapolated point and
current point has to be large as well to prevent corridor walls to
be segmented into separate points.
*/
void CPolarMatch::pm_segment_scan ( PMScan *ls )
{
	const float   MAX_DIST = PM_SEG_MAX_DIST;//max range diff between conseq. points in a seg
	PM_TYPE   dr;
	int       seg_cnt = 0;
	int       i,cnt;
	bool      break_seg;

	seg_cnt = 1;

	//init:
	if ( fabsf ( ls->r[0]-ls->r[1] ) < MAX_DIST ) //are they in the same segment?
	{
		ls->seg[0] = seg_cnt;
		ls->seg[1] = seg_cnt;
		cnt        = 2;    //2 points in the segment
	}
	else
	{
		ls->seg[0] = 0; //point is a segment in itself
		ls->seg[1] = seg_cnt;
		cnt        = 1;
	}

	for ( i=2;i<m_pParam->pm_l_points;i++ )
	{
		//segment breaking conditions: - bad point;
		break_seg = false;
		if ( ls->bad[i] )
		{
			break_seg = true;
			ls->seg[i] = 0;
		}
		else
		{
			dr = ls->r[i]- ( 2.0*ls->r[i-1] - ls->r[i-2] );//extrapolate & calc difference
			//Don't break a segment if the distance between points is small
			//or the distance beween the extrapolated point and current point is small.
			if ( fabsf ( ls->r[i]-ls->r[i-1] ) < MAX_DIST || 
				( ( ls->seg[i-1]==ls->seg[i-2] ) && fabsf ( dr ) <MAX_DIST ) )
			{
				//not breaking the segment
				cnt++;
				ls->seg[i] = seg_cnt;
			}
			else
				break_seg = true;
		}//if ls->bad

		if ( break_seg ) // breaking the segment?
		{
			if ( cnt==1 )
			{
				//check first if the last three are not on a line by coincidence
				dr = ls->r[i]- ( 2.0*ls->r[i-1]-ls->r[i-2] );
				if ( ls->seg[i-2] == 0 && ls->bad[i] == 0 && ls->bad[i-1] == 0
					&& ls->bad[i-2] == 0 && fabsf ( dr ) <MAX_DIST )
				{
					ls->seg[i]   = seg_cnt;
					ls->seg[i-1] = seg_cnt;
					ls->seg[i-2] = seg_cnt;
					cnt = 3;
				}//if ls->
				else
				{
					ls->seg[i-1] = 0;
					//what if ls[i] is a bad point? - it could be the start of a new
					//segment if the next point is a good point and is close enough!
					//in that case it doesn't really matters
					ls->seg[i] = seg_cnt;//the current point is a new segment
					cnt = 1;
				}
			}//if cnt ==1
			else
			{
				seg_cnt++;
				ls->seg[i] = seg_cnt;
				cnt = 1;
			}//else if cnt
		}//if break seg
	}//for
}//pm_segment_scan

/** @brief Tags point further than a given distance PM_MAX_RANGE.

Far away points get tagged as @a PM_RANGE.
@param ls The scan searched for far points.
*/
void CPolarMatch::pm_find_far_points ( PMScan *ls )
{
	for ( int i=0;i<m_pParam->pm_l_points;i++ )
	{
		if ( ls->r[i]>m_pParam->pm_max_range )
			ls->bad[i] |= PM_RANGE;
	}
}


/** @brief Filters the laser ranges with a median filter.

The job of this median filter is to remove chair and table 
legs which are likely to change position with time.

The median filter helps to get rid of spurious data.
If the median filter's window is 5, then 3 points need be 
close to each other to surrive the filtering. Chair legs taking
1 or 2 range readings will be removed.

Do not use this function when fitting lines to laser scans!

Median filter will round up corners.

x,y coordinates of points are not upadted.
@param ls Laser scan to be filtered.
*/
void CPolarMatch::pm_median_filter ( PMScan *ls )
{
	const int HALF_WINDOW  = 2;//2 to left 2 to right
	const int WINDOW = 2*HALF_WINDOW+1;
	float   r[WINDOW];
	float   w;

	int i,j,k,l;

	for ( i=0;i<this->m_pParam->pm_l_points;i++ )
	{
		k=0;
		for ( j=i-HALF_WINDOW;j<=i+HALF_WINDOW;j++ )
		{
			l = ( ( j>=0 ) ?j:0 );
			r[k]=ls->r[ ( ( l < m_pParam->pm_l_points ) ?l: ( m_pParam->pm_l_points-1 ) ) ];
			k++;
		}
		//bubble sort r
		for ( j= ( WINDOW-1 );j>0;j-- )
			for ( k=0;k<j;k++ )
				if ( r[k]>r[k+1] ) // wrong order? - swap them
				{
					w=r[k];
					r[k]=r[k+1];
					r[k+1] = w;
				}
				ls->r[i] = r[HALF_WINDOW];//choose the middle point
	}
}


// ----- ICP ------//

/** @brief Matches two laser scans using the iterative closest point method.

Minimizes least square error of points through changing lsa->rx, lsa->ry, lsa->th
by using ICP. It interpolates associated 
points. Only the best 80% of points are used in the pose calculation.
Scan projection is done at each iteration.

For maintanence reasons changed scan projection to that of psm.
*/
PM_TYPE CPolarMatch::pm_icp (  const PMScan *lsr,PMScan *lsa )
{
#define INTERPOLATE_ICP  //comment out if no interpolation of ref. scan points iS  necessary
	//PMScan    act,  ref;//copies of current and reference scans
	PMScan act(*lsa);
	PMScan ref(*lsr);

	PM_TYPE   rx,ry,rth,ax,ay,ath;//robot pos at ref and current scans
	PM_TYPE   t13,t23,LASER_Y = PM_LASER_Y;
	// int       new_bad[PM_L_POINTS];//bad flags of the projected current scan range readings
	// PM_TYPE   new_r[PM_L_POINTS];//ranges of current scan projected into ref. frame for occlusion check
	// PM_TYPE   nx[PM_L_POINTS];//current scanpoints in ref coord system
	// PM_TYPE   ny[PM_L_POINTS];//current scanpoints in ref coord system
	// int       index[PM_L_POINTS][2];//match indices current,refernce
	// PM_TYPE   dist[PM_L_POINTS];// distance for the matches

	vector<int> new_bad(m_pParam->pm_l_points);
	vector<PM_TYPE> new_r(m_pParam->pm_l_points);
	vector<PM_TYPE> nx(m_pParam->pm_l_points);
	vector<PM_TYPE> ny(m_pParam->pm_l_points);
	vector< vector<int> > index(m_pParam->pm_l_points, vector<int>(2) );
	vector<PM_TYPE> dist(m_pParam->pm_l_points);

	int       n = 0;//number of valid points
	int       iter,i,j,small_corr_cnt=0,k,imax;
	int       window       =  m_pParam->pm_scan_window; // PM_SEARCH_WINDOW;//+- width of search for correct orientation
	PM_TYPE   abs_err=0,dx=0,dy=0,dth=0;//match error, current scan corrections
	PM_TYPE   co,si;

#ifdef  PM_GENERATE_RESULTS
	double start_tick, dead_tick,end_tick,end_tick2;
	FILE *f;
	f = fopen ( PM_TIME_FILE,"w" );
	dead_tick = 0;
	start_tick =pm_msec();
#endif

	// act = *lsa;
	// ref = *lsr;

	rx =  ref.rx; ry = ref.ry; rth = ref.th;
	ax =  act.rx; ay = act.ry; ath = act.th;

	//transformation of current scan laser scanner coordinates into reference
	//laser scanner coordinates
	t13 = sinf ( rth-ath ) *LASER_Y+cosf ( rth ) *ax+sinf ( rth ) *ay-sinf ( rth ) *ry-rx*cosf ( rth );
	t23 = cosf ( rth-ath ) *LASER_Y-sinf ( rth ) *ax+cosf ( rth ) *ay-cosf ( rth ) *ry+rx*sinf ( rth )-LASER_Y;

	ref.rx = 0;   ref.ry = 0;   ref.th = 0;
	act.rx = t13; act.ry = t23; act.th = ath-rth;

	ax = act.rx; ay = act.ry; ath = act.th;
	//from now on act.rx,.. express the lasers position in the ref frame

	//intializing x,y of act and ref
	for ( i=0;i< m_pParam->pm_l_points /*PM_L_POINTS*/;i++ )
	{
		ref.x[i] = ref.r[i]*pm_co[i];
		ref.y[i] = ref.r[i]*pm_si[i];

		act.x[i] = act.r[i]*pm_co[i];
		act.y[i] = act.r[i]*pm_si[i];
	}//for i

	iter = -1;
	while ( ++iter<PM_MAX_ITER_ICP && small_corr_cnt<3 ) //have to be a few small corrections before stop
	{

		if ( ( fabsf ( dx ) +fabsf ( dy ) +fabsf ( dth ) *PM_R2D ) <PM_STOP_COND_ICP )
			small_corr_cnt++;
		else
			small_corr_cnt=0;

#ifdef  PM_GENERATE_RESULTS
		end_tick =pm_msec();
		fprintf ( f,"%i %lf %lf %lf %lf\n",iter,
			end_tick-start_tick-dead_tick ,ax,ay,ath*PM_R2D );
		end_tick2 =pm_msec();
		dead_tick += end_tick2- end_tick;
#endif

#ifdef GR
		dr_erase();
		dr_circle ( ax,ay,5.0,"green" );
		dr_line ( 0,-100,200,-100,"black" );
		dr_line ( 0,-200,200,-200,"black" );
#endif

		//Scan projection
		act.rx = ax;act.ry = ay;act.th = ath;
		pm_scan_project(&act,  &new_r[0], &new_bad[0]);

		// transformation the cartesian coordinates of the points:
		co = cosf ( ath );
		si = sinf ( ath );    
		for ( i=0;i<m_pParam->pm_l_points/*PM_L_POINTS*/;i++ )
		{
			nx[i]     = act.x[i]*co - act.y[i]*si + ax;
			ny[i]     = act.x[i]*si + act.y[i]*co + ay;
#ifdef GR
			if ( ref.bad[i] )
				dr_circle ( ref.x[i],ref.y[i],4,"yellow" );
			else
				dr_circle ( ref.x[i],ref.y[i],4,"black" );
			if ( new_bad[i] )
				dr_circle ( nx[i],ny[i],4,"green" );
			else
				dr_circle ( nx[i],ny[i],4,"blue" );
#endif
		}
		//    dr_zoom();
#ifdef GR
		cout <<"interpolated ranges. press enter"<<endl;
		/*    for ( i=0;i<PM_L_POINTS;i++ )
		dr_circle ( new_r[i]*pm_co[i],new_r[i]*pm_si[i],6,"red" );*/
		dr_zoom();
#endif

		//Correspondence search: go through the points of the current
		//scan and find the closest point in the reference scan 
		//lying withing a search interval. 
		n=0;
		PM_TYPE d,min_d;
		int min_idx;

		for ( i=0;i<m_pParam->pm_l_points/*PM_L_POINTS*/;i++ )
		{
			min_d = 1000000;
			min_idx = -1;
			if ( !new_bad[i] )
			{
				int imin,imax;
				imin = i-window ;
				if ( imin<0 )
					imin =0;
				imax = i+window ;
				if ( imax>m_pParam->pm_l_points/*PM_L_POINTS*/ )
					imax =m_pParam->pm_l_points;//PM_L_POINTS;

				for ( j=imin;j<imax;j++ )
				{
					if ( !ref.bad[j] )
					{
						d =  SQ ( nx[i]-ref.x[j] ) + SQ ( ny[i]-ref.y[j] );//square distance
						if ( d<min_d )
						{
							min_d  = d;
							min_idx = j;
						}
					}
				}//for
				if ( min_idx>=0 && sqrtf ( min_d ) <PM_MAX_ERROR ) // was there any match closer than 1m?
				{
					index[n][0] = i;
					index[n][1] = min_idx;
					dist[n] = sqrtf ( min_d );
					n++;
#ifdef GR
					dr_line ( nx[i],ny[i],ref.x[min_idx],ref.y[min_idx],"blue" );
#endif
				}
			}//if
		}//for
		//    dr_zoom();

		if ( n< m_pParam->pm_min_valid_points/*PM_MIN_VALID_POINTS*/ )
		{
			cerr <<"pm_icp: ERROR not enough points"<<endl;
#ifdef  PM_GENERATE_RESULTS
			fclose ( f );
#endif
			throw 1;
		}

		//sort the matches with bubble sort
		//put the largest 20 percent to the end
		imax = ( int ) ( ( double ) n*0.2 );
		for ( i=0;i<imax;i++ )
			for ( j=1;j< ( n-i );j++ )
			{
				if ( dist[j]<dist[j-1] ) //are they in the wrong order?
				{
					//swap them
					k             = index[j][0];
					index[j][0]   = index[j-1][0];
					index[j-1][0] = k;

					k             = index[j][1];
					index[j][1]   = index[j-1][1];
					index[j-1][1] = k;

					d             = dist[j];
					dist[j]       = dist[j-1];
					dist[j-1]     = d;
				}
			}//for j

#ifdef INTERPOLATE_ICP
			//------------------------INTERPOLATION---------------------------
			//comment out if not necessary
			//PM_TYPE ix[PM_L_POINTS],iy[PM_L_POINTS];//interp. ref. points.
			vector<PM_TYPE> ix(m_pParam->pm_l_points);
			vector<PM_TYPE> iy(m_pParam->pm_l_points);

			//replace nx,xy with their interpolated... where suitable
			{

				PM_TYPE d0,d1,d2;
				PM_TYPE minx1,miny1,minx2,miny2;
				int max_i = n-imax;
				for ( i=0;i<max_i;i++ )
				{

					//d1 = point_line_distance(1, 2,2,3, 2,2, &minx1, &miny1);  //debug

#ifdef GR
					dr_circle ( nx[index[i][0]],ny[index[i][0]],1.0,"brown" );
					dr_circle ( ref.x[index[i][1]],ref.y[index[i][1]],1.0,"brown" );
					dr_circle ( ref.x[index[i][1]-1],ref.y[index[i][1]-1],1.0,"brown" );
					dr_circle ( ref.x[index[i][1]+1],ref.y[index[i][1]+1],1.0,"brown" );
#endif
					d1=-1;d2=-1;
					if ( index[i][1]>0 ) //not associated to the first point?
					{
						d1 = point_line_distance ( ref.x[index[i][1]-1], ref.y[index[i][1]-1],
							ref.x[index[i][1]],   ref.y[index[i][1]],
							nx[index[i][0]],      ny[index[i][0]],
							&minx1, &miny1 );
					}

					if ( index[i][1]< (/*PM_L_POINTS*/ m_pParam->pm_l_points-1 ) ) //not associated to the last point?
					{
						d2 = point_line_distance ( ref.x[index[i][1]],  ref.y[index[i][1]],
							ref.x[index[i][1]+1],ref.y[index[i][1]+1],
							nx[index[i][0]],     ny[index[i][0]],
							&minx2, &miny2 );
					}

					ix[index[i][1]] = ref.x[index[i][1]];
					iy[index[i][1]] = ref.y[index[i][1]];
					d0 = sqrtf ( SQ ( ref.x[index[i][1]]-nx[index[i][0]] ) + SQ ( ref.y[index[i][1]]-ny[index[i][0]] ) );

					//is the first point closer?
					if ( d1>0 && d1<d0 )
					{
						ix[index[i][1]] = minx1;
						iy[index[i][1]] = miny1;
						d0 = d1;
					}

					//is the second point closer?
					if ( d2>0 && d2<d0 )
					{
						ix[index[i][1]] = minx2;
						iy[index[i][1]] = miny2;
					}
#ifdef GR
					dr_line ( nx[index[i][0]],ny[index[i][0]],ix[index[i][1]],iy[index[i][1]],"green" );
#endif
				}//for
			}
#endif


			//pose estimation
			//------------------------------------------translation-------------

			// do the weighted linear regression on the linearized ...
			// include angle as well
			//computation of the new dx1,dy1,dtheta1
			PM_TYPE sxx=0,sxy=0,syx=0,syy=0;
			PM_TYPE meanpx,meanpy,meanppx,meanppy;
			meanpx = 0;meanpy = 0;
			meanppx= 0;meanppy= 0;

			abs_err=0;
			imax = n-imax;
			for ( i=0;i<imax;i++ )
			{
				//weight calculation
				// do the cartesian calculations....
				meanpx +=  nx[index[i][0]];
				meanpy +=  ny[index[i][0]];

#ifdef INTERPOLATE_ICP
				meanppx +=  ix[index[i][1]];
				meanppy +=  iy[index[i][1]];
#else
				meanppx +=  ref.x[index[i][1]];
				meanppy +=  ref.y[index[i][1]];
#endif

#ifdef GR
				dr_line ( nx[index[i][0]],ny[index[i][0]],ref.x[index[i][1]],ref.y[index[i][1]],"red" );
#endif
			}//for
			meanpx /= imax;
			meanpy /= imax;

			meanppx /= imax;
			meanppy /= imax;

			for ( int i=0;i<imax;i++ )
			{
#ifdef INTERPOLATE_ICP
				sxx += ( nx[index[i][0]] - meanpx ) * ( ix[index[i][1]] - meanppx );
				sxy += ( nx[index[i][0]] - meanpx ) * ( iy[index[i][1]] - meanppy );
				syx += ( ny[index[i][0]] - meanpy ) * ( ix[index[i][1]] - meanppx );
				syy += ( ny[index[i][0]] - meanpy ) * ( iy[index[i][1]] - meanppy );
#else
				sxx += ( nx[index[i][0]] - meanpx ) * ( ref.x[index[i][1]] - meanppx );
				sxy += ( nx[index[i][0]] - meanpx ) * ( ref.y[index[i][1]] - meanppy );
				syx += ( ny[index[i][0]] - meanpy ) * ( ref.x[index[i][1]] - meanppx );
				syy += ( ny[index[i][0]] - meanpy ) * ( ref.y[index[i][1]] - meanppy );
#endif
			}
			//computation of the resulting translation and rotation
			//for method closest point match
			dth = atan2f ( sxy-syx,sxx+syy );
			dx  = meanppx - ax - ( cosf ( dth ) * ( meanpx- ax ) - sinf ( dth ) * ( meanpy - ay ) );
			dy  = meanppy - ay - ( sinf ( dth ) * ( meanpx- ax ) + cosf ( dth ) * ( meanpy - ay ) );

			ax += dx;
			ay += dy;
			ath+= dth;
			ath = norm_a ( ath );

			//    //for SIMULATION iteration results..
			//    cout <<iter<<"     "<<ax<<"    "<<ay<<"    "<<ath*PM_R2D<<" ;"<<endl;
#ifdef GR
			cout <<"iter "<<iter<<" "<<ax<<" "<<ay<<" "<<ath*PM_R2D<<" "<<dx<<" "<<dy<<endl;
			//      if(iter==0)
			dr_zoom();
			//usleep ( 10000 );

#endif

	}//for iter
	//cout <<iter<<endl;
#ifdef  PM_GENERATE_RESULTS
	end_tick =pm_msec();
	fprintf ( f,"%i %lf %lf %lf %lf\n",iter,
		end_tick-start_tick-dead_tick ,ax,ay,ath*PM_R2D );
	fclose ( f );
#endif

	lsa->rx =ax;lsa->ry=ay;lsa->th=ath;
	return ( abs_err/n );
}//pm_icp

/** @brief Calculate the distance of a point from a line section. 

Calculates the distance of the point (x3,y3) from a line defined by (x1,y1)
and (x2,y2). Returns the distance to the line or -1 if the
projection of (x3,y3) falls outside the line segment defined by (x1,y1)
and (x2,y2). The projection of (x3,y3) onto the line is also returned in x,y.
This function is used in ICP.
@param x1,y1 The start point of the line section.
@param x2,y2 The end point of the line section.
@param x3,y3 The point of which distance it sought.
@param x,y (x3,y3) projected onto the line segment is returned here.
@return The distance from the line or -1 if the projection falls outside of the line segment.
*/
PM_TYPE CPolarMatch::point_line_distance ( PM_TYPE x1, PM_TYPE y1, PM_TYPE x2, PM_TYPE y2,
							 PM_TYPE x3, PM_TYPE y3,PM_TYPE *x, PM_TYPE *y )
{
	PM_TYPE ax,ay,t1,D;
	ax = x2-x1;
	ay = y2-y1;
	D =  sqrtf ( ax*ax+ay*ay );
	if ( D < 0.0001 )
	{
		cerr <<"point_line_distance: unexpected D:" << D <<endl;
		return -1;
	}
	t1 =  - ( -ax*x3 + ay*y1 + ax*x1 - ay*y3 ) / ( ax*ax+ay*ay );
	if ( t1<0 || t1>1 )   // Projection falls outside the line segment?
	{
		return -1;
	}
	*x = x1+t1*ax;
	*y = y1+t1*ay;
	return ( sqrtf ( ( x3-*x ) * ( x3-*x ) + ( y3-*y ) * ( y3-*y ) ) );//distance of line to p.
}//  point_line_distance