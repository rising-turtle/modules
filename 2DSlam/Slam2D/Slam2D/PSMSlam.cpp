#include "preheader.h"
#include "PSMSlam.h"
#include ".\psm\polar_match.h"
#include <cmath>
#include <windows.h>


namespace{
	#define SQ(x) ((x)*(x))
	#define pi 3.141592654
    #define PM_D2R pi/180.0
	#define PM_R2D 180.0/pi
};

CPSMSlam::CPSMSlam(){
	float step_angle = PM_DFI*D2R;//pi/362.0;//
	m_pm_angle[0]=0;
	m_pm_cos[0] = cosf(0);
	m_pm_sin[0] = sinf(0);
	for(int i=1;i<MAX_LASER_NUM;i++)
	{
		m_pm_angle[i]=m_pm_angle[i-1]+step_angle;
		m_pm_cos[i]=cosf(m_pm_angle[i]);
		m_pm_sin[i]=sinf(m_pm_angle[i]);
	}
}
CPSMSlam::~CPSMSlam(){
	for(int i=0;i<m_Scan.size();i++)
	{
		delete m_Scan[i];
		m_Scan[i]=NULL;
	}
}

void CPSMSlam::testBug(){

	//char* f1="D:\\myproj\\2DSlam\\Slam2D\\Slam2D\\psm\\hokuyo_utm_30lx_from_Mohsen_Akbaritabar.txt";
	char* f1="D:\\myproj\\2DSlam\\Slam2D\\Slam2D\\psm\\sick_lms200.log";
	FILE* fin;
	//pm_init(f1,&fin);  
	//PMScan ls,ls_last;
	//int e1=read_scan4(fin,&ls_last);
	//int e2=read_scan4(fin,&ls);
 //	//int e1 =read_scan5(fin,&ls_last);
	////int e2 = read_scan5(fin,&ls);
	//if(e1 || e2){
	//	printf("laser reader failed!\n");
	//	return;
	//}
	// pm_preprocessScan(&ls);
	// pm_preprocessScan(&ls_last);
	// ls.rx = 0;ls.ry=0;ls.th=0;//matching without prior info
	// ls_last.rx = 0;ls_last.ry=0;ls_last.th=0;//matching without prior info
	// pm_psm(&ls_last,&ls);

	 string f2("D:\\myproj\\2DSlam\\Slam2D\\Slam2D\\out.txt");
	read_SICK511(f2);
	if(m_Scan.size()<2){
		cout<<"error to read scan!"<<endl;
		return ;
	}
	pm_bearing_init(m_pm_angle);

	pPMScan ls,ls_last;
	ls_last = m_Scan[0];
	ls=m_Scan[1];
	pm_preprocessScan(ls_last);	
	pm_preprocessScan(ls);

	//pm_icp(ls_last,ls);
	pm_psm(ls_last,ls);
}

int CPSMSlam::read_scan_sick(FILE* file, pPMScan ls){
		int n=0;

		ls->rx= ls->ry =ls->th = 0;
		ls->t = -1;

		char s[1024];
		fscanf(file,"%s",s);
		int pcnt ;
		fscanf(file,"%i",&pcnt);
		
		for(int i=0;i<MAX_LASER_NUM; i++){
			n+=fscanf(file,"%f",&(ls->r[i]));
			ls->r[i] = ls->r[i] * 100.0;
			ls->x[i] = (ls->r[i])*m_pm_cos[i];
			ls->y[i] = (ls->r[i])*m_pm_sin[i];
			
			ls->bad[i] = 0;
			if(ls->r[i]<10) 
				ls->r[i] = 10000;
		}
		if(n != MAX_LASER_NUM)
			return -1;
		return 0;
}

void CPSMSlam::read_SICK511(string filename){
	ifstream ifile(filename.c_str());
	if(!ifile.is_open())
	{
		cout<<"failed to open file: "<<filename<<endl;
		return ;
	}
	char line[8192];
	double timestamp;
	int N;
	pPMScan tmpScan=new struct PMScan;
	tmpScan->rx=0;
	tmpScan->ry=0;
	tmpScan->th=0;
	tmpScan->t=0;
	//float step_angle = 0.5*D2R;
	//// init for bearing angle
	//tmpScan.bearing[0] = 0;
	//for(int i=1;i<MAX_LASER_NUM;i++){
	//	tmpScan.bearing[i]=tmpScan.bearing[i-1]+step_angle;
	//}
	while(ifile.getline(line,8192)){
		strtok(line," ");
		//strtok(NULL," ");	// filter timestamp
		N = (int)(atof(strtok(NULL," ")));
		if(N>MAX_LASER_NUM)
			N=MAX_LASER_NUM;
		for(int i=0;i<N;i++){ 
			tmpScan->r[i] = atof(strtok(NULL," "))*100.0;
			tmpScan->x[i] = (tmpScan->r[i])*m_pm_cos[i];
			tmpScan->y[i] = (tmpScan->r[i])*m_pm_sin[i];
			if(tmpScan->r[i]<10){
				tmpScan->r[i]=10000;
			}
			tmpScan->bad[i]=0;
			//tmpScan->seg[i] = 0;
		}
		m_Scan.push_back(new struct PMScan(*tmpScan));
	}
}

pPMScan CPSMSlam::getframe(vector<float> input_scan){
	
	int Scan_Size = input_scan.size();
	if(Scan_Size>MAX_LASER_NUM)
	{
		Scan_Size=MAX_LASER_NUM;
		cout<<"input frame exceed scan number!"<<endl;
	}
	pPMScan cur_scan = new struct PMScan;
	for(int i=0;i<Scan_Size;i++){
		cur_scan->r[i] = input_scan[i]*100; // m->cm
		cur_scan->x[i] = input_scan[i]*m_pm_cos[i];
		cur_scan->y[i] = input_scan[i]*m_pm_sin[i];
		if(cur_scan->r[i] < 2 || cur_scan->r[i] > 8000) // <2cm or >8m
		{
			cur_scan->bad[i] = 1;
			cur_scan->r[i]=10000;
		}
		else
			cur_scan->bad[i]= 0 ;
		cur_scan->seg[i]=0;
	}
	return cur_scan;
}

void CPSMSlam::runFlirtData(string filename)
{
	// open the file
	//char filename[100]={0};
	//const char* filename=filename.c_str();
	FILE *fin=fopen(filename.c_str(),"rb");
	if(fin==NULL){
		cout<<"failed to open file: "<<filename<<endl;
		return ;
	}
	//pm_init(filename,&fin);
	
	pm_bearing_init(m_pm_angle);

	int startPoint =0;
	struct PMScan ls,ls_last;
	double rx=0,ry=0,th=0;//robot starts at 0,0,0

	bool bFirst = true;
	int cnt = 0;
	int error = 0;
	while(!error){
		//error = read_scan4(fin,&ls);'
		error = read_carmenLog(fin,&ls);
		if(error)
		{
			printf("laser_odometry_example: reading scan failed.\n");
			continue;
		}
		cnt++;
		cout<<cnt<<endl;
		if(cnt < startPoint)
		{
			continue;
		}
		//preprocess the scan...
		pm_preprocessScan(&ls);
		// the first frame
		if(bFirst)    
		{
			bFirst = false;
			ls_last = ls;
		}
		else
		{
			//match against the last scan
			try{
				//match the last agains the current
				ls.rx = 0;ls.ry=0;ls.th=0;//matching without prior info
				ls_last.rx = 0;ls_last.ry=0;ls_last.th=0;//matching without prior info

				pm_psm(&ls_last,&ls);        

				//convert the match result into global pose
				double xx,yy,tth;

				xx = ls.rx*cos(th) - ls.ry*sin(th) + rx;
				yy = ls.rx*sin(th) + ls.ry*cos(th) + ry;
				tth= th + ls.th;

				rx = xx;
				ry = yy;
				th = tth;
				ls.rx = 0;ls.ry=0;ls.th=0;     
				ls_last = ls;                
			}catch(int err){
				cerr<<"error_caught:run()"<<endl;
			};
		} //else
	}//while
	fclose(fin);
	printf("\nSucceed!\n");
	return ;
}

// run psm matching methods without odometry
void CPSMSlam::runSick511LogImproved(string filename){
	 read_SICK511(filename);
	 if(m_Scan.size()<=2){
		 cout<<"error in readSICK511()!"<<endl;
		 return ;
	 }
	
	 ofstream trajectory("d://exprdata//rpsm.log");

	 // Initialize angles used in psm
	 pm_bearing_init(m_pm_angle);
	 //FILE* fin;
	 //pm_init(filename.c_str(),&fin);

	double rx=0,ry=0,th=0; // robot position
	double xx_last=0,yy_last=0,tth_last=0;
	double slam_t; // record the time consuming during slam process

	 PMScan ls_last,ls,ls_ref;
	 bool bFirst=true;
	 for(int i=0;i<m_Scan.size();i++){
		 ls=*m_Scan[i];
	 /*int error = 0;
	 while(!error){
		 error = read_scan_sick(fin,&ls);
		 if(error) {
			cout<<"failed to read scan !"<<endl;
			continue;
		 }*/
		double s_t = ::GetTickCount();
		pm_preprocessScan(&ls);
		if(bFirst){
			bFirst=false;
			ls.rx=0; ls.ry=0; ls.th = 0;
			ls_last = ls;
			ls_ref = ls;
		}else{
			// matching using the previous result as prior info
			ls.rx = ls_last.rx; 
			ls.ry = ls_last.ry;
			ls.th=ls_last.th;
			// matching without prior info
			ls_ref.rx = 0; ls_ref.ry=0; ls_ref.th=0;

			bool MatchingFailed = false;
			try{
				pm_psm(&ls_ref,&ls);
			}catch(int err){
				cout<<"err: "<<err<<" has occured!"<<endl;
				MatchingFailed = true;
			}
			PM_TYPE err_idx = pm_error_index2(&ls_last,&ls);
			bool refSwitching = false;
			if(MatchingFailed || err_idx > 5.0){
				cout<<"switch reference scan!"<<endl;
				refSwitching = true;
				ls_ref = ls_last;
				ls.rx = 0; ls.ry = 0; ls.th = 0;
				ls_ref.rx = 0; ls_ref.ry=0; ls_ref.th = 0;
				pm_psm(&ls_ref,&ls);
				rx = xx_last;
				ry = yy_last;
				th = tth_last;
				ls_ref = ls_last;
			}
			slam_t = ::GetTickCount() - s_t;
			// convert the match result into global pose
			double xx,yy,tth;
			xx = ls.rx*cos(th) - ls.ry*sin(th) + rx;
			yy = ls.rx*sin(th) + ls.ry*cos(th) + ry;
			tth = th + ls.th;
			
			trajectory<<slam_t<<"\t"<<xx<<"\t"<<yy<<"\t"<<th<<endl;

			//ls.rx = xx; ls.ry = yy; ls.th = tth;
			ls.rx = 0; ls.ry = 0; ls.th=0;
			ls_last = ls;
			xx_last = xx;
			yy_last = yy;
			tth_last = tth;
		}
	 }
	trajectory.close();
	cout<<"Finished!"<<endl;
	return ;
}

void CPSMSlam::runSick511Log(string filename){
	read_SICK511(filename);
	if(m_Scan.size()<=2){
		cout<<"error in readSICK511()!"<<endl;
		return ;
	}
	pm_bearing_init(m_pm_angle);

	double rx=0,ry=0,th=0;//robot starts at 0,0,0
	pm_preprocessScan(m_Scan[0]);

	ofstream outf("d://exprdata//psm.log");

	for(int i=0;i<m_Scan.size()-1;i++){
		int j=i+1;
		pPMScan ls, ls_last;
		ls = m_Scan[j];
		ls_last = m_Scan[i];

		ls_last->rx=ls_last->ry=ls_last->th=0; // match without prior information
		ls->rx = ls->ry = ls->th = 0; // match without prior information
		pm_preprocessScan(ls);
		//pm_psm(ls_last,ls);        
		double st=::GetTickCount();
		//pm_icp(ls_last,ls);
		pm_psm(ls_last,ls);
		double slamt=::GetTickCount()-st;

		// the obtained pose is relative to the reference of ls_last      
		//convert the match result into global pose
		double xx,yy,tth;

		xx = ls->rx*cos(th) - ls->ry*sin(th) + rx;
		yy = ls->rx*sin(th) + ls->ry*cos(th) + ry;
		tth= th + ls->th;

		rx = xx;
		ry = yy;
		th = tth;
		
		//cout<<"robot arrives at pose (x,y,th) = ("<<rx<<","<<ry<<","<<th<<")"<<endl;
		outf<<slamt<<"\t"<<rx<<"\t"<<ry<<"\t"<<th<<endl;

		//ls->rx = rx; 
		//ls->ry = ry;
		//ls->th = tth;
	}
	outf.close();
}


void CPSMSlam::run(string fdata){
	// open the file
	//char filename[100]={0};
	const char* filename=fdata.c_str();
	FILE *fin;
	pm_init(filename,&fin);

	int startPoint =0;
    struct PMScan ls,ls_last;
	double rx=0,ry=0,th=0;//robot starts at 0,0,0

	bool bFirst = true;
	int cnt = 0;
	int error = 0;
	while(!error){
		error = read_scan4(fin,&ls);
		if(error)
		{
			printf("laser_odometry_example: reading scan failed.\n");
			continue;
		}
		cnt++;
		if(cnt < startPoint)
		{
			continue;
		}
		cout <<endl<<cnt<<" t: "<<ls.t<< " ";

		//preprocess the scan...
		pm_preprocessScan(&ls);
		// the first frame
		if(bFirst)    
		{
			bFirst = false;
			ls_last = ls;
		}
		else
		{
			//match against the last scan
			try{
				//match the last agains the current
				ls.rx = 0;ls.ry=0;ls.th=0;//matching without prior info
				ls_last.rx = 0;ls_last.ry=0;ls_last.th=0;//matching without prior info

				pm_psm(&ls_last,&ls);        

				//convert the match result into global pose
				double xx,yy,tth;

				xx = ls.rx*cos(th) - ls.ry*sin(th) + rx;
				yy = ls.rx*sin(th) + ls.ry*cos(th) + ry;
				tth= th + ls.th;

				rx = xx;
				ry = yy;
				th = tth;
				ls.rx = 0;ls.ry=0;ls.th=0;     
				ls_last = ls;                
			}catch(int err){
				cerr<<"error_caught:run()"<<endl;
			};
		} //else
	}//while
	fclose(fin);
	printf("\nSucceed!\n");
	return ;
}

void CPSMSlam::runtest(){
	const PM_TYPE eps = 0.5; //allowed error in X or Y coordinate
	const PM_TYPE epsTh = 0.5*PM_D2R;//allowed orientation error
	
	pm_init();
	PMScan lsc;//Current scan
	PMScan lsr;//Reference scan.
	float xc,yc,thc; 
	float xr,yr,thr;
	int test = -1;  
	
	PM_TYPE err;
	double c1,c2,c3,c4;
	pm_cov_est(err,&c1,&c2,&c3,&c4);
	pm_save_scan(&lsr,"savename");

	xr=0.0; yr=0.0; thr=0.0; 
	pm_take_simulated_scan(xr, yr, thr, &lsr);
	pm_preprocessScan( &lsr );
	
	xc=10.0; yc=0; thc=0.1;  test++;
	pm_take_simulated_scan(xc, yc, thc, &lsc);  
	lsc.rx=0.0; lsc.ry=0.0; lsc.th=0.0;  
	pm_preprocessScan( &lsc );

	 pm_psm(&lsr, &lsc);
	 printf("Test%i:init. pose:(%.1f,%.1f,%.1f); position error:%.2f[cm] orient. error:%.2f[deg]\n",
		 test,xc,yc,thc*PM_R2D,sqrtf(SQ(xc - lsc.rx)+SQ(yc - lsc.ry)),(thc- lsc.th)*PM_R2D);    
	printf("finished!\n");
}