#include "ICPSlam.h"

CICPSlam::CICPSlam(){}
CICPSlam::~CICPSlam(){}
bool CICPSlam::readfile(string infile){
	ifstream input_file(infile.c_str());
	if(!input_file.is_open()){
		cout<<"failed to open file : "<<infile<<endl;
		return false;
	}
	char line[4096];
	Laser_Scan tmpScan;
	int N;
	while(input_file.getline(line,4096)){
		strtok(line," ");	// This is to split the line 
		tmpScan.timestamp=atof(strtok(NULL," ")); // timestamp of this record
		N=(int)(atof(strtok(NULL," ")));	// total number of laser
		if(N>MAX_LASER_NUM)
			N = MAX_LASER_NUM;
		for(int i=0;i<N;i++){
			tmpScan.range[i] = atof(strtok(NULL," "));
			if(tmpScan.range[i]>=MAX_MEASURE_RANGE) // invalid point
				tmpScan.range[i]=-1.0;
		}
		m_scan.push_back(tmpScan);
	}
}

