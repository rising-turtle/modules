#ifndef ICPSLAM_H
#define ICPSLAM_H

#include "preheader.h"

#define MAX_LASER_NUM 180
#define MAX_MEASURE_RANGE 50 // 50 meters

typedef struct _Scan{
	double timestamp;
	float range[MAX_LASER_NUM];
}Laser_Scan;

class CICPSlam
{
public:
	CICPSlam();
	~CICPSlam();
public:
	bool readfile(string infile);
public:
	vector<Laser_Scan> m_scan;
protected:
private:
};


#endif