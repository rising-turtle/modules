#ifndef PSMSLAM_H
#define PSMSLAM_H
#include "preheader.h"

#define LMS_511
//#define LMS_211

#define M_PI 3.141592654
#define D2R (M_PI/180.0)
#define R2D (180.0/M_PI)
#define MAX_AGNLE_D 180.0
#define MIN_ANGLE_D 0
#define MAX_ANGLE_R M_PI
#define MIN_ANGLE_R 0

#ifdef LMS_511
	#define MAX_LASER_NUM 361
	#define PM_DFI 0.5
#endif
#ifdef LMS_211
	#define MAX_LASER_NUM 181
	#define PM_DFI 1
#endif 


typedef struct _Scan{
	float range[MAX_LASER_NUM];
	float bearing[MAX_LASER_NUM];
}Sick_Scan;

typedef struct PMScan* pPMScan;

class CPSMSlam{
public:
	CPSMSlam();
	~CPSMSlam();
	// for SICK 511
	void runSick511LogImproved(string filename); // run without odometry
	int read_scan_sick(FILE* file, pPMScan ls);
	void runSick511Log(string filename);
	void read_SICK511(string filename);

	// for Flirt data
	void runFlirtData(string filename);

	// for test
	pPMScan getframe(vector<float> input_scan);
	void run(string fdata);
	void runtest();
    void testBug();
public:
	float m_pm_angle[MAX_LASER_NUM];
	float m_pm_cos[MAX_LASER_NUM];
	float m_pm_sin[MAX_LASER_NUM];
	//vector<Sick_Scan> m_Scan;
	vector<pPMScan> m_Scan;
};


#endif