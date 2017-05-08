#ifndef TINYSLAM_H
#define TINYSLAM_H
#include "preheader.h"

class CTinySlam{
public:
	CTinySlam();
	~CTinySlam();
	
	// those functions failed for there are two many places have to be changed to 
	// be compatible with SICK 511
	// read sick data as the input struct
	int readSick511(string fdata);
	void runSick511(string fdata);

	// run the example as the openSLAM offers
	void run(string fdata);
};

#endif