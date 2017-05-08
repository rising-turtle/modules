#ifndef POLAR_MATCH_H
#define POLAR_MATCH_H

#include <string>
#include "PolarParameter.h"
using namespace std;


// To handle the front_end of our SLAM algorithm, that is,
// Input  / two scan_matchings
// Output / relative motions

class CPolarMatch
{
public:
	CPolarMatch(string laser_name);
	~CPolarMatch();

	// PSM Match Functions
	void pm_init(); // init bearing variables
	void pm_preprocessScan(PMScan *ls); // preprocess the scan-matching
	PM_TYPE pm_psm ( const PMScan *lsr,PMScan *lsa ); //Match two laser scans using polar scan matching. 
	
	// run PSM using SICK log 
	void runSICKFile(string filename, int run_num =-1);

protected:
	// PSM Match Functions
	void pm_median_filter(PMScan* ls); // median filter the scan-matching
	void pm_find_far_points (PMScan *ls ); // far filter
	void pm_segment_scan ( PMScan *ls );	// segment
	void pm_scan_project(const PMScan *act,  PM_TYPE   *new_r,  int *new_bad); //Performs scan projection.
	PM_TYPE pm_orientation_search(const PMScan *ref, const PM_TYPE *new_r, const int *new_bad); //Performs one iteration of orientation alignment of current scan.
	PM_TYPE pm_translation_estimation(const PMScan *ref, const PM_TYPE *new_r, const int *new_bad, PM_TYPE C, PM_TYPE *dx, PM_TYPE *dy); // Estimate the postion of the current scan with respect to a reference scan.

	// Input from Sick File
	bool readSICK(string filename);

protected:
	PM_TYPE*   pm_fi;//contains precomputed angles (0-180)
	PM_TYPE*   pm_si;//contains sinus of angles
	PM_TYPE*   pm_co;//contains cos of angles

protected:
	Base_PARAM* m_pParam;	// Parameters for laser 
	bool m_bReady;			// Whether the laser info is right
	vector<PMScan*> m_SickScans;	// Record scans
private:
	CPolarMatch(const CPolarMatch&);
	CPolarMatch& operator=(const CPolarMatch&);
};

#endif