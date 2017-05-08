#include "preheader.h"
#include "TinySlam.h"
//#include "PSMSlam.h"
//#include ".\psm\polar_match.h"
#include ".\zhpsm\ZHPolar_Match.h"

string file1("D:\\myproj\\2DSlam\\Slam2D\\Slam2D\\SICK511.txt");
string file2("D:\\exprdata\\slamlog\\intel-lab.log");
string file3("D:\\exprdata\\ourlog\\LMS151_Indoor_Results\\indoor.txt");
string file4("D:\\exprdata\\slamlog\\fr079.log");

int main(){
	//CTinySlam tinyslam;
	//tinyslam.run(".//tinyslam//test_lab2.dat");
	//tinyslam.runSick511("D:\\myproj\\2DSlam\\Slam2D\\Slam2D\\out.txt");
	//testGetline();
	//CPSMSlam psmslam;
	//psmslam.runSick511LogImproved(file1);
	//psmslam.testBug();
	//psmslam.runSick511Log("D:\\myproj\\2DSlam\\Slam2D\\Slam2D\\dpslam\\loop5.log");
	//psmslam.runSick511Log(file1);
	
	//psmslam.run(".\\PSM\\sick_lms200.log");
	//psmslam.runFlirtData(".\\psm\\intel-lab.log");
	//psmslam.run(".\\PSM\\hokuyo_urg_04lx_ug1_example.txt");
	//psmslam.runtest();

	 // CPolarMatch mypsm(_BEARING_361);
	 // mypsm.runFlirtFile(file4,100);
	 // mypsm.runSICKFile(file4,100);
		

	   CPolarMatch mypsm(_BEARING_181);
	   mypsm.runFlirtFile(file2);

	 // CPolarMatch mypsm(_BEARING_541);
	 // mypsm.runOurFile(file3);

	cout<<"Finish!"<<endl;

	return 0;
}