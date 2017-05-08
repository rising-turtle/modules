#include "preheader.h"

#include "GraphManager.h"

extern int testFlirt();

namespace{
	string loggt1="D:\\exprdata\\slamlog\\intel-lab.log";
	string loggt2="D:\\exprdata\\slamlog\\fr079.log";

	string logfile2="D:\\exprdata\\SICK511.txt";
	string ourfile("D:\\exprdata\\ourlog\\LMS151_Indoor_Results\\indoor.txt");

	string outfile1("D:\\exprdata\\intel_comp\\ground_truth.log");
	void changefile(string input,string output)
	{
		ifstream in_file(input.c_str());
		ofstream out_file(output.c_str());
		char line[4096];
		float x,y,th;
		while(in_file.getline(line,4096))
		{
			sscanf(line,"(%f,%f,%f)",&x,&y,&th);
			out_file<<x<<" "<<y<<" "<<th<<endl;
		}
		in_file.close();
		out_file.close();
	}

};


int main(int argc, char* argv[]){
	
	// testFlirt();
	// changefile("D:\\exprdata\\flirt_trajectory.log","d:\\exprdata\\flirt_t.log");
	
	CGraphManager graph;
	 graph.readlog(loggt1);
	 graph.runlog();
	 graph.recordTrajectory("D:\\exprdata\\intel_comp\\flirt.log");
	
	//graph.recordGTCarmon(loggt1,outfile1);

	// graph.readSicklog(logfile2.c_str(),graph.m_log);
	// graph.runSicklog();

	// graph.readOurlog(ourfile);
	// graph.runOurlog();


	cout<<"finished!"<<endl;
	getchar();
	return 0;
}