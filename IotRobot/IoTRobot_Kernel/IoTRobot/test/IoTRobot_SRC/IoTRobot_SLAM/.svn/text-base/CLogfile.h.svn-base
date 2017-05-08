#ifndef CLOGFILE_H
#define CLOGFILE_H

#include <iostream>
#include <fstream>
#include <string>
#include <boost/thread/thread.hpp>

using namespace std;

class CLogfile{
public:
	CLogfile(){
		filename.assign("D:\\CLogfile.log");
		// first create this class initialize the first line
		if(first_open)
		{
			file_handle.open(filename.c_str(),ios::out);
			file_handle<<"Frame "<<"\tSlam process "<<"\tDataObtain"<<"\tFeatureDetector"<<"\tMapbuilder process "<<"\n";
			file_handle.close();
			first_open = false;
		}
	}
	~CLogfile(){
		//if(file_handle.is_open())
		//	file_handle.close();
	}
	inline void getlogfile(){
		logmutex.lock();
		file_handle.open(filename.c_str(),ios::app|ios::out);
	}
	inline void releaselogfile(){
		file_handle.close();
		logmutex.unlock();
	}
	inline void writeintolog(double& start_t, double& end_t, bool anotherline = false){
		static char buf[20];
		sprintf(buf,"\t%0.2f ",end_t - start_t);
		std::string str(buf);
		writeintolog(str,anotherline);
	}
	inline void writeintolog(std::string& str,bool anotherline = false){
		getlogfile(); // lock file
		if(anotherline)
			file_handle<<str<<std::endl;
		else
			file_handle<<str;
		releaselogfile(); // release file
	}
public:
	fstream file_handle;
	string filename;
	boost::mutex logmutex;

	static bool first_open;
};


#endif