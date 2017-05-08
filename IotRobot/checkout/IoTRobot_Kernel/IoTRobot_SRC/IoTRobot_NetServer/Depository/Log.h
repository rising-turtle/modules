

#include "InternalDefine.h"

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
		filename.assign("D:\\NetServer.log");
		// first create this class initialize the first line
		if(first_open)
		{
			file_handle.open(filename.c_str(),ios::out);
			file_handle<<"Frame "<<"    Slam process "<<"     DataObtain"<<"       FeatureDetector"<<"       Mapbuilder process "<<"\n";
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

	inline void writeintolog(DWORD *pulTimeArray, bool anotherline = false)
	{
		double dGetDataTime,dSLAMTime,dFeatureTime,dMapBuilderTime;
		static char buf[200];
		dGetDataTime=(double)(pulTimeArray[1]-pulTimeArray[0]);
		dFeatureTime=(double)(pulTimeArray[2]-pulTimeArray[1]);
		dSLAMTime=(double)(pulTimeArray[3]-pulTimeArray[2]);
		dMapBuilderTime=(double)(pulTimeArray[5]-pulTimeArray[4]);

		sprintf(buf,"%d          %0.2f             %0.2f           %0.2f               %0.2f",pulTimeArray[7],dSLAMTime,dGetDataTime,dFeatureTime,dMapBuilderTime);
		std::string str(buf);
		writeintolog(str,true);
	}
public:
	fstream file_handle;
	string filename;
	boost::mutex logmutex;

	static bool first_open;
};

#endif