#ifndef CONFIG_FILE_H
#define CONFIG_FILE_H

#include <vector>
#include <map>
#include <string>
#include <fstream>
#include <stdlib.h>

using namespace std;

class CConfigFile
{
public:
	CConfigFile(ifstream&);
	CConfigFile(const char*);

	// g++ without any problem, but vc need following declaration
	// 因为vc 不支持模板特化，即使特化，也必须放在.h 文件中，不能在其它.cpp 中调用
	// template<typename T > 
	// bool getParam(string name, T&, T default_v); 

	bool getParam(string name, int&, int);
	bool getParam(string name, bool&, bool);
	bool getParam(string name, string& , string );
	bool getParam(string name, double&, double);

/*	template<>
	bool getParam<int>(string name, int&, int);

	template<>
	bool getParam<string>(string name, string&, string);
*/
	typedef map<string, string>::iterator iterator;
protected:
	void initParam(ifstream&);
	map<string, string> params_; 
	bool getParamStr(string name, string& v);

};
/*
template<typename T>
bool CConfigFile::getParam(string name, T& v, T default_v)
{
	string value; 
	if(!getParamStr(name, value))
	{
		v = default_v;
		return false;
	}
	v = atof(value.c_str());
	return true;
}
*/
#endif