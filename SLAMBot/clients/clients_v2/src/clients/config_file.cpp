#include "config_file.h"
#include <iostream>
#include <stdlib.h>

CConfigFile::CConfigFile(const char* str)
{
	ifstream inf(str); 
	initParam(inf);
}

CConfigFile::CConfigFile(ifstream& inf)
{
	initParam(inf);
}

void CConfigFile::initParam(ifstream& inf)
{
	if(!inf.is_open()) 
	{
		cout<<"config file not exist!"<<endl;
		return; 
	}
	char line[4096]; 
	string delim("=, ");
	string key; 
	string value;
	while(inf.getline(line, sizeof(line)))
	{
		key = string(strtok(line, delim.c_str()));
		value = string(strtok(NULL, delim.c_str()));
		params_[key] = value;
	}
}

bool CConfigFile::getParamStr(string name, string& v)
{
	iterator it = params_.find(name);
	if(it == params_.end())
	{
		return false;
	}
	v = it->second;
	return true;
}

bool CConfigFile::getParam(string name, int& v, int def_v)
{
	string value;
	if(!getParamStr(name, value))
	{
		v = def_v;
		return false;
	}
	v = atoi(value.c_str());
	return true;
}

bool CConfigFile::getParam(string name, bool& v, bool def_v)
{
	int intv ;
	if(!getParam(name, intv, -1))
	{
		v = def_v;
	}
	v = (intv==0)?false:true;
	return true;
}

bool CConfigFile::getParam(string name, string& v, string def_v)
{
	if(!getParamStr(name, v))
	{
		v = def_v;
		return false;
	}
	return true;
}

bool CConfigFile::getParam(string name, double& v, double default_v)
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

