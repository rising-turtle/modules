#pragma once
//#include <stdio.h>
#include <map>
#include <string>

using namespace std;
#define COMMENT_CHAR '#'

class CFileConfig{

public:
	CFileConfig();
	CFileConfig(const string& filename);
	bool UpdateParamMap();
	bool UpdateGlobalParam();

private:
	bool IsSpace(char c);
	bool IsCommentChar(char c);
	void Trim(string & str);
	bool AnalyseLine(const string & line, string & key, string & value);

private:
	string m_strFileName;
	std::map<std::string, std::string> m_mapParams;
	typedef std::map<std::string, std::string>::iterator ParamMapIt;
};