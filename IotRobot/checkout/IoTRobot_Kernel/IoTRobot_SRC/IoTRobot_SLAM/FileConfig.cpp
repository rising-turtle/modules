#include "FileConfig.h"
#include "globaldefinitions.h"
#include <fstream>

CFileConfig::CFileConfig(){
	m_strFileName = "D:\\rgbdslam.ini";
}

CFileConfig::CFileConfig(const std::string &filename){
	m_strFileName = filename;
}

bool CFileConfig::UpdateParamMap(){
	// clear the existing param map
	m_mapParams.clear();
	ifstream infile(m_strFileName.c_str());
	if (!infile) { 
		//cout << "file open error" << endl;        
		return false;    
	}    
	string line, key, value;    
	while (getline(infile, line)) { 
		if (AnalyseLine(line, key, value)) {       
			m_mapParams[key] = value;        
		}    
	}
	infile.close();

	return true;
}

bool CFileConfig::UpdateGlobalParam(){
	string key;
	ParamMapIt it;

	key = "global_connectivity";
	it = m_mapParams.find(key);
	if(it != m_mapParams.end())
		global_connectivity = atoi(it->second.c_str());

	key = "global_potential_nodes";
	it = m_mapParams.find(key);
	if(it != m_mapParams.end())
		global_potential_nodes = atoi(it->second.c_str());

	key = "global_graph_size";
	it = m_mapParams.find(key);
	if(it != m_mapParams.end())
		global_graph_size = atoi(it->second.c_str());

	key = "global_bg_graph_threshold";
	it = m_mapParams.find(key);
	if(it != m_mapParams.end())
		global_bg_graph_threshold = atoi(it->second.c_str());

	key = "global_min_translation_meter";
	it = m_mapParams.find(key);
	if(it != m_mapParams.end())
		global_min_translation_meter = atof(it->second.c_str());

	key = "global_min_rotation_degree";
	it = m_mapParams.find(key);
	if(it != m_mapParams.end())
		global_min_rotation_degree = atof(it->second.c_str());

	key = "global_max_translation_meter";
	it = m_mapParams.find(key);
	if(it != m_mapParams.end())
		global_max_translation_meter = atof(it->second.c_str());

	key = "global_feature_detector_type";
	it = m_mapParams.find(key);
	if(it != m_mapParams.end())
		global_feature_detector_type=it->second;
	return true;
}

bool CFileConfig::IsSpace(char c){   
	if (' ' == c || '\t' == c)       
		return true;   
	return false;
}

bool CFileConfig::IsCommentChar(char c){    
	switch(c) {    
case COMMENT_CHAR:        
	return true;   
default:       
	return false;    
	}
}

void CFileConfig::Trim(string & str){    
	if (str.empty()) {      
		return;    
	}   
	int i, start_pos, end_pos;   
	// find the first valid char
	for (i = 0; i < str.size(); ++i) {    
		if (!IsSpace(str[i])) {          
			break;        
		}    
	}
	if (i == str.size()) {
		// It is a empty string!!!
		str = "";       
		return;   
	}
	start_pos = i;  
	// find the last valid char
	for (i = str.size() - 1; i >= 0; --i) {     
		if (!IsSpace(str[i])) {     
			break;       
		}    
	}
	end_pos = i;       
	// compose the valid string
	str = str.substr(start_pos, end_pos - start_pos + 1);
}

bool CFileConfig::AnalyseLine(const string & line, string & key, string & value){  
	if (line.empty())      
		return false;    

	int start_pos = 0, end_pos = line.size() - 1, pos;   
	if ((pos = line.find(COMMENT_CHAR)) != -1) {   
		if (0 == pos) {  
			// The first char is comment char!     
			return false;        
		}       
		end_pos = pos - 1;   
	}   
	string new_line = line.substr(start_pos, start_pos + 1 - end_pos); 

	if ((pos = new_line.find('=')) == -1)      
		return false;  
            
	key = new_line.substr(0, pos);    
	value = new_line.substr(pos + 1, end_pos + 1- (pos + 1));      
	Trim(key);   
	if (key.empty()) {       
		return false;  
	}   
	Trim(value); 
	return true;
}
