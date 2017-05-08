#pragma once
#include "Net/ClientNet.h"
#include "ParseXML.h"


/*typedef struct RegisterInfo
{
	int nID;
	float fOffsetPamras[4];
}RegisterInfo;*/
class OnlineSLAMClient
{
public:
	OnlineSLAMClient(void);
	~OnlineSLAMClient(void);

	int OnlineSLAMClientInit();
	int OnlineSLAMClientRun();
	int OnlineSLAMClientStop();
	int OnlineSLAMClientUninit();

private:
	ClientNet m_CClientNet;
	ParseXML m_CParseXML;
	char m_cCncOnlineSLAMSvrParams[40];
	char m_cStorageFileIdxPath[100];
	char m_cRGBFilePath[100];
	char m_cDepthFilePath[100];


	int LoadConfFile(char *pcFilepath);

	bool m_bStopSendFilesOrderly;
	HANDLE m_hThreadSendFilesOrderly;
	static DWORD WINAPI ThreadSendFilesOrderly(LPVOID lpParam);


	bool m_bStopSendLatestFile;
	HANDLE m_hThreadSendLatestFile;
	static DWORD WINAPI ThreadSendSendLatestFile(LPVOID lpParam);


	bool m_bStopFileStatusUpdate;
	HANDLE m_hThreadFileStatusUpdate;
	static DWORD WINAPI ThreadFileStatusUpdate(LPVOID lpParam);

	bool m_bStopNetCnc;
	HANDLE m_hThreadNetCnc;
	static DWORD WINAPI ThreadNetCnc(LPVOID lpParam);


	int MonitorFilesStorageStatus(vector<string> &vctFileList,string & strLatestFileName);

	vector<string> m_vctFileList;

	int Start2SendFile(string strFileName,int nType);

	int ReadFileData(char *pcBuff,int & nDataLen,char *pcFileName);


	static 	int RecvCMDFromSrver(char *pcData,int nDataLen);

	static OnlineSLAMClient *m_pCOnlineSLAMClient;

	RegisterInfo m_stRegisterInfo;
};
