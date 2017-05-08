#include "OnlineSLAMClient.h"

extern bool getLatestFileName(std::string&);

CRITICAL_SECTION g_MutexSendFiles;
OnlineSLAMClient *OnlineSLAMClient::m_pCOnlineSLAMClient=NULL;
OnlineSLAMClient::OnlineSLAMClient(void)
{
	m_pCOnlineSLAMClient=this;
}

OnlineSLAMClient::~OnlineSLAMClient(void)
{
}


int OnlineSLAMClient::OnlineSLAMClientInit()
{
	m_vctFileList.clear();
	InitializeCriticalSection(&g_MutexSendFiles);
	LoadConfFile("OnlineSLAMClientParam.xml");
	
	m_CClientNet.m_cbRecvData=RecvCMDFromSrver;
	m_CClientNet.m_stRegisterInfo=m_stRegisterInfo;
	m_CClientNet.ComInit(m_cCncOnlineSLAMSvrParams);
	return 0;
}
int OnlineSLAMClient::OnlineSLAMClientRun()
{
	DWORD nID=0;
	m_bStopSendFilesOrderly=false;
	m_bStopSendLatestFile=false;
	m_bStopFileStatusUpdate=false;


	//m_hThreadSendFilesOrderly=CreateThread(NULL,0,ThreadSendFilesOrderly,(LPVOID)this,0,&nID);
	//nID++;
	//m_hThreadSendLatestFile=CreateThread(NULL,0,ThreadSendSendLatestFile,(LPVOID)this,0,&nID);
	//m_hThreadFileStatusUpdate=CreateThread(NULL,0,ThreadFileStatusUpdate,(LPVOID)this,0,&nID);
	//nID++;
	m_hThreadNetCnc=CreateThread(NULL,0,ThreadNetCnc,(LPVOID)this,0,&nID);


	return 0;
}
int OnlineSLAMClient::OnlineSLAMClientStop()
{
	return 0;
}
int OnlineSLAMClient::OnlineSLAMClientUninit()
{
	return 0;
}


int OnlineSLAMClient::RecvCMDFromSrver(char *pcData,int nDataLen)
{
	int nRtn=0;
	char *pcCMD="Snap";
	if (memcmp(pcData,pcCMD,4)==0)
	{
		printf("Snap one frame!!!!\n");
		string strLatestFileName;
		// EnterCriticalSection(&g_MutexSendFiles);
		bool getOK = getLatestFileName(strLatestFileName);
		nRtn = getOK? 0:-1;
		//	nRtn=m_pCOnlineSLAMClient->MonitorFilesStorageStatus(m_pCOnlineSLAMClient->m_vctFileList,strLatestFileName);
		// EnterCriticalSection(&g_MutexSendFiles);
		printf("nRtn:%d\n",nRtn);
		if (nRtn==0)
		{
			printf("Xtion Ok!!!!\n");
			m_pCOnlineSLAMClient->Start2SendFile(strLatestFileName,1);
		}
		else if(nRtn==-2)
		{
			char cRtnStatus=-1;
			//memcmp(cRtnStatus,&nCMD,4);
			m_pCOnlineSLAMClient->m_CClientNet.SendData(&cRtnStatus,1);
			printf("Xtion error!!!!\n");
		}
		else if (nRtn==-1)
		{
			printf("load index file err!!!!\n");
		}

	}
	return 0;
}

int OnlineSLAMClient::LoadConfFile(char *pcFilepath)
{
	int nPort;
	memset(m_cCncOnlineSLAMSvrParams,0,40);
	memset(m_cStorageFileIdxPath,0,100);
	memset(m_cRGBFilePath,0,100);
	memset(m_cDepthFilePath,0,100);


	if(!m_CParseXML.ParseXMLRun(pcFilepath))
	{
		printf("read conf failed!!!\n");
		return -1;
	}

	memcpy(m_cCncOnlineSLAMSvrParams,m_CParseXML.m_vctData[0].c_str(),
		strlen(m_CParseXML.m_vctData[0].c_str()));
	nPort=atoi(m_CParseXML.m_vctData[1].c_str());
	memcpy(m_cCncOnlineSLAMSvrParams+16,&nPort,4);
	memcpy(m_cCncOnlineSLAMSvrParams+20,m_CParseXML.m_vctData[2].c_str(),
		strlen(m_CParseXML.m_vctData[2].c_str()));
	nPort=atoi(m_CParseXML.m_vctData[3].c_str());
	memcpy(m_cCncOnlineSLAMSvrParams+36,&nPort,4);

	memcpy(m_cStorageFileIdxPath,m_CParseXML.m_vctData[4].c_str(),strlen(m_CParseXML.m_vctData[4].c_str()));
	memcpy(m_cDepthFilePath,m_CParseXML.m_vctData[5].c_str(),strlen(m_CParseXML.m_vctData[5].c_str()));
	memcpy(m_cRGBFilePath,m_CParseXML.m_vctData[6].c_str(),strlen(m_CParseXML.m_vctData[6].c_str()));

	m_stRegisterInfo.nID=atoi(m_CParseXML.m_vctData[7].c_str());
	m_stRegisterInfo.fOffsetPamras[0]=atof(m_CParseXML.m_vctData[8].c_str());
	m_stRegisterInfo.fOffsetPamras[1]=atof(m_CParseXML.m_vctData[9].c_str());
	m_stRegisterInfo.fOffsetPamras[2]=atof(m_CParseXML.m_vctData[10].c_str());
	m_stRegisterInfo.fOffsetPamras[3]=atof(m_CParseXML.m_vctData[11].c_str());
	return 0;
}

DWORD WINAPI OnlineSLAMClient::ThreadNetCnc(LPVOID lpParam)
{
	OnlineSLAMClient *pCOnlineSLAMClient=(OnlineSLAMClient *)lpParam;
	pCOnlineSLAMClient->m_CClientNet.ComRun(NULL);
	return 0;
}

DWORD WINAPI OnlineSLAMClient::ThreadFileStatusUpdate(LPVOID lpParam)
{
	OnlineSLAMClient *pCOnlineSLAMClient=(OnlineSLAMClient *)lpParam;
	while(!pCOnlineSLAMClient->m_bStopFileStatusUpdate)
	{
		string strLatestFileName;
		EnterCriticalSection(&g_MutexSendFiles);
		pCOnlineSLAMClient->MonitorFilesStorageStatus(pCOnlineSLAMClient->m_vctFileList,strLatestFileName);
		LeaveCriticalSection(&g_MutexSendFiles);
		//	pCOnlineSLAMClient->Start2SendFile(strLatestFileName);
		Sleep(3000);
	}
	return 0;
}
/*

DWORD WINAPI OnlineSLAMClient::ThreadSendSendLatestFile(LPVOID lpParam)
{
OnlineSLAMClient *pCOnlineSLAMClient=(OnlineSLAMClient *)lpParam;
while(!pCOnlineSLAMClient->m_bStopSendLatestFile)
{
string strLatestFileName;
EnterCriticalSection(&g_MutexSendFiles);
pCOnlineSLAMClient->MonitorFilesStorageStatus(pCOnlineSLAMClient->m_vctFileList,strLatestFileName);
EnterCriticalSection(&g_MutexSendFiles);
pCOnlineSLAMClient->Start2SendFile(strLatestFileName);
Sleep(3000);
}
return 0;
}*/

DWORD WINAPI OnlineSLAMClient::ThreadSendFilesOrderly(LPVOID lpParam)
{
	OnlineSLAMClient *pCOnlineSLAMClient=(OnlineSLAMClient *)lpParam;
	int nCurFileListLen=0;
	string strSendFilename;
	while(!pCOnlineSLAMClient->m_bStopSendFilesOrderly)
	{
		EnterCriticalSection(&g_MutexSendFiles);
		nCurFileListLen=pCOnlineSLAMClient->m_vctFileList.size();
		if (nCurFileListLen>0)
		{
			strSendFilename=pCOnlineSLAMClient->m_vctFileList[0];
		}
		LeaveCriticalSection(&g_MutexSendFiles);
		if (nCurFileListLen>0&&pCOnlineSLAMClient->m_CClientNet.GetNetStatus()==1)
		{
			//string strSendFilename=pCOnlineSLAMClient->m_vctFileList[0];
			//EnterCriticalSection(&g_MutexSendFiles);
			if(pCOnlineSLAMClient->Start2SendFile(strSendFilename,2)==0)
			{
				EnterCriticalSection(&g_MutexSendFiles);
				pCOnlineSLAMClient->m_vctFileList.erase(pCOnlineSLAMClient->m_vctFileList.begin()+0);
				LeaveCriticalSection(&g_MutexSendFiles);
			}
		}
		LeaveCriticalSection(&g_MutexSendFiles);
		Sleep(25);
	}
	return 0;
}

int OnlineSLAMClient::Start2SendFile(string strFileName,int nType)
{
	int nRtn=-1;
	char cRGBFileName[128],cDepthFilename[128];
	char cFileName[128];

	char *pcBuff=new char[2*1024*1024];
	char *pcHead=NULL;
	//char *pcRGBBuff=new char [1024*1024];
	//char *pcDepthBuff=new char [1024*1024];
	char *pcTmp=(char *)strFileName.c_str();
	int i=0,nRGBFileNameLen=0,nRGBDataLen,nDepthDataLen,nDataLen=0;
	char *pcSuffix=".png";


	memset(pcBuff,0,2*1024*1024);
	char cType=(char)nType;
	pcHead=pcBuff;
	memcpy(pcHead,&cType,1);
	pcHead+=1;

	memset(cRGBFileName,0,128);
	memset(cDepthFilename,0,128);
	memset(cFileName,0,128);



	while(pcTmp[i]!='\0'&&pcTmp[i]!=' ')
	{
		cFileName[i]=pcTmp[i];
		i++;
	}
	memcpy(cFileName+strlen(cFileName),pcSuffix,strlen(pcSuffix));

	nRGBFileNameLen=strlen(cFileName);
	nDepthDataLen=nRGBFileNameLen;

	memcpy(cRGBFileName,m_cRGBFilePath,strlen(m_cRGBFilePath));
	memcpy(cDepthFilename,m_cDepthFilePath,strlen(m_cDepthFilePath));

	cRGBFileName[strlen(cRGBFileName)]='/';
	cDepthFilename[strlen(cDepthFilename)]='/';

	memcpy(cRGBFileName+strlen(cRGBFileName),cFileName,strlen(cFileName));
	memcpy(cDepthFilename+strlen(cDepthFilename),cFileName,strlen(cFileName));

	if (nRGBFileNameLen!=0)
	{

		memcpy(pcHead,cFileName,nRGBFileNameLen);
		pcHead+=31;
		//memcpy(cDepthFilename,cRGBFileName,nRGBFileNameLen);
		//memcpy(cDepthFilename+nRGBFileNameLen,pcSuffix,strlen(pcSuffix));
		//nDepthDataLen=nRGBFileNameLen+strlen(pcSuffix);

		printf("RGBFile:%s  ,Depth:%s  \n",cRGBFileName,cDepthFilename);
		if(ReadFileData(pcHead+4,nRGBDataLen,cRGBFileName)!=0)
		{
			return -1;
		}
		memcpy(pcHead,&nRGBDataLen,4);
		pcHead+=(nRGBDataLen+4);

		if(ReadFileData(pcHead+4,nDepthDataLen,cDepthFilename)!=0)
		{
			return -1;
		}
		memcpy(pcHead,&nDepthDataLen,4);

		pcHead+=(nDepthDataLen+4);

		memcpy(pcHead,&m_stRegisterInfo,sizeof(RegisterInfo));


		nDataLen=40+nRGBDataLen+nDepthDataLen+sizeof(RegisterInfo);

		printf("Start 2 send :%s file\n",cFileName);
		if(m_CClientNet.SendData(pcBuff,nDataLen)==0)
		{
			nRtn=0;
		}
		else
		{
			nRtn=-1;
		}

		/*	memcpy(cDepthFilename,cRGBFileName,nRGBFileNameLen);
		memcpy(cDepthFilename+nRGBFileNameLen,pcSuffix,strlen(pcSuffix));
		nDepthDataLen=nRGBFileNameLen+strlen(pcSuffix);


		ReadFileData(pcRGBBuff,nRGBDataLen,cRGBFileName);
		ReadFileData(pcDepthBuff,nDepthDataLen,cDepthFilename);


		m_CClientNet.SendData(pcRGBBuff,nRGBDataLen);
		m_CClientNet.SendData(pcDepthBuff,nDepthDataLen);*/
		return 0;
	}
	else 
	{
		nRtn=-1;
		//return -1;
		//delete [] pcBuff;
	}
	delete [] pcBuff;

	return nRtn;
}

int OnlineSLAMClient::ReadFileData(char *pcBuff,int & nDataLen,char *pcFileName)
{
	FILE * pFile;
	int nLen;
	size_t result;
	pFile = fopen (pcFileName,"rb"); 
	if (pFile==NULL)  
	{  
		printf("failed to read data  :%s  \n",pcFileName);
		return -1;
	}
	fseek (pFile,0,SEEK_END);  
	nLen = ftell (pFile);
	rewind (pFile);
	result = fread (pcBuff,1,nLen,pFile); 
	if (result != nLen)  
	{  
		nDataLen=0;
		return -2;  
	}
	nDataLen=nLen;
	fclose(pFile);
	return 0;
}

int OnlineSLAMClient::MonitorFilesStorageStatus(vector<string> &vctFileList,string & strLatestFileName)
{
	static unsigned int uiLastReadPos=0;
	unsigned int uiLen;
	char cTmp[512];
	memset(cTmp,0,512);
	FILE * pFile; 
	printf("I get index file :%s",m_cStorageFileIdxPath);
	pFile = fopen (m_cStorageFileIdxPath,"rb");  
	
	if (pFile==NULL)  
	{  
		//fclose(pFile);
		printf("failed to read index file :%s  \n",m_cStorageFileIdxPath);
		return -1;
	}
	fseek (pFile,0,SEEK_END);  
	uiLen = ftell (pFile);

	if (uiLastReadPos!=uiLen)
	{
		fseek (pFile,uiLastReadPos,SEEK_SET); 

		while(fgets(cTmp,512,pFile)!=NULL)
		{
			string fileName(cTmp);
			vctFileList.push_back(cTmp);
		}
		strLatestFileName=vctFileList.back();

		//do not send the stored file orderly
		//vctFileList.pop_back();

		uiLastReadPos=uiLen;
		fclose(pFile);
		return 0;
	}else
	{
		fclose(pFile);
		return -2;
	}

}

/*
int main()
{
	OnlineSLAMClient COnlineSLAMClient;
	COnlineSLAMClient.OnlineSLAMClientInit();

//	Sleep(10000);
	COnlineSLAMClient.OnlineSLAMClientRun();

	while(1)
	{
		Sleep(100000);
	}
	return 0;
}*/