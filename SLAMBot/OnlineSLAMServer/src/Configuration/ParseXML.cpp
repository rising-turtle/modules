#include "ParseXML.h"



ParseXML::ParseXML(void)
{
}

ParseXML::~ParseXML(void)
{
}

int ParseXML::ParseXMLRun(const char *pcPath)
{
	int nBuffLen;
	int nRtn=0;
	if(ReadWholeFile(pcPath)!=true)
	{
		printf("XMLPath:%s  \n",pcPath);
	}
	else
	{
		Stat2Parse(m_pcFileBuff,nBuffLen);
		nRtn=1;
	}

	return nRtn;
}

bool ParseXML::ReadWholeFile(const char *pcPath)
{
	int i,j;
	FILE * pFile;   
	size_t result;
	char *pcFileBuff1,*pcFileBuff2;
	int nLen1,nLen2;
	bool bJump;
	//* ��Ҫһ��byte��©�ض�������ļ���ֻ�ܲ��ö����Ʒ�ʽ�� 
	pFile = fopen (pcPath, "rb" );  
	if (pFile==NULL)  
	{  
		printf("failed to read XMLPath:%s  \n",pcPath);
		return false;
	}  

	//* ��ȡ�ļ���С 
	fseek (pFile , 0 , SEEK_END);  
	nLen1 = ftell (pFile);  //tell the pointer drift number
	rewind (pFile);  //re-point to buff head
	//fseek (pFile , 0 , SEEK_SET);  



	//* �����ڴ�洢����ļ� 
	pcFileBuff1 = (char*) malloc (sizeof(char)*nLen1);  
	pcFileBuff2 = (char*) malloc (sizeof(char)*nLen1); 
	memset(pcFileBuff1,0,sizeof(char)*nLen1);
	if (pcFileBuff1 == NULL)  
	{  
		return false;  
	}  

	//* ���ļ�������buffer�� 
	result = fread (pcFileBuff1,1,nLen1,pFile); 


	if (result != nLen1)  
	{  
		return false;  
	}  

	nLen2=0;
	for (i=0;i<nLen1;i++)
	{
		
		
		if (pcFileBuff1[i]=='<'&&pcFileBuff1[i+1]=='!'&&pcFileBuff1[i+2]=='-'&&pcFileBuff1[i+3]=='-')
		{
			bJump=false;
			j=i;
			while (!bJump)
			{
				if (pcFileBuff1[j]=='-'&& pcFileBuff1[j+1]=='-'&&pcFileBuff1[j+2]=='>')
				{
					bJump=true;
				}
				j++;
			}
			i=j+2;
		}
		else if (pcFileBuff1[i]=='<'&&pcFileBuff1[i+1]=='?')
		{
			bJump=false;
			j=i;
			while (!bJump)
			{
				if (pcFileBuff1[j]=='?'&& pcFileBuff1[j+1]=='>')
				{
					bJump=true;
				}
				j++;
			}
			i=j+1;
		}
		else
		{
		//	if (pcFileBuff1[i]>=21)
			{
				pcFileBuff2[nLen2]=pcFileBuff1[i];
				nLen2++;
			}
		}
	}

	m_pcFileBuff=new char[nLen2];
	memcpy(m_pcFileBuff,pcFileBuff2,nLen2);

	free(pcFileBuff1);
	pcFileBuff1=NULL;

	free(pcFileBuff2);
	pcFileBuff1=NULL;

	//* ������ʾ���ر��ļ����ͷ��ڴ�   
	fclose (pFile);  
	return true;
}

bool ParseXML::Stat2Parse(char * pcBuff,int &nLen)
{
	char *pcBuffHead=pcBuff;
	char *pcTmp1=pcBuffHead;
	char cNote[1000];
	bool bJump=false,bJump2=false;
	int nThisTimeReadLen=0;
	int nCount,nCount2;
	nLen=0;
	
	nCount2=0;
	while (!bJump)
	{
		nCount=0;
		while (*(pcTmp1+nCount)==' '|| *(pcTmp1+nCount)=='	'||*(pcTmp1+nCount)==13||*(pcTmp1+nCount)==10)
		{
			nCount++;
		}
		pcTmp1+=nCount;
		nLen+=nCount;

		//exit condition
		if (*pcTmp1=='<'&&*(pcTmp1+1)=='/')
		{
			nCount=0;
			memset(cNote,0,1000);
			while (*(pcTmp1+nCount)!='>')
			{
				cNote[nCount]=*(pcTmp1+nCount);
				nCount++;
			}
			if (nCount2!=0)
			{
				nCount++;
				pcTmp1+=nCount;
				nLen+=nCount;
			}

			return true;
		}

		if (*pcTmp1=='<')
		{
			nCount=0;
			memset(cNote,0,1000);
			while (*(pcTmp1+nCount)!='>')
			{
				cNote[nCount]=*(pcTmp1+nCount);
				nCount++;
			}
			nCount++;
			
			if (nCount2==0)
			{
				GetContent(cNote,nCount,1);
				pcTmp1+=nCount;
				nLen+=nCount;

				if (cNote[nCount-2]=='/')
				{
					//m_vctData.push_back(cNote);
					
					bJump=true;
					//push data 2 Vector
				}
				else 
				{
					Stat2Parse(pcTmp1,nThisTimeReadLen);
					pcTmp1+=nThisTimeReadLen;
					nLen+=nThisTimeReadLen;
					//	bJump=true;
				}
			}
			else
			{
				if (cNote[nCount-2]=='/')
				{
					//m_vctData.push_back(cNote);
					//GetContent(cNote,nCount,1);
					GetContent(cNote,nCount,1);
					pcTmp1+=nCount;
					nLen+=nCount;
					//push data 2 Vector
				}
				else 
				{
					Stat2Parse(pcTmp1,nThisTimeReadLen);
					pcTmp1+=nThisTimeReadLen;
					nLen+=nThisTimeReadLen;
					//	bJump=true;
				}
			}
			
			

		}
		else 
		{
			nCount=0;
			memset(cNote,0,1000);
			while (*(pcTmp1+nCount)!='<')
			{
				cNote[nCount]=*(pcTmp1+nCount);
				nCount++;
			}
			pcTmp1+=nCount;
			nLen+=nCount;
			//push data 2 Vecotr
			//m_vctData.push_back(cNote);
			GetContent(cNote,nCount,0);
			bJump=true;
		}
		nCount2++;
	}
	
	return true;
}

bool ParseXML::GetContent(char *pcBuff, int &nLen, int nStyle)
{
	int i,j,nDataLen;
	char *pcContent;
	char cTmp[1000];
	//nStyle==0  
	if (nStyle==0)
	{
		pcContent=new char[nLen+1];
		memcpy(pcContent,pcBuff,nLen);
		pcContent[nLen]=0;
		m_vctData.push_back(pcContent);
		delete [] pcContent;
	}


	//nStyle==1

	else if (nStyle==1)
	{
		for (i=0;i<nLen;i++)
		{
			if (pcBuff[i]==34)
			{
				nDataLen=0;
				j=i+1;
			//	cTmp[nDataLen]=pcBuff[j];
			//	nDataLen++;
				while (pcBuff[j]!=34)
				{
					cTmp[nDataLen]=pcBuff[j];
					nDataLen++;
					j++;
				}

				pcContent=new char[nDataLen+1];
				memcpy(pcContent,cTmp,nDataLen);
				pcContent[nDataLen]=0;
				m_vctData.push_back(pcContent);
				delete [] pcContent;

				i=j;
			}

		}
	}
	return true;
}

