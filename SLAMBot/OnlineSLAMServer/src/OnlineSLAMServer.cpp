#include "OnlineSLAMServer.h"
#include "signal.h"
#include <sys/stat.h>

#include "cv.h"
#include "highgui.h"
#include "cxcore.h"

using namespace cv;

bool OnlineSLAMServer::m_bSnapSync[SUPPORT_CLEINT_NUM];
PCDData *OnlineSLAMServer::m_pstPCDData[SUPPORT_CLEINT_NUM];
pthread_mutex_t g_MutexRegister;

OnlineSLAMServer *OnlineSLAMServer::m_pCOnlineSLAMServer=NULL;
PosOffset OnlineSLAMServer::m_stPosOffset[SUPPORT_CLEINT_NUM];

int OnlineSLAMServer::m_nCurStatus[SUPPORT_CLEINT_NUM];
float g_fx = 1.0/577.246;//525.0;
float g_fy = 1.0/580.482;//525.0;
float g_cx = 302.494;//319.5;
float g_cy = 198.715;//239.5;
float g_factor = 1000; //for the 16-bit PNG files;
float g_depth_scale = 1;

OnlineSLAMServer::OnlineSLAMServer()
{
	m_pCOnlineSLAMServer=this;
}


OnlineSLAMServer::~OnlineSLAMServer()
{

}

int OnlineSLAMServer::OnlineSLAMServerInit()
{
	int i;
	char cConParams[30];
	memset(cConParams,0,30);
	//LoadConfFile("ResolveServerConf.xml");

	memset(m_cExternalComParams,0,1000);
	if(LoadConfFile("OnlineSLAMServerConf.xml")!=RTN_OK)
	{
		return RTN_ERR_READFILE;
	}
	pthread_mutex_init(&g_MutexRegister,0);

	memcpy(cConParams,m_cServerIP,strlen(m_cServerIP));
	memcpy(&cConParams[16],&m_nServerPort,4);

	m_CExternalCom.m_cbRegister=RegisterCenter;
	m_CExternalCom.m_cbData=HandleSLAMSycData;
	m_CExternalCom.ComInit(cConParams);



	for(i=0;i<SUPPORT_CLEINT_NUM;i++)
	{
		char cFolderName[20];
		memset(cFolderName,0,20);
		sprintf(cFolderName,"%d",i);
		mkdir(cFolderName,S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);

		sprintf(cFolderName,"%d/rgb",i);
		mkdir(cFolderName,S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);

		sprintf(cFolderName,"%d/depth",i);
		mkdir(cFolderName,S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);

		m_pstPCDData[i]=new PCDData[640*480];
		m_bSnapSync[i]=0;
		memset(&m_stClientInfo[i],0,sizeof(ClientInfo));

		m_pCOnlineSLAMServer->m_stClientInfo[i].bHasCnc=false;
		m_pCOnlineSLAMServer->m_stClientInfo[i].nCLientID=-1;
		m_pCOnlineSLAMServer->m_stClientInfo[i].nSock=-1;
	}
	m_nCurCncClientNum=0;

	m_C3DRender.MapRenderInit();
}

int OnlineSLAMServer::HandleSLAMSycData(int nSock,int nID,char *pcData,int nDataLen)
{
	//printf("Msg Type:%d \n",pcData[0]);
	if(pcData[0]==1)//RenderFrame
	{
		HandleSnapFrame(pcData+1,nID,nSock,pcData[0]);
	}
	else if(pcData[0]==-1)
	{
		HandleSnapFrame(pcData+1,nID,nSock,pcData[0]);
	}
	else if(pcData[0]==2)//SequenceFrame
	{
		HandleSequenceFrame(pcData+1,nID,nSock);
	}
	return 0;
}


int OnlineSLAMServer::HandleSnapFrame(char *pcData,int nID,int nSock,int nType)
{
	SaveRecvFile(pcData,nID,nSock,nType);
	printf("HandleSnapFrame:%d\n",nID);
	m_bSnapSync[nID]=true;
	return 0;
}

int OnlineSLAMServer::LoadConfFile(char *pcFilepath)
{
	unsigned long ulTmp;
	if(!m_CParseXML.ParseXMLRun(pcFilepath))
	{
		printf("read conf failed!!!\n");
		return -1;
	}
	memcpy(m_cServerIP,m_CParseXML.m_vctData[0].c_str(),
			strlen(m_CParseXML.m_vctData[0].c_str()));
	m_nServerPort=atoi(m_CParseXML.m_vctData[1].c_str());

	return 0;
}

int OnlineSLAMServer::HandleSequenceFrame(char *pcData,int nID,int nSock)
{
	SaveRecvFile(pcData,nID,nSock,0);
	return 0;
}


int OnlineSLAMServer::SaveRecvFile(char *pcData,int nID,int nSock,int nType)
{
	int i,j,k,nOffset;
	FILE * pFile;
	char cFileName[40];
	char cRGBFileName[40];
	char cDepthFileName[40];
	char *pcSuffix=".png";
	int nRGBFileLen,nDepthFileLen;
	char *pcFileNameHead,*pcRGBFileLenHead,*pcRGBDataHead,*pcDepthFileLenHead,*pcDepthDataHead,*pcOffsetHead;

	if(nType==1)
	{
		pcFileNameHead=pcData;
		pcRGBFileLenHead=pcData+31;
		memcpy(&nRGBFileLen,pcRGBFileLenHead,4);
		pcRGBDataHead=pcRGBFileLenHead+4;
		pcDepthFileLenHead=pcRGBDataHead+nRGBFileLen;
		memcpy(&nDepthFileLen,pcDepthFileLenHead,4);
		pcDepthDataHead=pcDepthFileLenHead+4;

		pcOffsetHead=pcDepthDataHead+nDepthFileLen;
		//memset(cFileName,0,40);
		//sprintf(cFileName,"%d/",nID);

		memset(cRGBFileName,0,40);
		sprintf(cRGBFileName,"%d/rgb/",nID);

		memset(cDepthFileName,0,40);
		sprintf(cDepthFileName,"%d/depth/",nID);

		//memcpy(cFileName+strlen(cFileName),pcFileNameHead,31);

		memcpy(cRGBFileName+strlen(cRGBFileName),pcFileNameHead,31);
		memcpy(cDepthFileName+strlen(cDepthFileName),pcFileNameHead,31);

		pFile = fopen (cRGBFileName, "wb" );
		fwrite(pcRGBDataHead,1,nRGBFileLen,pFile);
		fclose(pFile);

		//memcpy(cFileName+strlen(cFileName),pcSuffix,4);
		pFile = fopen (cDepthFileName, "wb" );
		fwrite(pcDepthDataHead,1,nDepthFileLen,pFile);
		fclose(pFile);


		memcpy(&m_stPosOffset[i],pcOffsetHead,sizeof(PosOffset));
		m_stPosOffset[i].nStatus=1;//Normal

		m_nCurStatus[nID]=1;

		cv::Mat m_rgb8u=imread(cRGBFileName,-1);
		cv::Mat m_depth16u = imread(cDepthFileName, -1);

		PCDData *pstPCDData=m_pstPCDData[nID];
		k=0;
		nOffset=0;
		for(i=0; i<m_rgb8u.rows; i++)
		{
			for(j=0; j<m_rgb8u.cols; j++)
			{
				float z = m_depth16u.at<unsigned short>(i, j);
				if (abs(z)<0.000001)
				{
					pstPCDData[k].ucR=0;
					pstPCDData[k].ucG=0;
					pstPCDData[k].ucB=0;

					pstPCDData[k].fX=0;
					pstPCDData[k].fY=0;
					pstPCDData[k].fZ=0;
					nOffset+=3;
					//color_pt.x = color_pt.y = color_pt.z = bad_point;
				}
				else
				{
					//Eigen::Vector4f pt_loc;
					pstPCDData[k].fZ = z / g_factor * g_depth_scale;
					pstPCDData[k].fX = (j - g_cx) * pstPCDData[k].fZ * g_fx;
					pstPCDData[k].fY = (i - g_cy) * pstPCDData[k].fZ * g_fy;

					pstPCDData[k].ucB=m_rgb8u.data[nOffset];
					nOffset++;
					pstPCDData[k].ucG=m_rgb8u.data[nOffset];
					nOffset++;
					pstPCDData[k].ucR=m_rgb8u.data[nOffset];
					nOffset++;

				}
				k++;
			}
		}
	}
	else
		if(nType==-1)//Kinect Error
		{
			m_stPosOffset[i].nStatus=-1;//Kinect error
			m_nCurStatus[nID]=-1;
		}

	return 0;
}

/*
int OnlineSLAMServer::OnlineSLAMServerInit()
{

	memset(m_cExternalComParams,0,1000);
	if(LoadConfFile("OnlineSLAMServerConf.xml")!=RTN_OK)
	{
		return RTN_ERR_READFILE;
	}
	pthread_mutex_init(&g_MutexRegister,0);
	return 0;
}*/


int OnlineSLAMServer::RegisterCenter(int nSock,int nID,int nType,char *pcData,int nDataLen)
{
	int i=0,nRtn,nResigerID=-1;

	pthread_mutex_lock(&g_MutexRegister);
	if(nType==1)//new client in
	{
		RegisterInfo stRegisterInfo;
		memcpy(&stRegisterInfo,pcData,nDataLen);

		m_pCOnlineSLAMServer->m_stClientInfo[stRegisterInfo.nID].nCLientID=stRegisterInfo.nID;
		m_pCOnlineSLAMServer->m_stClientInfo[stRegisterInfo.nID].nSock=nSock;
		m_pCOnlineSLAMServer->m_stClientInfo[stRegisterInfo.nID].bHasCnc=true;
		m_pCOnlineSLAMServer->m_stClientInfo[stRegisterInfo.nID].stRegisterInfo=stRegisterInfo;
		nResigerID=stRegisterInfo.nID;
		m_pCOnlineSLAMServer->m_nCurCncClientNum++;
		/*for(i=0;i<SUPPORT_CLEINT_NUM;i++)
		{
			if(!m_pCOnlineSLAMServer->m_stClientInfo[i].bHasCnc)
			{
				printf("New Client In!!!\n");
				m_pCOnlineSLAMServer->m_stClientInfo[i].nCLientID=i;
				m_pCOnlineSLAMServer->m_stClientInfo[i].nSock=nSock;
				m_pCOnlineSLAMServer->m_stClientInfo[i].bHasCnc=true;
				nResigerID=i;
				m_pCOnlineSLAMServer->m_nCurCncClientNum++;
				break;
			}
		}*/
	}
	else if(nType==2)//client gone
	{
		m_pCOnlineSLAMServer->m_stClientInfo[nID].bHasCnc=false;
		m_pCOnlineSLAMServer->m_stClientInfo[nID].nCLientID=-1;
		m_pCOnlineSLAMServer->m_stClientInfo[nID].nSock=-1;
		m_pCOnlineSLAMServer->m_nCurCncClientNum--;
	}

	pthread_mutex_unlock(&g_MutexRegister);
	return nResigerID;
}

void* OnlineSLAMServer::ThreadRenderSnapFrame(void* lpParam)
{
	int nRslt=0;
	OnlineSLAMServer *pCOnlineSLAMServer=(OnlineSLAMServer *)lpParam;
	while(!pCOnlineSLAMServer->m_bStopRenderSnapFrame)
	{

		memset(m_bSnapSync,0,sizeof(bool)*SUPPORT_CLEINT_NUM);
		memset(m_nCurStatus,0,sizeof(int)*SUPPORT_CLEINT_NUM);
		pCOnlineSLAMServer->SendSnapCMD(pCOnlineSLAMServer);
		nRslt=pCOnlineSLAMServer->WaitingforSnapFramesSync((void*)pCOnlineSLAMServer);
		pCOnlineSLAMServer->m_C3DRender.PCDDataIn(m_bSnapSync,
				m_pstPCDData,
				pCOnlineSLAMServer->m_stClientInfo,
				m_nCurStatus);
		sleep(2); // 8s for syn
	}
}

int OnlineSLAMServer::WaitingforSnapFramesSync(void*pContent)
{
	OnlineSLAMServer *pCOnlineSLAMServer=(OnlineSLAMServer *)pContent;
	int nCount=0,i,nHasSyncNum=0,nRtn=-1;;
	while(nCount<100) // 30 * 100ms = 1s
	{
		nHasSyncNum=0;
		for(i=0;i<6;i++)
		{
			if(m_bSnapSync[i])
			{
                printf("ID:%d Has Been Sync!!!!\n",i);
				nHasSyncNum++;
			}
		}
		if(nHasSyncNum==pCOnlineSLAMServer->m_nCurCncClientNum)
		{
            printf("Sync Done!!!!!!!\n");
			nRtn=0;
			break;
		}
		nCount++;
		usleep(100000); 
	}
    if(nRtn==-1)
    {
        printf("Sync Failed!!!!!!\n");
    }
	return nRtn;
}

int OnlineSLAMServer::SendSnapCMD(void*pContent)
{
	pthread_mutex_lock(&g_MutexRegister);
	OnlineSLAMServer *pCOnlineSLAMServer=(OnlineSLAMServer *)pContent;
	char cData[16],*pcSnapCMD="Snap";
	int i;
	memset(cData,0,16);
	memcpy(cData,pcSnapCMD,4);

	for(i=0;i<SUPPORT_CLEINT_NUM;i++)
	{
		if(m_pCOnlineSLAMServer->m_stClientInfo[i].bHasCnc)
		{
			m_pCOnlineSLAMServer->m_CExternalCom.SendData2Robot
			(4,cData,m_pCOnlineSLAMServer->m_stClientInfo[i].nSock);
		}
	}
	pthread_mutex_unlock(&g_MutexRegister);
}

void* OnlineSLAMServer::ThreadExternalComRun(void* lpParam)
{
	OnlineSLAMServer *pCOnlineSLAMServer=(OnlineSLAMServer *)lpParam;
	pCOnlineSLAMServer->m_CExternalCom.ComRun();
}

int OnlineSLAMServer::RenderSnapFrame()
{
	return 0;
}

void* OnlineSLAMServer::Thread3DRender(void* lpParam)
{
	OnlineSLAMServer *pCOnlineSLAMServer=(OnlineSLAMServer *)lpParam;
	pCOnlineSLAMServer->m_C3DRender.MapRenderRun();
}

int OnlineSLAMServer::OnlineSLAMServerRun()
{
	m_bStopRenderSnapFrame=false;
	m_bStop3DRender=false;
	pthread_create(&m_hThreadExternalComRun,
			NULL,
			ThreadExternalComRun,this);


	pthread_create(&m_hThreadRenderSnapFrame,
			NULL,
			ThreadRenderSnapFrame,this);

	pthread_create(&m_hThread3DRender,
			NULL,
			Thread3DRender,this);
	return 0;
}


int OnlineSLAMServer::OnlineSLAMServerUninit()
{
	return 0;
}

int main()
{
	signal(SIGPIPE, SIG_IGN);
	OnlineSLAMServer COnlineSLAMServer;

	COnlineSLAMServer.OnlineSLAMServerInit();
	printf("sys init!!!\n");
	COnlineSLAMServer.OnlineSLAMServerRun();
	printf("sys run!!!\n");
	while(1)
	{
		sleep(200000);
	}
	printf("sys out!!!\n");
	return 0;
}
