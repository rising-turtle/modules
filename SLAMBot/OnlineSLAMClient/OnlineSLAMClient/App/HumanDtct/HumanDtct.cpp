#include "HumanDtct.h"
#include <process.h>
#include <windows.h>

CRITICAL_SECTION g_cs2rdView;
#define DEPTH_SIZE 307200
#define RGB_SIZE_3X 921600


HumanDtct::HumanDtct()
{
	m_bNewDataIn=false;
	m_bShowRslt=false;
}

HumanDtct::~HumanDtct()
{

}

int HumanDtct::ShowRslt()
{	
	m_bShowRslt=true;
	return 0;
}
int HumanDtct::StopShowRslt()
{
	m_bShowRslt=false;
	return 0;
}
int HumanDtct::NewDataIn(unsigned char* pucRGB,unsigned short *pusDepth)
{
	m_bNewDataIn=true;
	return 0;
}
int HumanDtct::AppInit(void *pcParams)
{
	m_pucRGB=new unsigned char[RGB_SIZE_3X];
	return 0;
}
int HumanDtct::AppRun(void *pcParams)
{

	XnUInt16 nUserCount;
	int i,j;
	m_bStopHumanDtct=false;
	SceneMetaData stSceneMetaData;
	unsigned short *pusDepth;
	
	memset(m_pucRGB,0,RGB_SIZE_3X);
	unsigned char *pucTmpRGB=m_pucRGB;
	while (!m_bStopHumanDtct)
	{
		if (m_bNewDataIn)
		{
			nUserCount = m_pCUserGenerator->GetNumberOfUsers();
			memset(m_pucRGB,0,RGB_SIZE_3X);
			//EnterCriticalSection(&g_cs2rdView);
			if (nUserCount>0)
			{
			//	printf("human detect :%d!!!\n",nUserCount);
				int counter = 0;
				XnUserID * userID = new XnUserID[nUserCount];
				m_pCUserGenerator->GetUsers(userID, nUserCount);
		
				
				for (i = 0; i < nUserCount; i++)
				{
					m_pCUserGenerator->GetUserPixels(userID[i],stSceneMetaData);
					pusDepth=(unsigned short*)stSceneMetaData.Data();
					pucTmpRGB=m_pucRGB;
					for (j=0;j<DEPTH_SIZE;j++)
					{
						if (*pusDepth)
						{
							*(pucTmpRGB++)=255;
							*(pucTmpRGB++)=255;
							*(pucTmpRGB++)=255;
						}
						else
						{
							pucTmpRGB+=3;
						}
						pusDepth++;
					}
					
				}
				delete[] userID;
				userID = NULL;
				
			}
			m_bNewDataIn=false;
		}
		if (m_bShowRslt)
		{
			m_cbImgView(m_pucRGB);
		}
	}
	
	return 0;
}
int HumanDtct::AppStop(void *pcParams)
{
	m_bStopHumanDtct=false;
	return 0;
}
int HumanDtct::AppUninit(void *pcParams)
{
	delete [] m_pucRGB;
	return 0;
}