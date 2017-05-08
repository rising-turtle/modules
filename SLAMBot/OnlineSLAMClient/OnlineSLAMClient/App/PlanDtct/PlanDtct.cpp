#include "PlanDtct.h"




#define  PI 3.1415926
#define DownSampleScale 4
#define DepthSize 640/DownSampleScale*480/DownSampleScale
#define RGBSize 640/DownSampleScale*480/DownSampleScale*3

#define RGB_SIZE 307200
#define RGB_SIZE_3X 921600
#define DEPTH_SIZE 307200
#define DEPTH_SIZE_2X 614400

#define WIDTH 640
#define HEIGHT 480

#define RGB_WIDTH_SRIDE 12
#define RGB_HEIGHT_SRIDE (DownSampleScale-1)*WIDTH*3
#define DEPTH_HEIGHT_SRIDE (DownSampleScale-1)*WIDTH
PlanDtct::PlanDtct()
{
	m_bNewDataIn=false;
	m_bShowRslt=false;
}

PlanDtct::~PlanDtct()
{

}

int PlanDtct::NewDataIn(unsigned char* pucRGB,unsigned short *pusDepth)
{
	memcpy(m_pucRGB,pucRGB,RGB_SIZE_3X);
	memcpy(m_pusDepth,pusDepth,DEPTH_SIZE_2X);
	m_bNewDataIn=true;
	return 0;
}
int PlanDtct::AppInit(void *pcParams)
{
	ProcINSMonitor.MemberPorc = &INSMonitor::Run;
	m_pusDepthDownSample=new unsigned short[DepthSize];
	m_pucRGBDownSample = new unsigned char[RGBSize];
	m_pucRGB=new unsigned char[RGB_SIZE_3X];
	m_pusDepth=new unsigned short[DEPTH_SIZE];
	m_pucRGBView=new unsigned char[RGB_SIZE_3X];
	return 0;
}
int PlanDtct::AppRun(void *pcParams)
{
	int i,j,nCount,nIdx;

	unsigned short *pusDepthDownSample=NULL;
	unsigned char *pucRGBDownSample=NULL;
	unsigned short *pusDepth=NULL;
	unsigned char *pucRGB=NULL;

	PlanarDetector MultiPlane;
	float fSensorPose[3];

	m_bStopPlanDtct=false;

	while (!m_bStopPlanDtct)
	{
		if (m_bNewDataIn)
		{
			pusDepthDownSample=m_pusDepthDownSample;
			pucRGBDownSample=m_pucRGBDownSample;
			pucRGB=m_pucRGB;
			pusDepth=m_pusDepth;

			
			for (i = 0; i < HEIGHT; i+=DownSampleScale)
			{
				for (j = 0; j < WIDTH; j+=DownSampleScale)
				{
					*(pusDepthDownSample++)=*pusDepth;
					pusDepth+=DownSampleScale;

					memcpy(pucRGBDownSample,pucRGB,3);
					pucRGBDownSample+=3;
					pucRGB+=RGB_WIDTH_SRIDE;
				}			
				pusDepth+=DEPTH_HEIGHT_SRIDE;
				pucRGB+=RGB_HEIGHT_SRIDE;
			}


			pusDepthDownSample=m_pusDepthDownSample;
			pucRGBDownSample=m_pucRGBDownSample;
			pucRGB=m_pucRGB;
			pusDepth=m_pusDepth;

			fSensorPose[0] = shareImu2SlamData.roll;  //pitch
			fSensorPose[1] = shareImu2SlamData.yaw;//yaw
			fSensorPose[2] = shareImu2SlamData.pitch;//roll

			fSensorPose[0] = fSensorPose[0]*PI/180;
			fSensorPose[1] = fSensorPose[1]*PI/180;
			fSensorPose[2] = fSensorPose[2]*PI/180;

			MultiPlane.run(pucRGBDownSample,pusDepthDownSample,fSensorPose);
			if (MultiPlane.planeContours.size()>0)
			{
				memcpy(m_pucRGBView,m_pucRGB,RGB_SIZE_3X);
			}
			for (i= 0; i<MultiPlane.planeContours.size();i++)
			{
				
				//printf("detect plane!!!!!!!!!!!!!!!!!!!!!!1\n");
				PlaneContour * plane = NULL;//new PlaneContour();
				plane = MultiPlane.planeContours[i];
				if (plane->planepointnum>0)
				{
					if (plane->planepointnum<10000)
					{
						nCount = plane->planepointnum;
					}
					else
					{
						nCount = 10000;
					}
					for (j=0;j<nCount;j++)
					{
						// is floor red  ,else green              
						int temp_x, temp_y, xx, yy;
						int band = 2;
						if((plane->ContourXY[j].x<640)&&(plane->ContourXY[j].y<480) && !plane->isFloor)
						{


							for (xx=-band; xx<=band; xx++)
							{
								for (yy=-band; yy<=band; yy++)
								{
									temp_x = plane->ContourXY[j].x + xx;
									temp_y = plane->ContourXY[j].y + yy;
									if (temp_x<0 || temp_x>639 || temp_y<0 || temp_y>479)
									{
										continue;
									}
									nIdx=(temp_y*WIDTH+temp_x)*3;
									m_pucRGBView[nIdx]=0;
									m_pucRGBView[nIdx+1]=255;
									m_pucRGBView[nIdx+2]=0;
								}									
							}			
							
						}
						if((plane->ContourXY[j].x<640)&&(plane->ContourXY[j].y<480) && plane->isFloor)
						{
							for (xx=-band; xx<=band; xx++)
							{
								for (yy=-band; yy<=band; yy++)
								{
									temp_x = plane->ContourXY[j].x + xx;
									temp_y = plane->ContourXY[j].y + yy;
									if (temp_x<0 || temp_x>639 || temp_y<0 || temp_y>479)
									{
										continue;
									}
									nIdx=(temp_y*WIDTH+temp_x)*3;
									m_pucRGBView[nIdx]=255;
									m_pucRGBView[nIdx+1]=0;
									m_pucRGBView[nIdx+2]=0;
								}									
							}

						}

					}
				}
			}
			m_bNewDataIn=false;
		}
		if (m_bShowRslt)
		{
			m_cbImgView(m_pucRGBView);
		}
		
	}

	return 0;
}
int PlanDtct::AppStop(void *pcParams)
{
	m_bStopPlanDtct=true;
	return 0;
}
int PlanDtct::AppUninit(void *pcParams)
{
	delete [] m_pusDepthDownSample;
	delete [] m_pucRGBDownSample;
	delete [] m_pucRGB;
	delete [] m_pusDepth;
	delete [] m_pucRGBView;
	return 0;
}

int PlanDtct::ShowRslt()
{	
	m_bShowRslt=true;
	return 0;
}
int PlanDtct::StopShowRslt()
{
	m_bShowRslt=false;
	return 0;
}