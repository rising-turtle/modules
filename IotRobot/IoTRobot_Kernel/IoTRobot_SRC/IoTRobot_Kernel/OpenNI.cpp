#include "OpenNI.h"

/*#include <vtkConeSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkImageViewer.h>
#include <../io/vtkBMPReader.h>
#include <../Filtering/vtkImageData.h>
#include <../Filtering/vtkPointData.h>

#include <vtkGraphicsFactory.h>
#include <vtkWin32RenderWindowInteractor.h>
#include <vtkCommand.h>*/
COpenNI::COpenNI()
{

}

COpenNI::~COpenNI()
{

}

IoT_CallBackFuncSet COpenNI::m_stCallBackFuncSet;
IoTRobot_NetServer_Interface COpenNI::m_cNetServer;

/*vtkImageViewer *g_vtkImageViewer;
vtkBMPReader *g_vtkBMPReader;
vtkImageData *g_vtkImageData;
unsigned char *g_RGBBuff;*/


int g_fisrt=0;
/*boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > cloud(new pcl::PointCloud<pcl::PointXYZRGB > ());
pcl::PointCloud<pcl::PointXYZRGB>::Ptr COpenNI::IoT_ConvertToXYZRGBPointCloud(unsigned char*pucRGB,unsigned short *pusDepth) const
{
	static unsigned rgb_array_size = 0;
	//static boost::shared_array<unsigned char> rgb_array(0);
	static unsigned char* rgb_buffer = 0;

	

	cloud->header.frame_id = "ss";
	cloud->height = 480;
	cloud->width = 640;
	cloud->is_dense = false;

	cloud->points.resize(cloud->height * cloud->width);

	//float constant = 1.0f / device_->getImageFocalLength(cloud->width);
	register int centerX = (cloud->width >> 1);
	int centerY = (cloud->height >> 1);

	register const XnDepthPixel* depth_map = pusDepth;


	// here we need exact the size of the point cloud for a one-one correspondence! 


	rgb_buffer=pucRGB;
	// depth_image already has the desired dimensions, but rgb_msg may be higher res.
	register int color_idx = 0, depth_idx = 0;
	//pcl::RGBValue color;
	//color.Alpha = 0;
	unsigned char r = 0, g = 0, b = 0;    // Example: Red color

	float bad_point = std::numeric_limits<float>::quiet_NaN();

	for (int v = -centerY; v < centerY; ++v)
	{
		for (register int u = -centerX; u < centerX; ++u, color_idx += 3, ++depth_idx)
		{
			pcl::PointXYZRGB& pt = cloud->points[depth_idx];
			/// @todo Different values for these cases
			// Check for invalid measurements
			if (depth_map[depth_idx] == 0 )
			{
				pt.x = pt.y = pt.z = bad_point;
			}
			else
			{
				pt.z = depth_map[depth_idx] * 0.001f;
				pt.x = 0.001f*u * pt.z ;
				pt.y =  0.001f*v * pt.z;
			}

			// Fill in color
			pt.r=255;
			pt.g=255;
			pt.b=255;
			r = rgb_buffer[color_idx];
			g = rgb_buffer[color_idx + 1];
			b = rgb_buffer[color_idx + 2];
			//pt.rgb = ((unsigned int )r << 16 | (unsigned int )g << 8 | (unsigned int )b);
		}
	}
	return (cloud);
}

*/
unsigned char *pDepthRGB=new unsigned char[640*480*3];

#include "pcl/common/common_headers.h"
boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > cloud(new pcl::PointCloud<pcl::PointXYZRGB > ());

void COpenNI::RGB_Depth_cb(unsigned char*pucRGB,unsigned short *pusDepth,void *pContext)
{
	typedef union
	{

		struct /*anonymous*/
		{
			unsigned char Blue;
			unsigned char Green;
			unsigned char Red;
			unsigned char Alpha;
		};
		float float_value;
		long long_value;
	} RGBValue;
	static unsigned rgb_array_size = 0;
	static unsigned char* rgb_buffer = 0;

	cloud->header.frame_id = "/openni_rgb_optical_frame";;
	cloud->height = 480;
	cloud->width = 640;
	cloud->is_dense = false;

	cloud->points.resize(cloud->height * cloud->width);

	//float constant = 1.0f / device_->getImageFocalLength(cloud->width);
	register int centerX = (cloud->width >> 1);
	int centerY = (cloud->height >> 1);

	register const XnDepthPixel* depth_map = pusDepth;

	rgb_buffer=pucRGB;
	// depth_image already has the desired dimensions, but rgb_msg may be higher res.
	register int color_idx = 0, depth_idx = 0;
	RGBValue color;
	color.Alpha = 0;
	unsigned char r = 0, g = 0, b = 0;    // Example: Red color

	float bad_point = std::numeric_limits<float>::quiet_NaN();

	for (int v = -centerY; v < centerY; ++v)
	{
		for (register int u = -centerX; u < centerX; ++u, color_idx += 3, ++depth_idx)
		{
			pcl::PointXYZRGB& pt = cloud->points[depth_idx];
			if (depth_map[depth_idx] == 0 )
			{
				pt.x = pt.y = pt.z = bad_point;
			}
			else
			{
				cloud->points[depth_idx].z=depth_map[depth_idx] * 0.001f;
				cloud->points[depth_idx].x=0.001904*u *cloud->points[depth_idx].z;
				cloud->points[depth_idx].y=0.001904*v *cloud->points[depth_idx].z;
				cloud->points[depth_idx].r=rgb_buffer[color_idx];
				cloud->points[depth_idx].g=rgb_buffer[color_idx+1];
				cloud->points[depth_idx].b=rgb_buffer[color_idx+2];

			}
		}
	}

	if(m_stCallBackFuncSet.callBack_XYZRGB!=NULL)
	m_stCallBackFuncSet.callBack_XYZRGB(cloud,NULL);

	if (m_stCallBackFuncSet.callBack_RGB24!=NULL)
	{
		m_stCallBackFuncSet.callBack_RGB24(pucRGB,NULL);
	}
	if (m_stCallBackFuncSet.callBack_RGB24_Depth!=NULL)
	{
		m_stCallBackFuncSet.callBack_RGB24_Depth(pucRGB,pusDepth,NULL);
	}
}

void COpenNI::OpenNIInit(IoT_CallBackFuncSet callBackSet)
{
	IoT_NetCallBackSet stNetServerCallBack;
	m_stCallBackFuncSet=callBackSet;
	stNetServerCallBack.cbDisplay=m_stCallBackFuncSet.callBack_RGB24;
	stNetServerCallBack.cbSlam=m_stCallBackFuncSet.callBack_RGB24_Depth;
	stNetServerCallBack.cbSlam_IMU=m_stCallBackFuncSet.cbSlam_IMU;
	m_cNetServer.NetServer_Init((void*)&stNetServerCallBack);
	//printf("hahahaha");
}


void COpenNI::Cloud_cb (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud)
{

	if(m_stCallBackFuncSet.callBack_XYZRGB!=NULL)
	m_stCallBackFuncSet.callBack_XYZRGB(cloud,m_stCallBackFuncSet.pXYZRGBContext);
}
void COpenNI::Image_cb (const boost::shared_ptr<openni_wrapper::Image>& img)
{
	if(m_stCallBackFuncSet.callBack_RGB!=NULL)
	m_stCallBackFuncSet.callBack_RGB(img,m_stCallBackFuncSet.pRGBContext);
}

UINT COpenNI::ThreadNetServer(LPVOID lpParam)
{

	m_cNetServer.NetServer_Run();
	return 0;
}

int COpenNI::GetOneFrame()
{
	return m_cNetServer.NetServer_GetOneSlamFrame(0);
}

UINT ThreadSendConf(LPVOID lpParam)
{
	COpenNI *pOpenNi=(COpenNI *)lpParam;
	IoT_NetConf stMetConf;
	stMetConf.stSLAMConf.cCompression=0;
	stMetConf.stSLAMConf.cFrequency=25;
	stMetConf.stSLAMConf.cResolution=0;
	stMetConf.stSLAMConf.cType=2;

	stMetConf.stVisulizationConf.cCompression=0;
	stMetConf.stVisulizationConf.cFrequency=60;
	stMetConf.stVisulizationConf.cResolution=0;
	stMetConf.stVisulizationConf.cType=0;

	while (pOpenNi->m_cNetServer.NetServer_Conf(0,(void*)&stMetConf)!=0)
	{
		Sleep(30);
	}
	return 0;
}

void COpenNI::OpenNIRun()
{
	LPDWORD ID=0;
	CreateThread(NULL,0,(LPTHREAD_START_ROUTINE)ThreadNetServer,NULL,0,ID);	
	CreateThread(NULL,0,(LPTHREAD_START_ROUTINE)ThreadSendConf,this,0,ID);	


/*	IoT_NetConf stMetConf;
	stMetConf.stSLAMConf.cCompression=0;
	stMetConf.stSLAMConf.cFrequency=25;
	stMetConf.stSLAMConf.cResolution=0;
	stMetConf.stSLAMConf.cType=0;

	stMetConf.stVisulizationConf.cCompression=0;
	stMetConf.stVisulizationConf.cFrequency=60;
	stMetConf.stVisulizationConf.cResolution=0;
	stMetConf.stVisulizationConf.cType=0;*/


}

void COpenNI::OpenNIUnint()
{
	m_pcPCLGrabber->stop();
	delete m_pcPCLGrabber;
}

int COpenNI::GetDeviceState(int *pnState)
{
	return m_cNetServer.NetServer_GetDeviceState(pnState);
}