#include "ReadGroundTrues.h"
void ReadGroundTrues::calPCfromImageAndDepth(cv::Mat& image, cv::Mat& depth, \
							pcl::PointCloud<pcl::PointXYZRGB>::Ptr& outPC)
{
	outPC->header.frame_id = "/openni_rgb_optical_frame";// rgb_frame_id_;

	// just guess
	int depth_width_ = 640;
	int depth_height_ = 480;
	int image_width_ = 640;
	int image_height_ = 480;

	outPC->height = depth_height_;
	outPC->width = depth_width_;
	outPC->is_dense = false;

	outPC->points.resize(outPC->height * outPC->width);

	register int centerX = (outPC->width >> 1);
	int centerY = (outPC->height >> 1);
	// 这是 1/f of Kinect
	static const double constant=0.0019047619;

	// depth_image already has the desired dimensions, but rgb_msg may be higher res.
	register int color_idx = 0, depth_idx = 0;

	float bad_point = std::numeric_limits<float>::quiet_NaN();

	register int wi=0,ci=0,cj=0;

	for (int v = -centerY; v < centerY; ++v,++wi)
	{
		for (register int u = -centerX; u < centerX; ++u,cj+=3,ci++,depth_idx++)
		{
			pcl::PointXYZRGB& pt = outPC->points[depth_idx];
			/// @todo Different values for these cases
			// Check for invalid measurements
			if (depth.at<unsigned short>(wi,ci) == 0)
				/*depth[depth_idx] == depth_image->getNoSampleValue() ||
				depth[depth_idx] == depth_image->getShadowValue())*/
			{
				pt.x = pt.y = pt.z = bad_point;
			}
			else
			{
				pt.z = depth.at<unsigned short>(wi,ci)* 0.001f;
				pt.x = u * pt.z * constant;
				pt.y = v * pt.z * constant;
			}

			// Fill in color
			pt.r = image.at<unsigned char>(wi,cj);
			pt.g = image.at<unsigned char>(wi,cj + 1);
			pt.b = image.at<unsigned char>(wi,cj + 2);
		}
		cj=0;
		ci=0;
	}
}

ReadGroundTrues::ReadGroundTrues(void)
{
}

ReadGroundTrues::~ReadGroundTrues(void)
{
}


int ReadGroundTrues::CreateFileList(vector<string> &vctFileList,char *pcPath,char *pcMark,char *pcSuffix)
{
	//char *pcPath="depth.txt";
	FILE * pFile;  
	int nLen1,nLen2;
	char *pcFileBuff1,*pcFileBuff2;
	size_t result;
	int i,j,k;
	pFile = fopen (pcPath, "rb" );  

	char cName[1000];
	char cMark[10];
	memset(cMark,0,10);
	memcpy(cMark,pcMark,strlen(pcMark));
	char cSuffix[10];
	memset(cSuffix,0,10);
	memcpy(cSuffix,pcSuffix,strlen(pcSuffix));
	

	fseek (pFile , 0 , SEEK_END);  
	nLen1 = ftell (pFile);  //tell the pointer drift number
	rewind (pFile);  //re-point to buff head

	//* 分配内存存储整个文件 
	pcFileBuff1 = (char*) malloc (sizeof(char)*nLen1);  
	pcFileBuff2 = (char*) malloc (sizeof(char)*nLen1); 
	memset(pcFileBuff1,0,sizeof(char)*nLen1);
	if (pcFileBuff1 == NULL)  
	{  
		return false;  
	}  

	//* 将文件拷贝到buffer中 
	result = fread (pcFileBuff1,1,nLen1,pFile); 


	for (i=0;i<nLen1;i++)
	{
		if(memcmp(&pcFileBuff1[i],cMark,strlen(cMark))==0)
		{
			memset(cName,0,1000);
			memcpy(cName,cMark,strlen(cMark));
			k=strlen(cMark);
			i+=strlen(cMark);
			for (j=i;j<nLen1-i;j++)
			{
				if(memcmp(&pcFileBuff1[j],cSuffix,strlen(cSuffix))==0)
				{
					memcpy(&cName[k],cSuffix,strlen(cSuffix));
					vctFileList.push_back(cName);
					break;
				}
				cName[k]=pcFileBuff1[j];
				k++;
			}
		}
	}

	fclose(pFile);
	delete [] pcFileBuff1;
	delete [] pcFileBuff2;
	return 0;
}

int ReadGroundTrues::Init()
{
	CreateFileList(m_vctDFileList,"depth.txt","depth/",".png");
	CreateFileList(m_vctRGBFileList,"rgb.txt","rgb/",".png");
	return 0;
}


int ReadGroundTrues::Run(unsigned char *pucRGB,unsigned short *pusD,boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > &m_PC)
{
	static int nCount=0;
	if (nCount<m_vctDFileList.size())
	{
		cv::Mat image;
		cv::Mat depth;
		image=cv::imread(m_vctRGBFileList[nCount]);
		depth=cv::imread(m_vctDFileList[nCount],2);

		memcpy(pucRGB,image.data,640*480*3);
		memcpy(pusD,depth.data,640*480*2);

		calPCfromImageAndDepth(image,depth,m_PC);
		nCount++;
	}
	else return -1;
}



