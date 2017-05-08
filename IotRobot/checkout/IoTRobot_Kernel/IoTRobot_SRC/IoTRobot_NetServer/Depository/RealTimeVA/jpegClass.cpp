#include "jpegClass.h"

/*--------------------------------------------------
//
---------------------------------------------------*/
jpegClass::jpegClass()
{
	

}
/*--------------------------------------------------
//
---------------------------------------------------*/
jpegClass::~jpegClass()
{

}

/*--------------------------------------------------
//
---------------------------------------------------*/
void jpegClass::jpgToRGB(unsigned char * jpgData, 
						 unsigned long jpgSize, 
						 unsigned char * RGBdata, 
						 unsigned int * pRGBsize)
{
	// 声明解压缩对象及错误信息管理器
	struct jpeg_decompress_struct cinfo;
	struct jpeg_error_mgr jerr;
	JSAMPROW row_pointer[1];

	cinfo.err = jpeg_std_error(&jerr);

	//初始化jpeg解压对象
	jpeg_create_decompress(&cinfo);
	//绑定jpeg解压对象到输入流
	jpeg_mem_src(&cinfo, jpgData, jpgSize);

	//读取jpeg头部信息:
	jpeg_read_header(&cinfo, TRUE);

	//开始解压
	jpeg_start_decompress(&cinfo);

	/*
	调用这个函数之后,可以得到jpeg图像的下面几个参数:
	1.output_width // 图像的宽度
	2.output_height // 图像的高度
	3.output_components // 每个像素占用的字节数

	我们采用每扫描一行像素就输出到屏幕的方法的话,
	根据以上参数可以确定分配一行信息需要的缓冲区:
	buffer = (unsigned char *)malloc(cinfo.output_width *
	cinfo.output_components); 
	总共需要扫描output_height行.
	*/

	//读取一行
	int scanIndex = 0;
	while (cinfo.output_scanline < cinfo.output_height) {
		row_pointer[0] = &RGBdata[scanIndex * cinfo.output_width * cinfo.output_components];
		jpeg_read_scanlines(&cinfo, row_pointer, 1);
		scanIndex++;
	}

	*pRGBsize = scanIndex * cinfo.output_width * cinfo.output_components;

	//结束jpeg解码
	jpeg_finish_decompress(&cinfo);
	jpeg_destroy_decompress(&cinfo);
}

void jpegClass::RGBToJpg(unsigned int width,
						 unsigned int height,
						 unsigned char* RGBdata,
						 unsigned char** jpgData, 
						 unsigned long * jpgSize,
						 int quality)
{
	struct jpeg_compress_struct cinfo;
	struct jpeg_error_mgr jerr;
	JSAMPROW row_ptr[1];
	int row_stride;

	*jpgSize = 0;

	cinfo.err = jpeg_std_error(&jerr);
	jpeg_create_compress(&cinfo);
	jpeg_mem_dest(&cinfo, jpgData, jpgSize);

	cinfo.image_width = width;
	cinfo.image_height = height;
	cinfo.input_components = 3;	//每个像素占用的字节数
	cinfo.in_color_space = JCS_RGB;

	jpeg_set_defaults(&cinfo);
	jpeg_set_quality(&cinfo, quality, true);
	jpeg_start_compress(&cinfo, TRUE);
	row_stride = width * 3;

	while (cinfo.next_scanline < cinfo.image_height) {
		row_ptr[0] = &RGBdata[cinfo.next_scanline * row_stride];
		jpeg_write_scanlines(&cinfo, row_ptr, 1);
	}

	jpeg_finish_compress(&cinfo);
	jpeg_destroy_compress(&cinfo);
}