#pragma once
#include <stdio.h>
#include <iostream>
using namespace std;
#include "../JPEG/jpeglib.h"



class jpegClass
{
public:		//variables

	//methods
	jpegClass();
	~jpegClass();
	void jpgToRGB(unsigned char * jpgData,
				  unsigned long jpgSize, 
				  unsigned char * RGBdata, 
				  unsigned int* pRGBsize);

	void jpegClass::RGBToJpg(unsigned int width,
							unsigned int height,
							unsigned char* RGBdata,
							unsigned char** jpgData, 
							unsigned long * jpgSize,
							int quality);

private:	//variables

private:	//methods


};
