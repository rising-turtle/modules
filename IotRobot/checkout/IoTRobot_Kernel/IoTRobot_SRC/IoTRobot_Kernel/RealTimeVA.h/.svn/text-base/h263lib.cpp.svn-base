#include "h263lib.h"

void h263lib::Initialization()
{
	InitLookupTable();
	h263Param.format = 1;
	InitH263Encoder(&h263Param);
	//WriteByteFunction = this->OwnWriteFunction;
	InitH263Decoder();
}

h263lib::~h263lib()
{
	ExitH263Decoder();
	ExitH263Encoder(&h263Param);
}



//*************************************
// h263 encoder callback routine
//*************************************
void h263lib::OwnWriteFunction(int byte)
{
	if(ByteCount < cbuffer_size)
	{
		this->cdata[ByteCount]=(unsigned char)byte;
		++this->ByteCount;
	}
}

