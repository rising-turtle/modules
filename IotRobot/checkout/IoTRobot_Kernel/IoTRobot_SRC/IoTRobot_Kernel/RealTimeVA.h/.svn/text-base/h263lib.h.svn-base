#include "encoder/libr263.h"
#include "decoder/Tmndec.h"

class h263lib
{
public:
	h263lib();
	~h263lib();

	void Initialization();

private:
	CParam h263Param;
	Bits bits;
	size_t dataLen;
	unsigned int ByteCount;

	unsigned char cdata[20000];
	int cbuffer_size;
	unsigned char rgbdata[400000];
	int buffersize;

	//--------------------- h263 ---------------------
	unsigned char *readData;
	unsigned char *imgBuffer;
	//unsigned int yuv[WIDTH*HEIGHT*3/2];

	void OwnWriteFunction(int byte);

};