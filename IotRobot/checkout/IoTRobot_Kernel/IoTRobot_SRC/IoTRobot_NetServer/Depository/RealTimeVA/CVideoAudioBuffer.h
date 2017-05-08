/**********************************************************
// Name			: CVideoAudioBuffer.h
// Description	: define a CVideoAudioBuffer class
// Version		: v0.2
//---------------------------------------------------------
// This class 
**********************************************************/
#include <Windows.h>
#include <queue>
#include <assert.h>
#include <iostream>
using namespace std;


struct elementInBufferInfoUnit {
	UINT	data_head;		//image's beginning position in the buffer  
	UINT	data_size;		//ending position in buffer of one image
	BOOL	isBlockCrossBorder;
	UINT32	timestamp;	
	//Maybe some new items will be added here later, depends on experiment result
};


class CVideoAudioBuffer
{
public:	//methods
	CVideoAudioBuffer(UINT bufferSize, UINT maxFramesInBuffer);	//constructor
	~CVideoAudioBuffer();	//destructor

	//-----------------------------------------------
	// Return next image in the buffer
	//-----------------------------------------------
	BOOL getNextImageInBuffer(BYTE* dataRead, UINT* dataSizeRead, UINT32* timestamp);

	//-----------------------------------------------
	// put an image into the buffer
	//-----------------------------------------------
	BOOL putOneImageIntoBuffer(BYTE* dataTobeWrote, UINT dataSizeTobeWrote, UINT32 timestamp);

	//-----------------------------------------------
	// Return the size of free space in the buffer
	//-----------------------------------------------
	UINT getFreeSpaceSizeInBuffer();

	//-----------------------------------------------
	// 
	//-----------------------------------------------
	UINT getFrameCountInBuffer();

	//-----------------------------------------------
	// 
	//-----------------------------------------------
	INT getActiveInfoUnitCount();

	//-----------------------------------------------
	// 
	//-----------------------------------------------
	BOOL isImgQueueEmpty();



private:	//variables


	BYTE* img263Buffer;	//buffer for storing received images

	BOOL	isBufferBorderCrossed;
	UINT	img263BufferData_head;		//beginning of image data in buffer
	UINT	freespace_head;			//end of image data in buffer
	queue<elementInBufferInfoUnit *> imageInfoUnitQueue;
	INT		activeInfoUnitCount;
	HANDLE	hBufferMutex;		//Mutex for buffer reading and writing syc
	UINT	bufferSize;
	UINT	maxFramesInBuffer;


private:	//methods


	//--------------------------------------------------------
	// Write image data to buffer
	//--------------------------------------------------------
	BOOL writeImageIntoImgBuffer(elementInBufferInfoUnit *imageToWrite, 
		BYTE* dataTobeWrote);

	//---------------------------------------------------------
	// Read image data from buffer. 
	//---------------------------------------------------------
	BOOL readImageFromImgBuffer(elementInBufferInfoUnit *imageBeRead,
		BYTE* dataRead, 
		UINT* dataSizeRead
		);
	//-----------------------------------------------
	// Initialization
	//-----------------------------------------------
	void Initialization();

	//-----------------------------------------------
	// Check if the buffer in a good condition
	//-----------------------------------------------
	BOOL checkBufferIfNormal();


	//-----------------------------------------------
	// Delete oldest image in the buffer
	//-----------------------------------------------
	// If the buffer is full before writing a new image,
	// we should delete the oldest images in the buffer.
	// run this routine once, results in the oldest image
	// be deleted.
	//-----------------------------------------------
	void deleteOldestImageInBuffer(elementInBufferInfoUnit * imageToDeleteInfo);

	//-----------------------------------------------
	// See if next image to be write into buffer 
	// need store it cross the border
	//-----------------------------------------------
	BOOL doesNeedCrossBorder(UINT dataSizeTobeWrote);

	//-----------------------------------------------
	// 
	//-----------------------------------------------
	BOOL isImgBufferEmpty();

};
