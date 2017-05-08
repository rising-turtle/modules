/**********************************************************
// Name			: CVideoAudioBuffer.cpp
// Description	: Implement the CVideoAudioBuffer class
// Version		: v0.2
**********************************************************/
//#include "stdafx.h"
#include "CVideoAudioBuffer.h"


//-----------------------------------------------
// Constructor
//-----------------------------------------------
CVideoAudioBuffer::CVideoAudioBuffer(UINT bufferSize, UINT maxFramesInBuffer)
{
	this->maxFramesInBuffer = maxFramesInBuffer;
	this->bufferSize		= bufferSize;
	this->Initialization();
}

//-----------------------------------------------
// Initialization
//-----------------------------------------------
void CVideoAudioBuffer::Initialization()
{
	this->isBufferBorderCrossed = false;
	this->img263BufferData_head = 0;		//beginning of image data in buffer
	this->freespace_head = 0;			//end of image data in buffer


	this->img263Buffer = new BYTE[this->bufferSize];

	activeInfoUnitCount = 0;
	hBufferMutex = CreateMutex(NULL, FALSE, NULL);

}

//-----------------------------------------------
// Destructor
//-----------------------------------------------
CVideoAudioBuffer::~CVideoAudioBuffer()
{
	//release infoUnit one by one
	while (!imageInfoUnitQueue.empty())
	{
		elementInBufferInfoUnit * tmpPointer = this->imageInfoUnitQueue.front();
		this->imageInfoUnitQueue.pop();
		delete tmpPointer;
		this->activeInfoUnitCount--;
	}

	//release the image buffer

	if (this->img263Buffer)
	{
		delete [] this->img263Buffer;
	}

}

//-----------------------------------------------
// Return the size of free space in the buffer
//-----------------------------------------------
UINT CVideoAudioBuffer::getFreeSpaceSizeInBuffer()
{
	/* NOTE : -------------------------------------------------------
	//				How to tell the buffer is empty or full
	// If img263BufferData_head == freespace_head, means either
	// buffer is empty or full. If isBufferBorderCrossed == false, 
	// means empty, else means full.
	// --------------------------------------------------------------*/

	/* NOTE : -------------------------------------------------------
	//				How to calculate free space size in buffer
	// There's a relation as following :
	//		If isBufferBorderCrossed == false, 
	//		"freeBytesInImg263Buffer = freespace_head - img263BufferData_head"
	//		If isBufferBorderCrossed == true, 
	//		"freeBytesInImg263Buffer = img263BufferData_head - freespace_head"
	// --------------------------------------------------------------*/

	//check if the buffer is empty or full
	if (this->img263BufferData_head == this->freespace_head)
	{
		if (!isBufferBorderCrossed)
			return this->bufferSize;
		else
			return 0;
	}
	else	//general case
	{
		int size;
		if (!isBufferBorderCrossed)
			size = (this->bufferSize - (this->freespace_head - this->img263BufferData_head));
		else
			size = (this->img263BufferData_head - this->freespace_head);
		assert(size > 0);
		return size;
	}
}


//-----------------------------------------------
// Return next image in the buffer
//-----------------------------------------------
BOOL CVideoAudioBuffer::getNextImageInBuffer(BYTE* dataRead, UINT* dataSizeRead, UINT32* pTimestamp)
{
	BOOL result;

	if (!dataSizeRead || !pTimestamp)
	{
		cout<< "position 100 false."<<endl;
		return false;
	}


	WaitForSingleObject(hBufferMutex, INFINITE);

	//make sure there're elements in queue
	if (this->imageInfoUnitQueue.empty())
	{
		cout<< "position 101 false."<<endl;
		result = false;
		goto routineExit;
	}

	//get a imageInfoUnit
	elementInBufferInfoUnit *nextImageInfo = this->imageInfoUnitQueue.front();
	this->imageInfoUnitQueue.pop();

	//if the imageInfoUnit is null, there's something wrong
	if (!nextImageInfo)
	{
		assert(false);
		cout<< "position 102 false."<<endl;
		result = false;
		goto routineExit;
	}

	//read from buffer
	result = readImageFromImgBuffer(nextImageInfo,
		dataRead,		//put image data in this buffer
		dataSizeRead	//return the size of the image
		);
	if(result)
		*pTimestamp = nextImageInfo->timestamp;
	delete nextImageInfo;	//delete the infoUnit
	this->activeInfoUnitCount--;

routineExit:
	ReleaseMutex(hBufferMutex);
	return result;

}

//-----------------------------------------------
// put an image into the buffer
//-----------------------------------------------
BOOL CVideoAudioBuffer::putOneImageIntoBuffer(BYTE* dataTobeWrote, 
											  UINT dataSizeTobeWrote, 
											  UINT32 timestamp)
{
	BOOL result;

	if (dataSizeTobeWrote > this->bufferSize)
	{
		assert(false);
		cout<< "position 01 false."<<endl;
		return false;
	}

	//create a infoUnit
	WaitForSingleObject(hBufferMutex, INFINITE);
	elementInBufferInfoUnit *currentImageInfoUnit = new elementInBufferInfoUnit;
	this->activeInfoUnitCount++;
	//assign values
	currentImageInfoUnit->data_head = this->freespace_head;
	currentImageInfoUnit->data_size = dataSizeTobeWrote;
	currentImageInfoUnit->timestamp = timestamp;

	if (this->getFreeSpaceSizeInBuffer() < dataSizeTobeWrote)
	{
		UINT maxDeleteCount = 50;
		while(maxDeleteCount != 0)
		{
			elementInBufferInfoUnit * imageToDeleteInfo;
			if (this->imageInfoUnitQueue.empty())
			{
				assert(false);
				cout<< "position 02 false."<<endl;
				result = false;
				goto routineExit;
			}
			imageToDeleteInfo = this->imageInfoUnitQueue.front();
			this->imageInfoUnitQueue.pop();

			deleteOldestImageInBuffer(imageToDeleteInfo);
			delete imageToDeleteInfo;	//delete the infoUnit
			this->activeInfoUnitCount--;

			if (this->getFreeSpaceSizeInBuffer() >= dataSizeTobeWrote)
			{
				break;
			}
			maxDeleteCount--;
		}
	}

	if (this->getFreeSpaceSizeInBuffer() < dataSizeTobeWrote)	
	{	//still not enough, then return false
		assert(false);
		delete currentImageInfoUnit;
		this->activeInfoUnitCount--;
		cout<< "position 03 false."<<endl;
		result = false;
		goto routineExit;
	}

	//see if need to cross buffer's border
	currentImageInfoUnit->isBlockCrossBorder = doesNeedCrossBorder(dataSizeTobeWrote);

	result = writeImageIntoImgBuffer(currentImageInfoUnit, dataTobeWrote);
	if (result)
	{
		this->imageInfoUnitQueue.push(currentImageInfoUnit);
	}
	else
	{
		assert(false);
		delete currentImageInfoUnit;
		this->activeInfoUnitCount--;

	}

	//there's a limit on max count of image frames in the buffer
	//so if it is exceeded, then delete the oldest frames
	while (this->imageInfoUnitQueue.size() > this->maxFramesInBuffer)
	{
		elementInBufferInfoUnit * imageToDeleteInfo = this->imageInfoUnitQueue.front();
		this->imageInfoUnitQueue.pop();
		deleteOldestImageInBuffer(imageToDeleteInfo);
		delete imageToDeleteInfo;	//delete the infoUnit
		this->activeInfoUnitCount--;
	}

routineExit:
	ReleaseMutex(hBufferMutex);
	return result;
}

//$$$$$$$$$$$$$$$$$$$$$$$$$$$ private methods $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$

//--------------------------------------------------------
// Write image data to buffer
//--------------------------------------------------------
// Normally, this routine would always return true,
// if a false returned, this means something unknown happened.
//--------------------------------------------------------
BOOL CVideoAudioBuffer::writeImageIntoImgBuffer(elementInBufferInfoUnit *imageToWrite, 
												BYTE* dataTobeWrote)
{

	if (!imageToWrite || !dataTobeWrote)
	{
		cout<< "position 04 false."<<endl;
		return false;
	}

	assert(this->getFreeSpaceSizeInBuffer() >= imageToWrite->data_size);

	//begin copy
	if (!imageToWrite->isBlockCrossBorder)
	{
		assert(imageToWrite->data_size <= this->bufferSize);

		memcpy(	&img263Buffer[imageToWrite->data_head], 
			dataTobeWrote,
			imageToWrite->data_size);

		this->freespace_head += imageToWrite->data_size;

		assert(this->freespace_head <= this->bufferSize);

		if (this->freespace_head == this->bufferSize)
		{
			this->freespace_head = 0;
			this->isBufferBorderCrossed = true;
		}

	}
	else
	{
		memcpy(	&img263Buffer[imageToWrite->data_head], 
			dataTobeWrote,
			this->bufferSize - imageToWrite->data_head);
		memcpy(	&img263Buffer[0], 
			dataTobeWrote + this->bufferSize - imageToWrite->data_head,
			imageToWrite->data_size - (this->bufferSize - imageToWrite->data_head));

		assert(imageToWrite->data_size >= (this->bufferSize - imageToWrite->data_head));

		this->freespace_head = imageToWrite->data_size - (this->bufferSize - imageToWrite->data_head);
		this->isBufferBorderCrossed = true;

	}

	return true;
}

//---------------------------------------------------------
// Read image data from buffer. 
//---------------------------------------------------------
// Return true means succeed, 
// false means buffer is empty or something abnormal happened
//---------------------------------------------------------
BOOL CVideoAudioBuffer::readImageFromImgBuffer(elementInBufferInfoUnit *imageToReadInfo,
											   BYTE* dataRead, 
											   UINT* dataSizeRead
											   )
{
	/*NOTE: ------------------------------------------------------------------
	//			How to read out one image from image263 buffer? 
	// If "isBlockCrossBorder == false", then it's straightforward, just 
	// read from the position of data_head sequentially till data_size achieved.
	// If "isBlockCrossBorder == true", it's a little bit tricky,
	// the process broke into two parts, first, read from the position of data_head
	// to the end of buffer, i.e. till "this->bufferSize - 1".
	// Next, read from the beginning of the buffer to the position of 
	//------------------------------------------------------------------------*/
	if (!imageToReadInfo || !dataRead)
	{
		cout<< "position 05 false."<<endl;
		return false;
	}

	if (!imageToReadInfo->isBlockCrossBorder)
	{

		memcpy(	dataRead, 
			&img263Buffer[this->img263BufferData_head],
			imageToReadInfo->data_size);
		this->img263BufferData_head += imageToReadInfo->data_size;

		assert(this->img263BufferData_head < this->bufferSize);

		//if (this->img263BufferData_head == this->bufferSize)
		//{
		//	this->freespace_head = 0;
		//}
	}
	else
	{
		memcpy(	dataRead, 
			&img263Buffer[this->img263BufferData_head],
			this->bufferSize - this->img263BufferData_head);

		memcpy(	dataRead + (this->bufferSize - this->img263BufferData_head), 
			&img263Buffer[0],
			imageToReadInfo->data_size- (this->bufferSize - this->img263BufferData_head));

		assert(imageToReadInfo->data_size >= (this->bufferSize - this->img263BufferData_head));

		this->img263BufferData_head = imageToReadInfo->data_size 
			- (this->bufferSize - this->img263BufferData_head);

		this->isBufferBorderCrossed = false;
	}
	*dataSizeRead = imageToReadInfo->data_size;

	assert(checkBufferIfNormal());

	return true;

}

//-----------------------------------------------
// Delete oldest image in the buffer
//-----------------------------------------------
// If the buffer is full before writing a new image,
// we should delete the oldest images in the buffer.
// run this routine once, results in the oldest image
// be deleted.
//-----------------------------------------------
void CVideoAudioBuffer::deleteOldestImageInBuffer(elementInBufferInfoUnit * imageToDeleteInfo)
{

	if (!imageToDeleteInfo->isBlockCrossBorder)
	{
		this->img263BufferData_head += imageToDeleteInfo->data_size;

		assert(this->img263BufferData_head < this->bufferSize);

	}
	else
	{
		assert(imageToDeleteInfo->data_size >= (this->bufferSize - this->img263BufferData_head));
		this->img263BufferData_head = imageToDeleteInfo->data_size 
			- (this->bufferSize - this->img263BufferData_head);
		this->isBufferBorderCrossed = false;
	}

}

BOOL CVideoAudioBuffer::checkBufferIfNormal()
{
	/* NOTE : -------------------------------------------------------
	//				How to tell the buffer in a chaos state?
	// Maybe for some unknown reasons, the buffer would be chaotic.
	// So we should check if the buffer in a good condition occasionally.
	// If isBufferBorderCrossed == false, and
	// img263BufferData_head > freespace_head, 
	// or if isBufferBorderCrossed == true, and
	// img263BufferData_head < freespace_head, these two cases mean
	//  something is wrong. In this case, we need to reset the buffer,
	//  i.e. to initialize this class.
	// --------------------------------------------------------------*/

	if (!isBufferBorderCrossed)
	{
		if (this->img263BufferData_head > this->freespace_head)
			return false;
		else
			return true;
	}
	else
	{
		if (this->img263BufferData_head < this->freespace_head)
			return false;
		else
			return true;
	}
}

//-----------------------------------------------
// See if next image to be write into buffer 
// need store it cross the border
//-----------------------------------------------
BOOL CVideoAudioBuffer::doesNeedCrossBorder(UINT dataSizeTobeWrote)
{
	UINT distant01;

	if (!this->isBufferBorderCrossed)
	{
		distant01 = this->bufferSize - this->freespace_head;
		return (dataSizeTobeWrote >= distant01)? true : false;
	}
	else
	{
		return false;
	}

}

//-----------------------------------------------
// 
//-----------------------------------------------
BOOL CVideoAudioBuffer::isImgBufferEmpty()
{
	BOOL result;

	if (this->img263BufferData_head == this->freespace_head)
	{
		if (!this->isBufferBorderCrossed)
			result = true;
		else
			result = false;
	}
	else
		result = false;
	return result;
}

//-----------------------------------------------
// 
//-----------------------------------------------
UINT CVideoAudioBuffer::getFrameCountInBuffer()
{
	queue <int>::size_type sizeInQueue;
	sizeInQueue = this->imageInfoUnitQueue.size();
	return (UINT)sizeInQueue;
}

//-----------------------------------------------
// 
//-----------------------------------------------
INT CVideoAudioBuffer::getActiveInfoUnitCount()
{
	return this->activeInfoUnitCount;
}

//-----------------------------------------------
// 
//-----------------------------------------------
BOOL CVideoAudioBuffer::isImgQueueEmpty()
{
	WaitForSingleObject(hBufferMutex, INFINITE);
	BOOL result = this->imageInfoUnitQueue.empty();
	ReleaseMutex(hBufferMutex);
	return result;

}