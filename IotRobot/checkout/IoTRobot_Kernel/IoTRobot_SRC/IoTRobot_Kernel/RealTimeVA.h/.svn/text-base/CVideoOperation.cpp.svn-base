#include "stdafx.h"
#include "CVideoOperation.h"

/*--------------------------------------------------
//
---------------------------------------------------*/
CVideoOperation::CVideoOperation()
{
	this->doUseDefaultCamera		= false;
	this->isVideoRTPThreadCanQuit	= false;
	this->isVideoOperationRunning	= false;
	this->hTrdVideoRTP		= NULL;
	this->pVideoRcvBuffer	= NULL;
	this->pVideoSndBuffer	= NULL;
	this->pCamera			= NULL;
	this->videoTimeStamp	= 0;
	this->isWorkingNormal	= false;
	
	this->Initialization();

}
/*--------------------------------------------------
//
---------------------------------------------------*/
CVideoOperation::~CVideoOperation()
{
	this->Release();
}
/*--------------------------------------------------
//
---------------------------------------------------*/
bool CVideoOperation::Initialization()
{

	this->pVideoSndBuffer	= new CVideoAudioBuffer(/*buffer size = */1024 * 400, 2);
	this->pVideoRcvBuffer	= new CVideoAudioBuffer(/*buffer size = */1024 * 400, 1);
	this->pJpegDealer = new jpegClass();

	return true;
}
/*--------------------------------------------------
//
---------------------------------------------------*/
bool CVideoOperation::Open(bool doesUseCamera,
						   int videoWidth,
						   int videoHeight,
						   BYTE* remoteIpAdd,
						   USHORT localVideoPort,
						   USHORT remoteVideoPort,
						   int jpgQulity,
						   int frameRate)
{
	if (this->isVideoOperationRunning)
	{
		return false;
	}

	if (!remoteIpAdd)
	{
		return false;
	}

	for (int i = 0; i < 4; i++)
	{
		this->remoteIp[i] =	remoteIpAdd[i];
	}

	this->doUseDefaultCamera = doesUseCamera;
	this->width		= videoWidth;
	this->height	= videoHeight;
	
	this->localVideoRtpPort	= localVideoPort;
	this->remoteVideoRtpPort = remoteVideoPort;

	this->jpegQulity	= jpgQulity;
	this->frameRate		= frameRate;
	this->tickInterval	= 1000 / frameRate;

	if (this->doUseDefaultCamera)
	{
		this->pCamera = new CCameraDS();
		int camCount = pCamera->CameraCount();
		if(camCount < 1)
		{
			//indicates that there's no camera connected to local computer
			SAFE_DELETE(this->pCamera);
			goto exitEntry;
		}
		char camName[30];
		pCamera->CameraName(0, camName, 30);			//获取指定摄像头的名字
		bool openResult = pCamera->OpenCamera(0, false, this->width, this->height);	//0 means default video device
		if (!openResult){
			goto exitEntry;
		}
	}
	//_CrtDumpMemoryLeaks();

	this->isWorkingNormal = true;
	this->hTrdVideoRTP = CreateThread(NULL, 0, Thread_RTP_VideoTransfer, (LPVOID)this, 0, NULL);
	this->isVideoOperationRunning = true;



	return true;

exitEntry:
	isWorkingNormal = false;
	errorCode = 2;
	return false;

}
/*--------------------------------------------------
//
---------------------------------------------------*/
bool CVideoOperation::Close()
{
	if (!this->isVideoOperationRunning)
	{
		return false;
	}

	this->isVideoRTPThreadCanQuit = true;
	Sleep(200);

	if (this->doUseDefaultCamera)
	{
		if (pCamera)
		{
			pCamera->CloseCamera();
		}
		
	}

	this->isVideoOperationRunning = false;

	return true;
}
/*--------------------------------------------------
//
---------------------------------------------------*/
bool CVideoOperation::Release()
{
	this->isVideoRTPThreadCanQuit = true;

	Sleep(30);

	SAFE_DELETE(pVideoRcvBuffer);
	SAFE_DELETE(pVideoSndBuffer);
	SAFE_DELETE(pJpegDealer);

	if (this->doUseDefaultCamera)
	{
		if(this->pCamera)
		{
			delete this->pCamera;
			this->pCamera = NULL;
		}
		

	}

	CloseHandle(hTrdVideoRTP);

	return true;
}

/*--------------------------------------------------
//
---------------------------------------------------*/
bool CVideoOperation::GetRemoteVideoFrame(BYTE* pOutVideoBuffer, UINT* sizeOfData)
{
	UINT	dataLen;
	BYTE*	pReadData = new BYTE[100 * 1024];
	UINT32	timeStamp;


	if (this->pVideoRcvBuffer && !this->pVideoRcvBuffer->isImgQueueEmpty())
	{
		this->pVideoRcvBuffer->getNextImageInBuffer(pReadData,
			&dataLen,
			&timeStamp);

		pJpegDealer->jpgToRGB(	pReadData, 
								(unsigned long)dataLen,
								pOutVideoBuffer,
								sizeOfData);

		delete[] pReadData;
		return true;
	}
	else
	{
		delete[] pReadData;
		return false;
	}


}
/*--------------------------------------------------
//
---------------------------------------------------*/
bool CVideoOperation::SendLocalVideoFrame(BYTE* pInVideoBuffer, UINT sizeOfData)
{
	UINT32 timeStamp = 0;
	if (!this->pVideoSndBuffer || this->doUseDefaultCamera)
	{
		return false;
	}

	UINT ByteCount = 0;
	BYTE* pJpegDataOutBuffer = NULL;
	this->pJpegDealer->RGBToJpg(this->width,
		this->height, 
		pInVideoBuffer, 
		&pJpegDataOutBuffer, 
		(unsigned long *)&ByteCount,
		this->jpegQulity);

	bool result = this->pVideoSndBuffer->putOneImageIntoBuffer(pJpegDataOutBuffer, ByteCount, timeStamp);
	delete pJpegDataOutBuffer;
	return result;

}
/*--------------------------------------------
// Video Transfer thread. Encode, send and receive 
// video data via RTP
--------------------------------------------*/
DWORD WINAPI CVideoOperation::Thread_RTP_VideoTransfer(LPVOID pvoid)
{
	CVideoOperation * mainClass = (CVideoOperation *)pvoid;
	//----------------- images processing -------------
	UINT32	timeStamp = 0, lastTimeStamp = 0;
	BYTE*	pReadData;

	//用于存放压缩后的jpeg数据
	unsigned char*	pJpegDataOutBuffer2 = new unsigned char[20 * 1024];
	unsigned char*	pJpegDataOutBuffer = NULL;
	size_t			dataLen;
	unsigned int	ByteCount = 0;
	unsigned char*	pCameraFrame;


	rtpTransfer*	pRTP_VideoTransfer = new rtpTransfer();
	bool result = pRTP_VideoTransfer->Initialization(mainClass->localVideoRtpPort, mainClass->remoteVideoRtpPort, mainClass->remoteIp);
	if (!result)
	{
		mainClass->isWorkingNormal = false;
		mainClass->errorCode = 1;
		goto exitEntry;
	}

	UINT32 lastTickCount = GetTickCount();
	while (!mainClass->isVideoRTPThreadCanQuit && mainClass->isWorkingNormal)
	{
		if ((GetTickCount() - lastTickCount) < mainClass->tickInterval)
		{
			goto receivePart;
		}

		lastTickCount = GetTickCount();

		if (!mainClass->isVideoOperationRunning)
		{
			break;
		}

		if (mainClass->doUseDefaultCamera)
		{
			pCameraFrame =mainClass->pCamera->QueryFrame();

			timeStamp = GetTickCount();

			ByteCount = 0;
			mainClass->pJpegDealer->RGBToJpg(mainClass->width,
											mainClass->height, 
											pCameraFrame, 
											&pJpegDataOutBuffer, 
											(unsigned long *)&ByteCount,
											mainClass->jpegQulity);

			///-------------- send RPT pocket--------------//
			if (ByteCount < pRTP_VideoTransfer->MaxPackSize)
			{
				pRTP_VideoTransfer->SendPacket(pJpegDataOutBuffer, ByteCount, 26, true, timeStamp - lastTimeStamp);
				lastTimeStamp = timeStamp;
			}
			///--------------------------------------------//

			SAFE_DELETE(pJpegDataOutBuffer);
		}
		else
		{
			if (mainClass->pVideoSndBuffer && !mainClass->pVideoSndBuffer->isImgQueueEmpty())
			{
				mainClass->pVideoSndBuffer->getNextImageInBuffer(pJpegDataOutBuffer2, &ByteCount, &timeStamp);
				///-------------- send RPT pocket--------------//
				if (ByteCount < pRTP_VideoTransfer->MaxPackSize)
				{
					pRTP_VideoTransfer->SendPacket(pJpegDataOutBuffer2, ByteCount, 26, true, timeStamp - lastTimeStamp);
					lastTimeStamp = timeStamp;
				}
				///--------------------------------------------//
			}
		}

receivePart:
		///-------------- receive RTP pocket from remote --------------//
		pRTP_VideoTransfer->session.BeginDataAccess();
		if (pRTP_VideoTransfer->session.GotoFirstSourceWithData())
		{
			do 
			{
				RTPPacket *packet;
				while( (packet = pRTP_VideoTransfer->session.GetNextPacket()) != NULL)
				{
					dataLen = packet->GetPayloadLength();
					pReadData = packet->GetPayloadData();
					mainClass->videoTimeStamp = packet->GetTimestamp();
					//cout<<"received dataLen: "<<dataLen<<endl;

					if (mainClass->pVideoRcvBuffer)
					{
						mainClass->pVideoRcvBuffer->putOneImageIntoBuffer(pReadData,
							dataLen,
							mainClass->videoTimeStamp);
					}

					//release buffers and packet
					pRTP_VideoTransfer->session.DeletePacket(packet);

				}//while( (packet = session_rec.GetNextPacket()) != NULL)
			} while (pRTP_VideoTransfer->session.GotoNextSourceWithData());

		}//if (session_rec.GotoFirstSourceWithData())
		pRTP_VideoTransfer->session.EndDataAccess();

#ifndef RTP_SUPPORT_THREAD
		pRTP_VideoTransfer->Poll();
#endif // RTP_SUPPORT_THREAD
		Sleep(10);

	}	//while (waitKey(40) != 27)

	//////////////////////////// ready to exit //////////////////////////////////

exitEntry:

	Sleep(10);
	//release resources
	SAFE_DELETE(pJpegDataOutBuffer);
	SAFE_DELETE(pRTP_VideoTransfer);
	delete[] pJpegDataOutBuffer2;

	return TRUE;
}

