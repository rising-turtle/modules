#include "stdafx.h"
#include "CAudioOperation.h"

/*--------------------------------------------------
//
---------------------------------------------------*/
CAudioOperation::CAudioOperation()
{
	isAudioCaptureRunning	= false;
	isAudioPlayerRunning	= false;
	isWorkingNormal		= false;
	errorCode			= 0;
	isAudioRTPThreadCanQuit	= false;

	encoderState	= NULL;
	decoderState	= NULL;
	AMR_mode		= MR122;
	dtx				= 0;
	
	pAudioCapture	= NULL;
	pAudioPlayer	= NULL;
	pAudioRcvBuffer	= NULL;	//audio buffer for receiving
	pAudioSendBuffer= NULL;	//audio buffer for sending
	hTrdAudioRTP	= NULL;

	this->Initialization();

}
/*--------------------------------------------------
//
---------------------------------------------------*/
CAudioOperation::~CAudioOperation()
{
	this->Release();
}
/*--------------------------------------------------
//
---------------------------------------------------*/
bool CAudioOperation::Initialization()
{
	//make instances of every module
	this->pAudioCapture	= new CDirectSoundRecorder();
	this->pAudioPlayer	= new CDirectSoundPlayer();
	this->pAudioRcvBuffer	= new CVideoAudioBuffer(/*buffer size = */AMR_BLOCK_SIZE * 6, /*max frame count = */150);
	this->pAudioSendBuffer	= new CVideoAudioBuffer(AMR_BLOCK_SIZE * 150, 150);

	this->encoderState = Encoder_Interface_init(dtx);
	this->decoderState = Decoder_Interface_init();

	return true;
}
/*--------------------------------------------------
//
---------------------------------------------------*/
bool CAudioOperation::Open(HWND hWnd, BYTE* remoteIpArr, USHORT localAudioRtpPort, USHORT remoteAudioRtpPort)//AudioPlayer need a window handle
{

	if (this->isAudioPlayerRunning && this->isAudioCaptureRunning)
	{
		return false;
	}

	this->localAudioRtpPort		= localAudioRtpPort;
	this->remoteAudioRtpPort	= remoteAudioRtpPort;


	if (remoteIpArr == NULL)
	{
		return false;
	}
	else
	{
		for (int i = 0;i < 4; i++)
		{
			this->remoteIp[i] = remoteIpArr[i];
		}
	}

	bool result;
	result = this->pAudioCapture->Open();
	if (!result)
	{
		goto exitEntry;
	}
	result = this->pAudioPlayer->Open(hWnd, this);
	if (!result)
	{
		goto exitEntry;
	}

	result = this->pAudioCapture->BeginCapture(TRUE, this);
	if (!result)
	{
		goto exitEntry;
	}
	this->isAudioCaptureRunning = TRUE;

	result = this->pAudioPlayer->BeginPlay(TRUE);
	if (!result)
	{
		goto exitEntry;
	}
	this->isAudioPlayerRunning = TRUE;

	this->isAudioRTPThreadCanQuit = false;
	this->hTrdAudioRTP = CreateThread(NULL, 0, Thread_RTP_AudioTransfer, (LPVOID)this, 0, NULL);
	if (!hTrdAudioRTP)
	{
		goto exitEntry;
	}
	isWorkingNormal = true;
	errorCode = 0;

	return true;

exitEntry:
	isWorkingNormal = false;
	errorCode = 2;


}
/*--------------------------------------------------
//
---------------------------------------------------*/
bool CAudioOperation::Close()
{

	if (!this->isAudioPlayerRunning && !this->isAudioCaptureRunning)
	{
		return true;
	}

	this->isAudioRTPThreadCanQuit = true;
	Sleep(50);

	bool result;
	result = this->pAudioPlayer->BeginPlay(FALSE);
	this->isAudioPlayerRunning = FALSE;

	result = this->pAudioCapture->BeginCapture(FALSE, NULL);
	this->isAudioCaptureRunning = FALSE;

	result = this->pAudioCapture->Close();
	result = this->pAudioPlayer->Close();

	if (!result)
	{
		isWorkingNormal = false;
		errorCode = 2;
	}
	return result;
}
/*--------------------------------------------------
//
---------------------------------------------------*/
void CAudioOperation::AudioCaptureCallback(unsigned char* pBuffer, long lBufferSize)
{
	int		byteSize;
	BYTE	amrEncoded[AMR_BLOCK_SIZE];
	UINT32	timestamp = GetTickCount();

	int dataBlockCount = lBufferSize / PCM_BLOCK_SIZE;//calculate how many blocks in there

	for(int blockIndex = 0; blockIndex < dataBlockCount; blockIndex++)
	{
		//encode every PCM block
		byteSize = Encoder_Interface_Encode(encoderState, 
			AMR_mode, 
			(short*)(pBuffer + PCM_BLOCK_SIZE * blockIndex),
			amrEncoded,
			0);

		pAudioSendBuffer->putOneImageIntoBuffer(amrEncoded, AMR_BLOCK_SIZE, timestamp);
	}

}
/*--------------------------------------------------
//
---------------------------------------------------*/
void CAudioOperation::AudioPlayerCallback(unsigned char* pBuffer, long lBufferSize)
{
	int blockIndex, i;
	UINT32 timestamp;
	UINT dataSizeRead;
	BYTE amrDataBuffer[AMR_BLOCK_SIZE];

	int dataBlockCount = lBufferSize / PCM_BLOCK_SIZE;

	for(blockIndex = 0; blockIndex < dataBlockCount; blockIndex++)
	{
		//get data from rcv buffer
		if(pAudioRcvBuffer->isImgQueueEmpty())
		{
			for(i = 0; i < PCM_BLOCK_SIZE; i++)
			{
				pBuffer[PCM_BLOCK_SIZE * blockIndex + i] = 0x0;
			}
		}
		else
		{
			pAudioRcvBuffer->getNextImageInBuffer(amrDataBuffer,
				&dataSizeRead,
				&timestamp);

			Decoder_Interface_Decode(this->decoderState, 
				amrDataBuffer,
				(short*)&pBuffer[PCM_BLOCK_SIZE * blockIndex],
				0);

		}
	}

}
/*--------------------------------------------------
//
---------------------------------------------------*/
bool CAudioOperation::Release()
{
	Decoder_Interface_exit(decoderState);
	Encoder_Interface_exit(encoderState);

	//stop capture
	if (this->isAudioCaptureRunning)
	{
		this->pAudioCapture->BeginCapture(FALSE, NULL);
	}

	//stop play
	if (this->isAudioPlayerRunning)
	{
		this->pAudioPlayer->BeginPlay(FALSE);
	}

	isAudioCaptureRunning	= false;
	isAudioPlayerRunning	= false;

	//ask rtp thread to quit
	this->isAudioRTPThreadCanQuit = TRUE;

	Sleep(20);

	//delete all substances
	SAFE_DELETE(pAudioCapture);
	SAFE_DELETE(pAudioPlayer);
	SAFE_DELETE(pAudioRcvBuffer);
	SAFE_DELETE(pAudioSendBuffer);

	CloseHandle(hTrdAudioRTP);

	return true;
}
/*--------------------------------------------
// AudioTransfer thread
--------------------------------------------*/
DWORD WINAPI CAudioOperation::Thread_RTP_AudioTransfer(LPVOID pvoid)
{
	CAudioOperation * mainClass = (CAudioOperation *)pvoid;
	size_t			dataLen;
	uint8_t*		readData;
	unsigned int	byteCount;
	int				dataBlockCount;
	UINT32			timeStamp = 0, lastTimeStamp = 0;
	BYTE amrDataBuffer[AMR_BLOCK_SIZE];
	RTPPacket*		packet;


	rtpTransfer*	pRTP_AudioTransfer = new rtpTransfer();
	bool result;
	result = pRTP_AudioTransfer->Initialization(mainClass->localAudioRtpPort, mainClass->remoteAudioRtpPort, mainClass->remoteIp);
	if (!result)
	{
		mainClass->isWorkingNormal = false;
		mainClass->errorCode = 1;
		goto exitEntry;
	}

	while (!mainClass->isAudioRTPThreadCanQuit && mainClass->isWorkingNormal)
	{
		//·¢ËÍaudioÊý¾Ý
		if (!mainClass->pAudioSendBuffer->isImgQueueEmpty())
		{
			mainClass->pAudioSendBuffer->getNextImageInBuffer(amrDataBuffer,
				&byteCount,
				&timeStamp);

			pRTP_AudioTransfer->SendPacket(amrDataBuffer, byteCount, 100, true, timeStamp - lastTimeStamp);
			lastTimeStamp = timeStamp;
		}


		///-------------- receive RTP pocket from remote --------------//
		pRTP_AudioTransfer->session.BeginDataAccess();
		if (pRTP_AudioTransfer->session.GotoFirstSourceWithData())
		{
			do 
			{
				while( (packet = pRTP_AudioTransfer->session.GetNextPacket()) != NULL)
				{
					dataLen = packet->GetPayloadLength();
					readData = packet->GetPayloadData();
					dataBlockCount = dataLen / AMR_BLOCK_SIZE;

					for(int blockIndex = 0; blockIndex < dataBlockCount; blockIndex++)
					{
						mainClass->pAudioRcvBuffer->putOneImageIntoBuffer((BYTE *)&readData[blockIndex * AMR_BLOCK_SIZE],
							AMR_BLOCK_SIZE,
							packet->GetTimestamp());
					}

					//release and packet
					pRTP_AudioTransfer->session.DeletePacket(packet);

				}//while( (packet = session_rec.GetNextPacket()) != NULL)
			} while (pRTP_AudioTransfer->session.GotoNextSourceWithData());

		}//if (session_rec.GotoFirstSourceWithData())
		pRTP_AudioTransfer->session.EndDataAccess();

#ifndef RTP_SUPPORT_THREAD
		pRTP_AudioTransfer->Poll();
#endif // RTP_SUPPORT_THREAD

	}	//while
exitEntry:
	SAFE_DELETE(pRTP_AudioTransfer);

	return TRUE;
}
/*--------------------------------------------
// 
--------------------------------------------*/
void CAudioOperation::SetPlayLocalAudio(bool status)
{
	if (status == this->isAudioPlayerRunning)
	{
		return;
	}

	bool result;

	result = this->pAudioPlayer->BeginPlay(status);
	this->isAudioPlayerRunning = status;

	if (!result)
	{
		isWorkingNormal = false;
		errorCode = 2;
	}

}
/*--------------------------------------------
// 
--------------------------------------------*/
bool CAudioOperation::GetPlayLocalAudioStatus()
{
	return this->isAudioPlayerRunning;
}