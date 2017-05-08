#include "stdafx.h"
#include "rtpTransfer.h"
#include "Global.h"

/*--------------------------------------------------
//	RTP error check
---------------------------------------------------*/
bool rtpTransfer::checkerror(int rtperr)
{
	if (rtperr < 0)
	{
		std::cout << "ERROR: " << RTPGetErrorString(rtperr) << std::endl;
		return false;
	}
	return true;
}
/*--------------------------------------------------
//
---------------------------------------------------*/
rtpTransfer::rtpTransfer()
{
	this->MaxPackSize = MAX_RTP_POCKET_SIZE;
}
/*--------------------------------------------------
//
---------------------------------------------------*/
rtpTransfer::~rtpTransfer()
{
	RTPTime delay(5.0);
	session.BYEDestroy(delay, "trans over", 10);
	WSACleanup();
}

/*--------------------------------------------------
//
---------------------------------------------------*/
bool rtpTransfer::Initialization(unsigned short localPort, unsigned short remotePort, uint8_t remoteIp[4])
{
	bool result = true;
	RTPIPv4Address addr(remoteIp, remotePort);

	WSAStartup(MAKEWORD(2,2), &WSAdata);
	sessionparams.SetOwnTimestampUnit(1.0/90000.0);	//for video ---see RFC2190
	transparams.SetPortbase(localPort);
	status = session.Create(sessionparams,&transparams);
	result = checkerror(status);
	if (!result)
	{
		goto exitEntry;
	}
	status = session.AddDestination(addr);
	result = checkerror(status);
	if (!result)
	{
		goto exitEntry;
	}
	session.SetDefaultPayloadType(34);	//PT for H.263----see RFC2190
	session.SetDefaultMark(true);		//true or false ?? both ok for my test	
	session.SetDefaultTimestampIncrement(3600);	// =90000/25
	session.SetMaximumPacketSize(this->MaxPackSize);

exitEntry:
	return result;
}

/*--------------------------------------------------
//
---------------------------------------------------*/
bool rtpTransfer::SendPacket(const void *data,size_t len)
{
	status = session.SendPacket(data, len);
	return checkerror(status);
}

bool rtpTransfer::SendPacket(const void *data,size_t len,
			   uint8_t pt,bool mark,uint32_t timestampinc)
{
	status = session.SendPacket(data, len, pt, mark, timestampinc);
	return checkerror(status);
}


/*--------------------------------------------------
//
---------------------------------------------------*/
bool rtpTransfer::Poll()
{
	session.Poll();
	return checkerror(status);
}