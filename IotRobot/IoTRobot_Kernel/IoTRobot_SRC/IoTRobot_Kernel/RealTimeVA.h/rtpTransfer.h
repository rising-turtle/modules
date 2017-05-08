#ifndef _HEADER_H_RTP
#define _HEADER_H_RTP

#include <stdio.h>
#include <iostream>
using namespace std;

#include "RTP/rtpheader.h"
using namespace jrtplib;

class rtpTransfer
{
public:	//methods
	rtpTransfer();
	~rtpTransfer();

	bool checkerror(int rtperr);

	bool Initialization(unsigned short localPort, 
						unsigned short remotePort, 
						uint8_t remoteIp[4]);



	bool SendPacket(const void *data, size_t len);
	bool rtpTransfer::SendPacket(const void *data, size_t len,
								uint8_t pt, bool mark, uint32_t timestampinc);
	bool Poll();
	
public:	//variables
	RTPSession session;
	size_t MaxPackSize;


private:	//methods
	bool CheckError(int rtperr);

private:	//variables
	WSADATA WSAdata;
	RTPSessionParams sessionparams;
	RTPUDPv4TransmissionParams transparams;
	int status;



};

#endif