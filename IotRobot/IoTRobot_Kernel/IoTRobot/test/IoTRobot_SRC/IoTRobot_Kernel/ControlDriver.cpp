//#include "stdafx.h"
#include "ControlDriver.h"
#include <string>


CControlDriver::CControlDriver()
{
	CreateMsgArray();
//	MsgArrayTest();
}

CControlDriver::~CControlDriver()
{

}

void CControlDriver::CreateMsgArray()
{
	int i;
	memset(&m_stMsgArray,0,sizeof(CtrlDrv_MSGArray));

	for (i=0;i<MSGARRAY_LEN-1;i++)
	{
		m_stMsgArray.stMsgArray[i].pstNext=&m_stMsgArray.stMsgArray[i+1];
	}
	m_stMsgArray.stMsgArray[MSGARRAY_LEN-1].pstNext=&m_stMsgArray.stMsgArray[0];

	m_stMsgArray.pstWritePos=&m_stMsgArray.stMsgArray[0];
	m_stMsgArray.pstReadPos=&m_stMsgArray.stMsgArray[0];
}

int CControlDriver::AddMsg(char *pcContent)
{
	if(m_stMsgArray.pstWritePos->cStateFlag!=1)
	{
		m_stMsgArray.pstWritePos->cContent=(*pcContent);
		m_stMsgArray.pstWritePos->cStateFlag=1;
		m_stMsgArray.pstWritePos=m_stMsgArray.pstWritePos->pstNext;
		return 1;
	}
	else return 0;
}

int CControlDriver::DrawMsg(char *pcContent)
{
	if (m_stMsgArray.pstReadPos->pstNext!=0)
	{
		*pcContent=m_stMsgArray.pstReadPos->cStateFlag;
		m_stMsgArray.pstReadPos->cStateFlag=0;
		m_stMsgArray.pstReadPos=m_stMsgArray.pstReadPos->pstNext;
		return 1;
	}
	else return 0;
}


/*void CControlDriver::MsgArrayTest()
{
	int i;
	char cTMp;
	for (i=0;i<5;i++)
	{
		cTMp=i;
		AddMsg(&cTMp);
	}

	DrawMsg(&cTMp);

	for (i=5;i<12;i++)
	{
		cTMp=i;
		AddMsg(&cTMp);
	}

	for (i=0;i<20;i++)
	{
		DrawMsg(&cTMp);
	}
}*/