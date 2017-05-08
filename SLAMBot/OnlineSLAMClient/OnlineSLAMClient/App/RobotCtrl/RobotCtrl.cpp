#include "RobotCtrl.h"
#include <stdio.h>
#include <string>
int RobotCtrl::m_nCurRunType;
RobotCtrl::RobotCtrl()
{
	m_nCurRunType=0;
}
RobotCtrl::~RobotCtrl()
{

}

int RobotCtrl::CtrlCMD(int nCtrlCMDCode)
{
	printf("nCtrlCMDCode:%d\n",nCtrlCMDCode);
	if (nCtrlCMDCode==1)//IDrive
	{
		m_nCurRunType=1;
		printf("IDrive ~~\n");
		//fist send stop cmd
	}
	else if(nCtrlCMDCode==2)
	{
		m_nCurRunType=2;
		printf("IDrive ~~\n");
		//fist send stop cmd
	}
	else if (nCtrlCMDCode==10)
	{
		if (m_nCurRunType==1)
		{
			printf("left ~~\n");
			//cmd to chassis turn left
		}
	}
	else if (nCtrlCMDCode==11)
	{
		if (m_nCurRunType==1)
		{
			printf("right ~~\n");
			//cmd to chassis turn right
		}
	}
	else if (nCtrlCMDCode==12)
	{
		if (m_nCurRunType==1)
		{
			printf("forward ~~\n");
			//cmd to chassis forward
		}
	}
	else if (nCtrlCMDCode==13)
	{
		if (m_nCurRunType==1)
		{
			printf("backward ~~\n");
			//cmd to chassis backward
		}
	}
	else if (nCtrlCMDCode==14)
	{
		if (m_nCurRunType==1)
		{
			printf("stop ~~\n");
			//cmd to chassis stop
		}
	}
	return 0;
}
int RobotCtrl::GlobalPath(char *pcData)
{
	int nMileStoneNum,i;
	float fMileStone[100];
	char *pcTmp=pcData+4;;
	memcpy(&nMileStoneNum,pcData,4);
	printf("nMileStoneNum:%d  \n",nMileStoneNum);
	for (i=0;i<nMileStoneNum;i++)
	{
		memcpy(&fMileStone[i*2],pcTmp,4);
		printf("mile1:%f  \n",fMileStone[i*2]);
		pcTmp+=4;
		memcpy(&fMileStone[i*2+1],pcTmp,4);
		printf("mile1:%f  \n",fMileStone[i*2+1]);
		pcTmp+=4;
	}

	for (i=0;i<nMileStoneNum;i++)
	{
		printf("MileStoneIdx:%d,  X:%f  ,Y:%f  \n",i,fMileStone[i*2],fMileStone[i*2+1]);
	}
	return 0;
}