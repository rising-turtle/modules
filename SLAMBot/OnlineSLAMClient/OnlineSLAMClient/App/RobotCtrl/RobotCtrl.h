#pragma once
#include "../App.h"
class RobotCtrl:public App
{

public:
	RobotCtrl();
	~RobotCtrl();

	static int CtrlCMD(int nCtrlCMDCode);
	static int GlobalPath(char *pcData);

private:
	static int m_nCurRunType;
};