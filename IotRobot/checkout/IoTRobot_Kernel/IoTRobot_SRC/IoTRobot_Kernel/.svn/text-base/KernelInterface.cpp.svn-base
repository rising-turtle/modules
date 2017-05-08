#include "KernelInterface.h"
#include "CommandAndControl.h"

CCommandAndControl CCmdCtrl;
CKernelInterface::CKernelInterface(void)
{
}

CKernelInterface::~CKernelInterface(void)
{
}

void CKernelInterface::KernelRun(void *pCmdCtrl)
{
		pCmdCtrl=&CCmdCtrl;
		CCmdCtrl.CmdCtrlRun();
}


void CKernelInterface::GetRGB24Data(unsigned char *puRGB24)
{
	CCmdCtrl.m_CMapBuilder.GetRGB24Data(puRGB24);
}