#pragma once
#define DLL_EXPORT __declspec(dllexport)

class DLL_EXPORT CKernelInterface
{
public:
	CKernelInterface(void);
	~CKernelInterface(void);

	void KernelRun(void *pCmdCtrl);

	void GetRGB24Data(unsigned char *puRGB24);
};
