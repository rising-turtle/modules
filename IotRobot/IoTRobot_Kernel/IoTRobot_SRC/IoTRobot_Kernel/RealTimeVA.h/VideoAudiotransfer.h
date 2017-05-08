// VideoAudiotransfer.h : main header file for the PROJECT_NAME application
//

#pragma once
#include "Global.h"

#ifndef __AFXWIN_H__
	#error "include 'stdafx.h' before including this file for PCH"
#endif

#include "resource.h"		// main symbols


// CVideoAudiotransferApp:
// See VideoAudiotransfer.cpp for the implementation of this class
//

class CVideoAudiotransferApp : public CWinApp
{
public:
	CVideoAudiotransferApp();

// Overrides
	public:
	virtual BOOL InitInstance();

// Implementation

	DECLARE_MESSAGE_MAP()
};

extern CVideoAudiotransferApp theApp;