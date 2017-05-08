// IoTRobot_GUIDoc.cpp : implementation of the CIoTRobot_GUIDoc class
//

#include "stdafx.h"
#include "IoTRobot_GUI.h"

#include "IoTRobot_GUIDoc.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#endif


// CIoTRobot_GUIDoc

IMPLEMENT_DYNCREATE(CIoTRobot_GUIDoc, CDocument)

BEGIN_MESSAGE_MAP(CIoTRobot_GUIDoc, CDocument)
END_MESSAGE_MAP()


// CIoTRobot_GUIDoc construction/destruction

CIoTRobot_GUIDoc::CIoTRobot_GUIDoc()
{
	// TODO: add one-time construction code here

}

CIoTRobot_GUIDoc::~CIoTRobot_GUIDoc()
{
}

BOOL CIoTRobot_GUIDoc::OnNewDocument()
{
	if (!CDocument::OnNewDocument())
		return FALSE;

	// TODO: add reinitialization code here
	// (SDI documents will reuse this document)

	return TRUE;
}




// CIoTRobot_GUIDoc serialization

void CIoTRobot_GUIDoc::Serialize(CArchive& ar)
{
	if (ar.IsStoring())
	{
		// TODO: add storing code here
	}
	else
	{
		// TODO: add loading code here
	}
}


// CIoTRobot_GUIDoc diagnostics

#ifdef _DEBUG
void CIoTRobot_GUIDoc::AssertValid() const
{
	CDocument::AssertValid();
}

void CIoTRobot_GUIDoc::Dump(CDumpContext& dc) const
{
	CDocument::Dump(dc);
}
#endif //_DEBUG


// CIoTRobot_GUIDoc commands

void CIoTRobot_GUIDoc::OnCloseDocument()
{
	// TODO: Add your specialized code here and/or call the base class
	POSITION pos = this->GetFirstViewPosition();
	CDocument::OnCloseDocument();
}
