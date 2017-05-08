// IoTRobot_GUIDoc.h : interface of the CIoTRobot_GUIDoc class
//


#pragma once


class CIoTRobot_GUIDoc : public CDocument
{
protected: // create from serialization only
	CIoTRobot_GUIDoc();
	DECLARE_DYNCREATE(CIoTRobot_GUIDoc)

// Attributes
public:

// Operations
public:

// Overrides
public:
	virtual BOOL OnNewDocument();
	virtual void Serialize(CArchive& ar);

// Implementation
public:
	virtual ~CIoTRobot_GUIDoc();
#ifdef _DEBUG
	virtual void AssertValid() const;
	virtual void Dump(CDumpContext& dc) const;
#endif

protected:

// Generated message map functions
protected:
	DECLARE_MESSAGE_MAP()
public:
	virtual void OnCloseDocument();
};


