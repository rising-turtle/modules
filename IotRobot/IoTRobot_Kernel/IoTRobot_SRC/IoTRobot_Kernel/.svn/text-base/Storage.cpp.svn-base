//#include "stdafx.h"
#include "Storage.h"
#include <string>

CStorage::CStorage()
{

}

CStorage::~CStorage()
{

}

CSettings::CSettings()
{
	m_stSettingsData.pcSLAMMode=new char[SLAMMODE_MEMORY_SIZE];
	m_stSettingsData.pcPatrolMode=new char [PATROLMODE_MEMORY_SIZE];
	m_stSettingsData.pcTask=new char [TASKS_MEMORY_SIZE];
}

CSettings::~CSettings()
{
	delete [] m_stSettingsData.pcSLAMMode;
	delete [] m_stSettingsData.pcPatrolMode;
	delete [] m_stSettingsData.pcTask;
}


void CSettings::GetData(int pos,char *pcContent)
{
	switch(pos)
	{
	case 0:
			memcpy(pcContent,m_stSettingsData.pcSLAMMode,SLAMMODE_MEMORY_SIZE);
			break;
	case 1:
			memcpy(pcContent,m_stSettingsData.pcPatrolMode,PATROLMODE_MEMORY_SIZE);
			break;
	case 2:
			memcpy(pcContent,m_stSettingsData.pcTask,TASKS_MEMORY_SIZE);
			break;
	default:
		break;

	}
}

void CSettings::SetData(int nPos,char *pcContent)
{
	switch(nPos)
	{
	case 0:
		memcpy(m_stSettingsData.pcSLAMMode,pcContent,SLAMMODE_MEMORY_SIZE);
		break;
	case 1:
		memcpy(m_stSettingsData.pcPatrolMode,pcContent,PATROLMODE_MEMORY_SIZE);
		break;
	case 2:
		memcpy(m_stSettingsData.pcTask,pcContent,TASKS_MEMORY_SIZE);
		break;
	default:
		break;

	}
}


CSensorAndEvent::CSensorAndEvent()
{
	m_stSsrEvtData.pcReserve=new char [SENSOREVENT_MEMORY_SIZE];
}

CSensorAndEvent::~CSensorAndEvent()
{
	delete [] m_stSsrEvtData.pcReserve;
}

void CSensorAndEvent::SetData(int nPos, char *pcContent)
{
	switch(nPos)
	{
	case 0:
		memcpy(m_stSsrEvtData.pcReserve,pcContent,SENSOREVENT_MEMORY_SIZE);
		break;
	default:
		break;
	}
}

void CSensorAndEvent::GetData(int nPos, char *pcContent)
{
	switch(nPos)
	{
	case 0:
			memcpy(pcContent,m_stSsrEvtData.pcReserve,SENSOREVENT_MEMORY_SIZE);
			break;
	default:
		break;
	}
}