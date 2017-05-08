#pragma once
#define  SLAMMODE_MEMORY_SIZE  1024  //1k
#define  PATROLMODE_MEMORY_SIZE  1024  //1k
#define	 TASKS_MEMORY_SIZE  1024  //1k


#define SENSOREVENT_MEMORY_SIZE 10240 //10k
typedef struct Storage_Settings
{
	char *pcSLAMMode;
	char *pcPatrolMode;
	char *pcTask;
}Storage_Settings;

typedef struct Storage_SensorEvent
{
	char *pcReserve;
}Storage_SensorEvent;




class CStorage
{
public:
	CStorage();
	virtual ~CStorage();

	virtual void GetData(int nPos,char *pcContent)=0;
	virtual void SetData(int nPos,char *pcContent)=0;
	void SaveData();
	void LoadData();
};



class CSettings:public CStorage
{
public:
	CSettings();
	~CSettings();
	Storage_Settings m_stSettingsData;
	virtual void GetData(int nPos,char *pcContent);
	virtual void SetData(int nPos,char *pcContent);

};


class CSensorAndEvent:public CStorage
{
public:
	CSensorAndEvent();
	~CSensorAndEvent();

	Storage_SensorEvent m_stSsrEvtData;
	virtual void GetData(int nPos,char *pcContent);
	virtual void SetData(int nPos,char *pcContent);
};