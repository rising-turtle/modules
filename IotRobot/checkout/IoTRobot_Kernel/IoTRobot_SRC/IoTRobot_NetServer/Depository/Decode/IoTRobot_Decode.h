#pragma once
class IoTRobot_Decode
{
public:
	IoTRobot_Decode();
	virtual ~IoTRobot_Decode();


	virtual int IoTRobot_DecodeInit()=0;
	virtual int IoTRobot_DecodeRun()=0;
	virtual int IoTRobot_DecodeUnint()=0;

protected:
private:
};