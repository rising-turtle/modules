#pragma once
class IoTRobot_Encode
{
public:
	IoTRobot_Encode();
	virtual ~IoTRobot_Encode();


	virtual int IoTRobot_EncodeInit()=0;
	virtual int IoTRobot_EncodeRun()=0;
	virtual int IoTRobot_EncodeUnint()=0;

protected:
private:
};