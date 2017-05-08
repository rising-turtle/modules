set(WinSockLib ws2_32.lib
 wsock32.lib
 odbc32.lib
 odbccp32.lib
)

set(IoT_Net_Src IoTRobot_NetServer.cpp
 IoTRobot_NetServer.h
 IoTRobot_NetServer_Define.h
)
add_library (IoTRobot_NetServer SHARED ${IoT_Net_Src})
target_link_libraries (IoTRobot_NetServer ${WinSockLib})










 

