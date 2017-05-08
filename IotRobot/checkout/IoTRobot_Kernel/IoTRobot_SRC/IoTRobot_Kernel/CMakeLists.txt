find_package(PCL 1.0 REQUIRED)

set(IOT_ROBOT_GUI_LIB IoTRobot_GUI.lib IoTRobot_NetServer.lib IoTRobot_SLAM.lib)
set(IOT_ROBOT_LIBRARY_DIRS D:\Work\IoTRobot_Kernel\IoTRobot_RSLT\Lib)
set(IOT_ROBOT_INCLUDE_DIRS D:\Work\IoTRobot_Kernel\IoTRobot_RSLT\Include)



include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(${IOT_ROBOT_INCLUDE_DIRS})
link_directories(${IOT_ROBOT_LIBRARY_DIRS})


set(IoTRob_Krl_Src CommandAndControl.cpp
 CommandAndControl.h
 ControlDriver.cpp
 ControlDriver.h
 IDrive.cpp
 IDrive.h
 IMU.cpp
 IMU.h
 KernelInterface.cpp
 KernelInterface.h
 MapBuilder.cpp
 MapBuilder.h
 OpenNI.cpp
 OpenNI.h
 Point_Cloud.cpp
 Point_Cloud.h
 SLAM.cpp
 SLAM.h
 Storage.cpp
 Storage.h
 3DMap.cpp
 3DMap.h
)

add_executable (IoTRobot_Kernel  ${IoTRob_Krl_Src})
target_link_libraries (IoTRobot_Kernel ${PCL_LIBRARIES} ${IOT_ROBOT_GUI_LIB})
 

