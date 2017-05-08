cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(RGBDSLAM_Clinet)
find_package(PCL 1.0 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(IOT_ROBOT_LIB IoTRobot_SLAM.lib)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


set(RGBDSLAM_ClinetSrc IoTRobot_RGBDSLAM_Client_Interface.cpp
 IoTRobot_RGBDSLAM_Client_Interface.h
 IoTRobot_RGBDSLAM_Interface.h
 IoTRobot_RGBDSLAM_Client.cpp
 IoTRobot_RGBDSLAM_Client.h
 PCL_Openni.cpp
 PCL_Openni.h)


add_library (RGBDSLAM_Clinet SHARED ${RGBDSLAM_ClinetSrc})
target_link_libraries (RGBDSLAM_Clinet ${PCL_LIBRARIES} ${IOT_ROBOT_LIB})
 

