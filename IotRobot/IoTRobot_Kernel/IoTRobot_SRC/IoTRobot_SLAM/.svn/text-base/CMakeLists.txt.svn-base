find_package(PCL 1.0 REQUIRED)
find_package(OPENCV)
find_package(OPENGL)


include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(${OPENCV_INCLUDE_DIRS})
link_directories(${OPENCV_LIBRARY_DIRS})
add_definitions(${OPENCV_DEFINITIONS})




set(CS_LIB CS_Lib.lib)
set(IOT_ROBOT_LIBRARY_DIRS D:\Work\IoTRobot_Kernel\IoTRobot_RSLT\Lib)
set(IOT_ROBOT_INCLUDE_DIRS D:\Work\IoTRobot_Kernel\IoTRobot_RSLT\Include)

include_directories(${IOT_ROBOT_INCLUDE_DIRS})
link_directories(${IOT_ROBOT_LIBRARY_DIRS})

add_subdirectory(AIS_Lib)

set(OPENCV_LIBRARIES 
 opencv_calib3d230d.lib
 opencv_contrib230d.lib
 opencv_core230d.lib
 opencv_features2d230d.lib
 opencv_flann230d.lib
 opencv_gpu230d.lib
 opencv_haartraining_engined.lib
 opencv_highgui230d.lib
 opencv_imgproc230d.lib
 opencv_legacy230d.lib
 opencv_ml230d.lib
 opencv_objdetect230d.lib
 opencv_ts230d.lib
 opencv_video230d.lib
)
set(RGBDSLAMSrc CLogfile.cpp
 CLogfile.h
 CPose3D.cpp
 CPose3D.h
 globaldefinitions.cpp
 globaldefinitions.h
 #glviewer.cpp
 #glviewer.h
 graph_manager.cpp
 graph_manager.h
 IoTRobot_RGBDSLAM_Interface.cpp
 IoTRobot_RGBDSLAM_Interface.h
 matching_result.h
 MyICP.cpp
 MyICP.h
 node.cpp
 node.h
 Openni.cpp
 Openni.h
 test_pcl.cpp
 test_pcl.h
 test_surf.cpp
 test_surf.h
 test_vslam.cpp
 test_vslam.h
 Openni.cpp
 Openni.h
 Viewer.cpp
 Viewer.h
 IoTRobot_SLAM.h
 IoTRobot_SLAM.cpp
 IoTRobot_RGBDSLAM.cpp
 IoTRobot_RGBDSLAM.h
)


add_library (IoTRobot_SLAM SHARED ${RGBDSLAMSrc} ${AIS_Src})

#add_executable (IoTRobot_SLAM  ${RGBDSLAMSrc})
target_link_libraries (IoTRobot_SLAM ${PCL_LIBRARIES} ${OPENCV_LIBRARIES} ${CS_LIB})
 

