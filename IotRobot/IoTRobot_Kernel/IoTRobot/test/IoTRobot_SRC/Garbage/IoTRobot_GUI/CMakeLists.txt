find_package(PCL 1.0 REQUIRED)
#find_package(MFC REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_subdirectory(res)
set(IoT_GUI_Src ChildFrm.cpp
 ChildFrm.h
 CoolTabCtrl.cpp
 CoolTabCtrl.h
 DlgBar.cpp
 DlgBar.h
 DlgOptions.cpp
 DlgOptions.h
 IoTRobot_GUI.cpp
 IoTRobot_GUI.h
 IoTRobot_GUI.aps
 IoTRobot_GUI.rc
 IoTRobot_GUIDoc.cpp
 IoTRobot_GUIDoc.h
 IoTRobot_GUIView.cpp
 IoTRobot_GUIView.h
 MainFrm.cpp
 MainFrm.h
 Resource.h
 scbarg.cpp
 scbarg.h
 SIZECBAR.cpp
 SIZECBAR.h
 stdafx.cpp
 stdafx.h
 targetver.h
 VTKTest.cpp
 VTKTest.h
 IoTRobot_GUI_Interface.cpp
 IoTRobot_GUI_Interface.h
)
#SET(CMAKE_C_FLAGS "/nologo /W3 /GX /Gy /YX")
#SET(CMAKE_C_FLAGS_DEBUG "/MTd /Z7 /Od")
#SET(CMAKE_C_FLAGS_RELEASE "/MT /O2")
#SET(CMAKE_CXX_FLAGS "/nologo /W3 /GX /Gy /YX")
#SET(CMAKE_CXX_FLAGS_DEBUG "/MTd /Z7 /Od")
#SET(CMAKE_CXX_FLAGS_RELEASE "/MT /O2")

#ADD_DEFINITIONS(-D_USRDLL)
SET(CMAKE_MFC_FLAG 2)
SET(LIBRARY_OUTPUT_PATH D:\Work\IoTRobot_Kernel\IoTRobot_RSLT\Lib\IoTRobot_GUI)
add_library (IoTRobot_GUI SHARED ${IoT_GUI_Src} ${Res})
target_link_libraries (IoTRobot_GUI ${PCL_LIBRARIES})

#install(TARGETS IoTRobot_GUI LIBRARY DESTINATION D:\Work\IoTRobot_Kernel\IoTRobot_RSLT\Lib\IoTRobot_GUI)
install(FILES IoTRobot_GUI_Interface.h DESTINATION D:\Work\IoTRobot_Kernel\IoTRobot_RSLT\Include\IoTRobot_GUI)

 

