add_subdirectory(res)
set(IoT_GUI_Src
 CoolTabCtrl.cpp
 CoolTabCtrl.h
 DlgOptions.cpp
 DlgOptions.h
 IoTRobot_GUI.aps
 IoTRobot_GUI.cpp
 IoTRobot_GUI.h
 IoTRobot_GUI.rc
 IoTRobot_GUI_Interface.cpp
 IoTRobot_GUI_Interface.h
 IoTRobot_GUIDoc.cpp
 IoTRobot_GUIDoc.h
 IoTRobot_GUIView.cpp
 IoTRobot_GUIView.h
 IoTRobot_Internal_Define.h
 MainFrm.cpp
 MainFrm.h
 OpenGL.cpp
 OpenGL.h
 resource.h
 scbarg.cpp
 scbarg.h
 SIZECBAR.cpp
 SIZECBAR.h
 stdafx.cpp
 stdafx.h
 targetver.h
)

SET(CMAKE_MFC_FLAG 2)
SET(LIBRARY_OUTPUT_PATH D:\Work\IoTRobot_Kernel\IoTRobot_RSLT\Lib\IoTRobot_GUI)
add_library (IoTRobot_GUI SHARED ${IoT_GUI_Src} ${Res})


#target_link_libraries (IoTRobot_GUI ${PCL_LIBRARIES})
#install(TARGETS IoTRobot_GUI LIBRARY DESTINATION D:\Work\IoTRobot_Kernel\IoTRobot_RSLT\Lib\IoTRobot_GUI)
install(FILES IoTRobot_GUI_Interface.h DESTINATION D:\Work\IoTRobot_Kernel\IoTRobot_RSLT\Include\IoTRobot_GUI)

 

