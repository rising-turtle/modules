SET DISK_SIGN="D:"
SET ZheDaBin="%DISK_SIGN%\client_fusion\bin\ZDbin"
SET DATA_DIR="%DISK_SIGN%\client_fusion\data\%DATE:~0,4%%DATE:~5,2%%DATE:~8,2%-%TIME:~0,2%%TIME:~3,2%%TIME:~6,2%"
MKDIR %DATA_DIR% %DATA_DIR%\rgb %DATA_DIR%\depth %DATA_DIR%\log
START "ROS CORE" "%ZheDaBin%\..\suro_ros_core.exe"
CD /d "%ZheDaBin%"
START  "ODO" "02NR-Mecanum.exe" 
START  "JOYSTICK" "NR-JoyStickCtrl.exe"
START  "LASER" "01NR-SickLaserCapture.exe"
START /D%DATA_DIR%\log "LOGGER" "NR-2dLogger.exe"
START /D%DATA_DIR% "CLIENTS" "%ZheDaBin%\..\Release\clients_fusion.exe"

