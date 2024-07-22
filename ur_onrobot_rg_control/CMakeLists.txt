cmake_minimum_required(VERSION 3.0.2)
project(ur_onrobot_rg_control)
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_srvs
  ur_onrobot_rg_modbus_serial
  message_generation
)

catkin_python_setup()

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  DEPENDS
  CATKIN_DEPENDS
  rospy
  ur_onrobot_rg_modbus_serial
)

catkin_install_python(
  PROGRAMS
  nodes/OnRobotRGSerialNode.py
  nodes/OnRobotRGJointPositionControllerServer.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
