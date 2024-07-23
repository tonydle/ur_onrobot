cmake_minimum_required(VERSION 3.0.2)
project(ur_onrobot_rg_control)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_srvs
  std_msgs
  hardware_interface
  controller_interface
  controller_manager
  joint_limits_interface
  ur_onrobot_rg_modbus_serial
  message_generation
)

add_message_files(
  FILES
  OnRobotRGInputCopy.msg
  OnRobotRGOutputCopy.msg
) 

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    rospy
    roscpp
    std_msgs
    hardware_interface
    controller_interface
    controller_manager
    joint_limits_interface
    ur_onrobot_rg_modbus_serial
    onrobot_rg_control
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node nodes/ur_onrobot_rg_control.cpp src/ur_onrobot_rg_hw_interface.cpp)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)

catkin_install_python(
  PROGRAMS
    nodes/OnRobotRGSerialNode.py
    nodes/MessageBridge.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
