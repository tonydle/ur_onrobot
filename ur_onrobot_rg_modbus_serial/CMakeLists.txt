cmake_minimum_required(VERSION 3.0.2)
project(ur_onrobot_rg_modbus_serial)
find_package(catkin REQUIRED COMPONENTS rospy)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

catkin_python_setup()
catkin_package(
    CATKIN_DEPENDS rospy 
)
