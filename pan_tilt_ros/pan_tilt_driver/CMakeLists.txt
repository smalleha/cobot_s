cmake_minimum_required(VERSION 2.8.3)
project(pan_tilt_driver)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  serial
  roscpp
  rospy
  std_msgs
  pan_tilt_msgs
  tf2
  tf2_ros
)

find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pan_tilt_driver
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(PanTiltDriverNode
  src/PanTiltDriverNode.cpp
  src/PanTiltDriver.cpp
  src/QThread.cpp
  src/modbus_rtu_master.cpp
)  
target_link_libraries(PanTiltDriverNode
  ${catkin_LIBRARIES}
)
add_dependencies(PanTiltDriverNode pan_tilt_msgs_generate_messages_cpp)

# add_executable(test2
#   src/test.cpp
#   src/PanTiltDriver.cpp
#   src/QThread.cpp
#   src/QSerialPort.cpp
#   src/QSerialFrame.cpp
# )  
# target_link_libraries(test2
#   ${catkin_LIBRARIES}
# )
