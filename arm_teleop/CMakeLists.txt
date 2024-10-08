cmake_minimum_required(VERSION 3.1.3)
project(arm_teleop)

find_package(catkin REQUIRED COMPONENTS
  rospy
  trajectory_msgs
  sensor_msgs
  geometry_msgs
  moveit_commander
  tf
  trac_ik_lib
)

## Declare a catkin package
catkin_package(
  CATKIN_DEPENDS rospy trajectory_msgs sensor_msgs geometry_msgs moveit_commander tf trac_ik_lib
)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Install Python src
catkin_install_python(PROGRAMS
  src/controller.py
  src/joystick_controller.py
  src/goal_based_control.py
  src/controller_based_control.py
  src/teleop_xbox_ee.py
  src/teleop_xbox_joints.py
  src/aa.py
  src/ik_test.py
  src/test_planning_n_execution.py

  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)