cmake_minimum_required(VERSION 3.0.2)
project(robot_api_up_demo)

find_package(catkin REQUIRED)

catkin_python_setup()

###################################
## catkin specific configuration ##
###################################
catkin_package()

#############
## Install ##
#############

catkin_install_python(PROGRAMS
  nodes/robot_api_up_demo.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
