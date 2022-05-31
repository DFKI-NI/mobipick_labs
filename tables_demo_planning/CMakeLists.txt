cmake_minimum_required(VERSION 3.0.2)
project(tables_demo_planning)

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
  nodes/tables_demo_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
