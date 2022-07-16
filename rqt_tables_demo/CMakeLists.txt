cmake_minimum_required(VERSION 3.0.2)
project(rqt_tables_demo)

find_package(catkin REQUIRED COMPONENTS
  rqt_gui
  rqt_gui_py
)

catkin_python_setup()

catkin_package()

#############
## Install ##
#############

# Mark executable scripts (Python etc.) for installation
install(PROGRAMS scripts/rqt_tables_demo
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
