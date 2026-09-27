cmake_minimum_required(VERSION 3.0.2)
project(sim_ground_truth_viz)

find_package(catkin REQUIRED)

catkin_package()

catkin_install_python(PROGRAMS
  nodes/sim_ground_truth_viz.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES README.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
