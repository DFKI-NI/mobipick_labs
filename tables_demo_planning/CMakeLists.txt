cmake_minimum_required(VERSION 3.0.2)
project(tables_demo_planning)

find_package(catkin REQUIRED message_generation actionlib_msgs)

add_action_files(
  DIRECTORY action
  FILES
  PlanAndExecuteTask.action
)

###################################
## catkin specific configuration ##
###################################
catkin_python_setup()

generate_messages(
  DEPENDENCIES actionlib_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime actionlib_msgs
)

#############
## Install ##
#############

catkin_install_python(PROGRAMS
  nodes/pick_n_place_demo_node.py
  nodes/tables_demo_node.py
  nodes/uplexmo_pick_n_place_demo_node.py
  nodes/uplexmo_tables_demo_node.py
  nodes/hierarchical_demo_node.py
  nodes/task_server_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
