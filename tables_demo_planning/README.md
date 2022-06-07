# Mobipick Tables Demo Planning

Demo of planning and executing a robot's actions using simple Python interfaces.

It uses the [Unified Planning Library](https://github.com/aiplan4eu/unified-planning)
of the AIPlan4EU project for task planning and the [Robot API](https://git.ni.dfki.de/acting/robot_api/)
for plan execution.

## Description

Goal of the robot in this demo is to

- fetch the power drill from the table,
- hand it over to a person,
- return empty-handed to its home position.

`tables_demo_node.py` shows how you can implement this use-case in one Python file. It

- defines environment representation states,
- defines executable actions by using the Robot API,
- defines plannable actions by using the UP library,
- composes the problem description from Gazebo's current world state,
- requests a plan from the planner,
- loops through the list of actions for execution,
- and replans on failure with the failed action removed.

`up_planning.py` is assumed to be provided as a library in the future.

Feel free to abort and restart the Python script at any time to see what parts
of the world state it stores and which it can perceive.

## Execution

After sourcing your ROS environment as usual, the following commands each should go into their own console:

```bash
roslaunch mobipick_gazebo mobipick_moelk.launch
rosservice call /gazebo/unpause_physics   # or click the "start" button in the Gazebo GUI
roslaunch mir_gazebo fake_localization.launch __ns:="mobipick" odom_frame_id:="mobipick/odom" base_frame_id:="mobipick/base_footprint"
roslaunch mir_navigation start_planner.launch map_file:=$(rospack find pbr_maps)/maps/moelk/pbr_robot_lab.yaml prefix:="mobipick/"
roslaunch mobipick_moveit_config moveit_planning_execution.launch use_pointcloud:=true simulation:=true
roslaunch mobipick_task_server mobipick_task_server.launch
roslaunch mobipick_pick_n_place mobipick_pick_n_place.launch simulation:=true
roslaunch robot_api moveit_macros.launch namespace:='mobipick'
rosrun tables_demo_planning tables_demo_node.py
```

## Plan visualization

Install and source this [dot_graph_visualization](https://git.ni.dfki.de/acting/dot_graph_visualization) module, then call it with:

```bash
rqt --standalone dot_graph_visualization
```

## Improvements

This is a basic demo built on existing work. Please add your requests and
suggestions to improve it further as issues into this
[mobipick_tables_demo](https://git.ni.dfki.de/mobipick/mobipick_tables_demo)
repository.
