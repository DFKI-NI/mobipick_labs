# Robot API + UP Demo

Demo of planning and executing a robot's actions using simple Python interfaces.

It uses the [Unified Planning Library](https://github.com/aiplan4eu/unified-planning)
of the AIPlan4EU project for task planning and the [Robot API](https://git.ni.dfki.de/asung/robot_api/)
for plan execution.

## Description

Goal of the robot in this demo is to

- fetch the power drill from the table,
- hand it over to a person,
- return empty-handed to its home position.

`robot_api_up_demo.py` shows how you can implement this use-case in one Python file. It

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

## Installation

This demo might install a lot, depending on what you already have on your
system. Your catkin workspace for mobipick, if you don't mind adding further
repositories, or a new catkin workspace with only this repository is
recommended to start with. [vcs](https://pypi.org/project/vcstool/),
[wstool](http://wiki.ros.org/wstool), and Python 3 are assumed to be available.

```bash
./install-deps.sh
./build.sh
```

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
rosrun robot_api_up_demo robot_api_up_demo.py
```

## Improvements

This is a basic demo built on existing work. Please add your requests and
suggestions to improve it further as issues into this
[mobipick_tables_demo](https://git.ni.dfki.de/mobipick/mobipick_tables_demo)
repository.
