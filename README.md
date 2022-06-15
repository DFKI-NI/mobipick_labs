mobipick_tables_demo
====================

Code for the "tables" demo on 2022-07-12.

Scenario description
--------------------

See this [wiki page](https://git.ni.dfki.de/mobipick/documentation/-/wikis/Mobipick-tables-demo).


Installation
------------

This demo might install a lot, depending on what you already have on your
system. Your catkin workspace for mobipick, if you don't mind adding further
repositories, or a new catkin workspace with only this repository is
recommended to start with. [vcs](https://pypi.org/project/vcstool/) and
[wstool](http://wiki.ros.org/wstool) will be installed when needed,
ROS and Python 3 are assumed to be available on your system.

```bash
./install-deps.sh
./build.sh
```

Gazebo demo
-----------

```bash
roslaunch mobipick_gazebo tables_demo.launch
rosservice call /gazebo/unpause_physics

roslaunch mir_gazebo fake_localization.launch __ns:="mobipick" odom_frame_id:="mobipick/odom" base_frame_id:="mobipick/base_footprint"
roslaunch mir_navigation start_planner.launch map_file:=$(rospack find pbr_maps)/maps/tables_demo/tables_demo.yaml \
    virtual_walls_map_file:=$(rospack find pbr_maps)/maps/tables_demo/tables_demo_virtual_walls.yaml \
    prefix:="mobipick/"
roslaunch mobipick_moveit_config moveit_planning_execution.launch use_pointcloud:=true simulation:=true
rviz -d $(rospack find mir_navigation)/rviz/navigation.rviz __ns:="mobipick"
roslaunch mobipick_task_server mobipick_task_server.launch
roslaunch robot_api moveit_macros.launch namespace:='mobipick'
roslaunch mobipick_pick_n_place mobipick_pick_n_place.launch world:=moelk_tables_demo simulation:=true
rosrun tables_demo_planning tables_demo_node.py
```

Pick-and-place demo on the real robot
-------------------------------------

Goal of the robot in this demo is to

- fetch the power drill from the table,
- hand it over to a person,
- return empty-handed to its home position.

```bash
roslaunch mobipick_bringup mobipick_bringup_both.launch
roslaunch pbr_dope dope.launch
roslaunch mobipick_pick_n_place mobipick_pick_n_place.launch world:=moelk_tables_demo
rosservice call /mobipick/continue_statemachine
```

Plan visualization
------------------

Install and source this
[dot_graph_visualization](https://git.ni.dfki.de/acting/dot_graph_visualization)
module, then call it with:

```bash
rqt --standalone dot_graph_visualization
```

pre-commit Formatting Checks
----------------------------

This repo has a [pre-commit](https://pre-commit.com/) check that runs in CI.
You can use this locally and set it up to run automatically before you commit
something. To install, use pip:

```bash
pip3 install --user pre-commit
```

To run over all the files in the repo manually:

```bash
pre-commit run -a
```

To run pre-commit automatically before committing in the local repo, install the git hooks:

```bash
pre-commit install
