mobipick_tables_demo
====================

Code for the "tables" demo on 2022-05-05 and 2022-07-12.

Scenario description
--------------------

See this [wiki page](https://git.ni.dfki.de/mobipick/documentation/-/wikis/Mobipick-tables-demo).


Gazebo demo
-----------

```bash
roslaunch mobipick_gazebo tables_demo.launch
rosservice call /gazebo/unpause_physics

roslaunch mir_gazebo fake_localization.launch __ns:="mobipick" odom_frame_id:="mobipick/odom" base_frame_id:="mobipick/base_footprint"
roslaunch mir_navigation start_planner.launch map_file:=$(rospack find pbr_maps)/maps/tables_demo/tables_demo.yaml prefix:="mobipick/"
roslaunch mobipick_moveit_config moveit_planning_execution.launch use_pointcloud:=true simulation:=true
rviz -d $(rospack find mir_navigation)/rviz/navigation.rviz __ns:="mobipick"
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
