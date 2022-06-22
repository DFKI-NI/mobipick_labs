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
roslaunch tables_demo_bringup demo_sim.launch
rosrun tables_demo_planning tables_demo_node.py
```

Optionally for visualization:

```bash
rviz -d $(rospack find mir_navigation)/rviz/navigation.rviz __ns:="mobipick"
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
