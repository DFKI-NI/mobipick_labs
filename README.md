mobipick_labs
=============

Scenario description
--------------------

![mobipick_tables_sim_and_real.png](images/mobipick_tables_sim_and_real.png)

The tables demo consists of an environment with items on multiple tables, in which the robot shall

- find the box and place it onto the target table,
- find the multimeter and place it into the box,
- in any order,
- and react to changes to the environment, e.g. items being moved around during the demo.

For details about the event at which this demo was presented, see its (DFKI internal)
[Mobipick tables demo wiki page](https://git.ni.dfki.de/mobipick/documentation/-/wikis/Mobipick-tables-demo).

YouTube overview
----------------

[![Mobipick Labs on YouTube](http://img.youtube.com/vi/4-GgOg2nuGE/0.jpg)](http://www.youtube.com/watch?v=4-GgOg2nuGE
"Mobipick Labs System Demonstration")

Installation
------------

This demo might install a lot, depending on what you already have on your system.
Your catkin workspace for [mobipick](https://github.com/DFKI-NI/mobipick),
if you don't mind adding further repositories, or a new catkin workspace with only this
repository is recommended to start with. [vcs](https://pypi.org/project/vcstool/) and
[wstool](http://wiki.ros.org/wstool) will be installed when needed,
ROS and Python 3 are assumed to be available on your system.

```bash
./install-deps.sh
./build.sh
```

Alternatively, you can use the [Docker environment](https://github.com/brean/mobipick_labs_docker)
created by our colleague Andreas Bresser for a simple all-in-one quick start.

Real robot demo
---------------

Start up the robot according to the (DFKI internal)
[instructions on the wiki](https://git.ni.dfki.de/mobipick/documentation/-/wikis/starting-up-the-robot),
then:


```bash
roslaunch mobipick_bringup mobipick_bringup_both.launch  # already part of the startup instructions
roslaunch pbr_dope dope.launch
roslaunch tables_demo_bringup bringup.launch   # optional: world_config:=cic_tables (default: moelk_tables)
```

To start the full tables demo, then run:

```bash
rosrun tables_demo_planning tables_demo_node.py  # optional: <number of target table>, e.g., "4"; default: 2
```

To start the power drill pick&place demo, run:

```bash
rosrun tables_demo_planning pick_n_place_demo_node.py
```

Optionally, for making the robot speak, run this command on a PC with a speaker (e.g., your laptop):

```bash
rosrun espeak_ros espeak_node
```

Optionally for visualization:

```bash
rosrun rviz rviz -d `rospack find tables_demo_bringup`/config/pick_n_place.rviz __ns:=mobipick
```

Gazebo demo
-----------

```bash
roslaunch tables_demo_bringup demo_sim.launch
rosrun tables_demo_planning tables_demo_node.py
```

The optional components above for speaker and visualization work in simulation as well.

Grasping/Placing/Inserting objects demo (using grasplan)
--------------------------------------

Goal of the robot in this demo is to

- test grasplan
- grasp multiple objects in simulation
- useful for debugging

```bash
roscore
roslaunch tables_demo_bringup demo_sim.launch robot_x:=12.43 robot_y:=2.21 robot_yaw:=1.5708
rosrun rviz rviz -d `rospack find tables_demo_bringup`/config/pick_n_place.rviz __ns:=mobipick
```

Pick

```bash
rosrun grasplan pick_obj_test_action_client __ns:=mobipick power_drill_with_grip table_1
```

Optionally you can specify a list of objects to ignore/delete from planning scene. This will allow e.g. to pick
a box with objects inside it. To run do:

```bash
rosrun grasplan pick_obj_test_action_client __ns:=mobipick power_drill_with_grip table_1 object_to_ignore_1 object_to_ignore_2 ...
rosrun grasplan pick_obj_test_action_client __ns:=mobipick klt table_1 multimeter_1
```

Place

```bash
rosrun grasplan place_obj_test_action_client __ns:=mobipick table_3 true
```

Insert

```bash
rosrun grasplan insert_obj_test_action_client __ns:=mobipick klt_3 true
```

If you want to grasp the other objects you can use the following robot pose.

```bash
roslaunch tables_demo_bringup demo_sim.launch robot_x:=10.46 robot_y:=2.47 robot_yaw:=3.1415
```

If you want to grasp another object after picking, please place the object first.


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

Hierarchical planning using ROS task server
------------------

Used to send hierarchical tasks directly to the planner and executor.

On the real robot:

```bash
roslaunch mobipick_bringup mobipick_bringup_both.launch
roslaunch pbr_dope dope.launch
rosrun tables_demo_planning task_server_node.py
```

For Gazebo:

```bash
roslaunch tables_demo_bringup demo_sim.launch
rosrun tables_demo_planning task_server_node.py
```

Now tasks can be sent to the robot using the ROS client provided:

```bash
rosrun tables_demo_planning task_server_client.py task_name parameter_1 ... parameter_n
```

Depending on the chosen task, different parameters have to be sent to the server.
Available tasks and their parameters can be seen
[here](https://github.com/DFKI-NI/mobipick_labs/blob/4d8e48adc9d2b78682e91e4ad9c091baa03fc4ec/tables_demo_planning/src/tables_demo_planning/hierarchical_domain.py#L89-L98).

Example task to move the multimeter_1 from its current location to table_2:

```bash
rosrun tables_demo_planning task_server_client.py move_item mobipick multimeter_1 table_2
```

Additionally it is possible to send tasks to the task server using the provided
[ROS action message](https://github.com/DFKI-NI/mobipick_labs/blob/aaf5639ced33866c17eb036fcd22b27cd72cff21/tables_demo_planning/action/PlanAndExecuteTasks.action).
The default topic to send ROS actions to is `/mobipick/task_planning`.


Plan visualization
------------------

Install and source the
[dot_graph_visualization](https://github.com/DFKI-NI/dot_graph_visualization)
rqt plugin, then call it with:

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
```

Citation
--------

If you use this work in your research, consider citing our
[PlanRob 2023 paper](https://icaps23.icaps-conference.org/program/workshops/planrob/PlanRob-23_paper_9.pdf):

```plain
@inproceedings{lima2023physics,
  title={A Physics-Based Simulated Robotics Testbed for Planning and Acting Research},
  author={Lima, O and G{\"u}nther, M and Sung, A and Stock, S and Vinci, M and Smith, A and Krause, JC and Hertzberg, J},
  booktitle={ICAPS Workshop on Planning and Robotics (PlanRob 2023)},
  year={2023}
}
```
