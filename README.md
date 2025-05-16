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

Launch file structure and world configs
---------------------------------------

This section documents the launch file structure in `mobipick_labs` and
`mobipick`, with a focus on how to add a new "world config" (i.e., the config
files required to run the system in a new environment / with a different table
arrangement etc.).

### Launch File Structure for Gazebo Simulation

The top-level launch file for the Gazebo simulation is `demo_sim.launch`. This file accepts several key arguments:

#### Example Command

```bash
roslaunch tables_demo_bringup demo_sim.launch \
  world:=pbr_cic \
  world_config:=cic_tables \
  robot_x:=20.50 \
  robot_y:=15.20 \
  robot_yaw:=1.57
```

#### Important Arguments

1. **world**: Specifies which Gazebo world to load (only the building, no tables or objects).
   - **Options**: `pbr_moelk` (default), `pbr_cic`
   - **Effect**: Passed to `mobipick_gazebo/tables_demo.launch` to spawn the
     specified Gazebo world.

2. **world_config**: Defines the arrangement of objects and tables.
   - **Options**: `moelk_tables`, `cic_tables`, `truck_assembly`
   - **Effect**:
      - Passed to `mobipick_gazebo/tables_demo.launch` to launch
        `mobipick_gazebo/launch/worlds/<world_config>_spawn_sim_objects.launch`,
        which spawns the scenario-specific table and object arrangement in
        Gazebo.
      - Includes `includes/navigation/<world_config>.launch` to set up
        `move_base` and localization with the appropriate maps and virtual
        walls.
      - Passed to `tables_demo_bringup/launch/bringup.launch` to load:
         - `tables_demo_bringup/config/<world_config>_planning_scene.yaml` for
           grasplan (defines MoveIt planning scene collision boxes, including
           tables, walls, and ceiling).
         - `mobipick_pick_n_place/config/<world_config>_demo.yaml` for
           `tables_demo_planning` (defines move_base target poses in front of
           tables, home pose, handover pose, etc.).

3. **robot_x, robot_y, robot_yaw**: Specifies the robot's initial position and orientation in the simulation.
   - **Requirement**: Must match the selected `world` and `world_config`.
   - **Suggestion**: Ideally, the `base_home_pose` from
     `mobipick_pick_n_place/config/<world_config>_demo.yaml` should be used
     here.
   - **Effect**: Passed to `mobipick_gazebo/tables_demo.launch`.

#### World Config Definitions

Each `world_config` is characterized by four scenario-specific launch and YAML files:

1. **Spawn Simulation Objects**:
   - `mobipick/mobipick_gazebo/launch/worlds/<world_config>_spawn_sim_objects.launch`
   - **Purpose**: Spawns tables and objects in Gazebo.

2. **Base Pose Configuration**:
   - `mobipick/mobipick_pick_n_place/config/<world_config>_demo.yaml`
   - **Purpose**: Defines `move_base` goal poses in front of the tables, home pose, handover pose, and other parameters.

3. **Planning Scene Configuration**:
   - `mobipick_labs/tables_demo_bringup/config/<world_config>_planning_scene.yaml`
   - **Purpose**: Defines planning scene boxes for MoveIt (tables, walls,
     ceiling, other static obstacles); also used by grasplan (for picking and
     placing from tables).

4. **Navigation Launch File**:
   - `mobipick_labs/tables_demo_bringup/launch/includes/navigation/<world_config>.launch`
   - **Purpose**: Launches `move_base` with the appropriate map and virtual
     walls matching the environment and table arrangement. Maps are typically
     stored in the `pbr_maps` repository.

### Running on Real Robot vs. Gazebo Simulation

- **Real Robot**:
   - Only **Base Pose Configuration** (2) and **Planning Scene Configuration** (3) are required.
   - Launch only `bringup.launch`.

- **Gazebo Simulation**:
   - All four files are required.
   - Launch `demo_sim.launch` which orchestrates the entire setup.

Adapting to a new environment
-----------------------------

To add a new `world_config` and adapt the demo to a new physical environment, follow these steps:

### A. Mapping and Configuration

1. **Map the Environment**:
   - Use the MiR web interface to map the new environment.

2. **Define Forbidden Areas**:
   - In the MiR web interface, draw forbidden areas for tables and other obstacles.
   - **Note**: Be precise; overly generous forbidden zones may prevent the robot from navigating close to tables.

3. **Determine move_base Goal Poses**:
   - Launch RViz and use it to send the robot to various poses in front of each table.
   - Move the robot arm to the `observe100cm_right` configuration using:

    ```bash
    roslaunch mobipick_moveit_config moveit_rviz.launch
    ```

   - Verify that the front edge of each table is just visible in the camera image.
   - While navigating, run:

    ```bash
    rostopic echo /mobipick/move_base/goal
    ```

    Capture the poses displayed in the terminal and use them to create the
    `mobipick_pick_n_place/config/<world_config>_demo.yaml` file (Base Pose
    Configuration).

4. **Create Planning Scene Boxes**:
   - With the robot arm still in the `observe100cm_right` configuration, record
     a rosbag while manually controlling the robot to drive slowly around the
     scene, observing all tables and their edges.
   - Use the recorded rosbag with the helper launch file to generate
     `mobipick_labs/tables_demo_bringup/config/<world_config>_planning_scene.yaml`
     (Planning Scene Configuration) by running:

     ```bash
     roslaunch grasplan find_planning_scene_boxes_helper.launch
     ```

### B. Setting Up Gazebo Simulation (Optional)

If a Gazebo simulation for the new environment is desired, proceed with the following:

1. **Export the Map**:
   - Use the MiR web interface to export the map, or run:

     ```bash
     rosrun map_server map_saver map:=/mobipick/map
     ```

2. **Process the Maps**:
   - Extract both the regular occupancy map and the forbidden zones map (virtual walls).
   - Convert these maps to PNG format.
   - Create accompanying YAML files and save them in the `pbr_maps` repository.

3. **Create Navigation Launch File**:
   - Develop a launch file that loads the processed maps.
   - This will be your
     `mobipick_labs/tables_demo_bringup/launch/includes/navigation/<world_config>.launch`
     (Navigation Launch File).

4. **Create Gazebo Launch File**:
   - Develop a Gazebo launch file that aligns table positions with the new map.
   - This will be your
     `mobipick/mobipick_gazebo/launch/worlds/<world_config>_spawn_sim_objects.launch`
     (Spawn Simulation Objects).


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
