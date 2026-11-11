# octomap_cloud_gate

Forwards a depth point cloud to MoveIt's octomap updater only while the arm is at rest. Generic: any robot whose
camera sits on a moving arm.

**Why.** The simulated wrist camera's depth cloud shows the world a little later than its stamp, while MoveIt masks the
robot as it was at the stamp (`PlanningSceneMonitor::getShapeTransformCache`). While the arm moves, slivers of the arm pass
the mask (the sim padding is 2 cm) and become phantom voxels in the air, which block later plans. Clearing the octomap
before every step, as the test harnesses do, also removes the open-set objects that only the octomap knows, so a later pick finds no obstacle where the object stands.

**What.** `nodes/octomap_cloud_gate.py` subscribes to the camera cloud and to `joint_states` and republishes a cloud only when
every arm joint (names ending in the six UR5 joint names, so a tf prefix does not matter) has been slower than
`~rest_speed` (0.05 rad/s) for `~rest_time` (0.3 s) both before the cloud was taken (its stamp) and before now. Speeds come
from the message velocity, else from the position difference. No joint state for `~stale_time` (1 s): the gate stays closed.
The octomap keeps the map of the last rest. Perception only: nothing is sent to the robot.
Every 30 s the node logs how many clouds passed and were dropped and the clouds' lag (arrival minus stamp, median and
max): the gate watches the arm only, so while the base drives, a lag L shifts the whole cloud by speed * L.

**Use.** `launch/octomap_cloud_gate.launch` in the robot's namespace, and a sensors yaml whose `point_cloud_topic` is the
gate's `cloud_out` (`eef_main_cam/depth_registered/points_at_rest`). `tables_demo_bringup`'s `robot_stack_sim.launch` does
both with the launch arg `octomap_update_only_at_rest` (safe feature flag, off in code, Advanced Launch Option on in the sim),
using `config/sensors_gripper_cam_pointcloud_sim_at_rest.yaml`, a copy of the sim sensors yaml with the other topic.
The real robot's MoveIt starts on the robot with its own launch files; wiring the gate there needs the same two changes.

**Test.** `python3 -m unittest discover test` in a ROS shell (the detector on synthetic joint states and cloud stamps).
