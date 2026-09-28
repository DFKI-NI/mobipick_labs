# Experiment camera recorder

This package provides a transparent, static Gazebo RGB camera named
`experiment_camera`. It publishes `sensor_msgs/Image` messages on
`/experiment_camera/image_raw` and saves timestamped JPEG frames at a low rate.

The tables demo starts the camera without recording frames by default:

```bash
roslaunch tables_demo_bringup demo_sim.launch world_config:=${MOBIPICK_WORLD:-moelk_tables}
```

Unless `experiment_image_output_dir` is set, frames are stored in a new run
directory below `~/.ros/log/experiment_camera_frames`. The ROS image timestamp
is embedded in every filename, for example `frame_1724938123_123456789.jpg`.
Recording can be enabled and configured on the demo launch command line:

```bash
roslaunch tables_demo_bringup demo_sim.launch \
  record_experiment_images:=true \
  experiment_image_frequency:=0.1 \
  experiment_image_output_dir:=/data/my_experiment/frames
```

The camera pose depends on the world: `configured_camera.launch` holds one pose
per `world_config` (`moelk_tables`, `cic_tables`); other worlds get the
`camera.launch` default pose. After Gazebo and the camera are running, start
the pose tuner for the world the sim runs:

```bash
roslaunch experiment_camera_recorder pose_tuner.launch world_config:=cic_tables
```

(`world_config` defaults to `$MOBIPICK_WORLD`, else `moelk_tables`.) Its six
sliders, or the number boxes next to them for exact values, update the camera
model in Gazebo while the live image is shown; **Saved pose** jumps to the pose
currently saved for that world. Clicking **OK** rewrites only that world's pose
in `experiment_camera_recorder/launch/configured_camera.launch` in the
workspace. `demo_sim.launch` includes that file automatically, so the saved pose
is used the next time the simulator starts.

## Video of the moving robot

`video_recorder.py` turns one or more image topics into videos that only contain
the moments the robot moves and cannot be blocked by windows on the screen:

```bash
roslaunch experiment_camera_recorder video_recorder.launch \
  image_topics:=/experiment_camera/image_raw,/mobipick/eef_main_cam/rgb/image_raw \
  name:=insert_sugar
```

Videos go to `output_dir` (default `/data/experiment_recordings`, a host folder the
GUI mounts into its containers at the same path).

Per topic it writes `<topic>.mp4` (real time, `fps`, default 10) and
`<topic>_4x.mp4` (same frames at four times the rate, `speedup`). With
`auto_pause:=true` (default) frames are written only while `/mobipick/cmd_vel`
is non-zero or an arm joint (`joint_pattern`, default `ur5`; the simulated gripper finger is noisy at rest) moves faster than
`joint_velocity_threshold`, kept for `motion_hold_s` after the last motion, so
idle phases (an agent thinking, perception) are cut. Manual control:

```bash
rosservice call /experiment_video_recorder/pause      # nothing is written until resume
rosservice call /experiment_video_recorder/resume
rostopic pub -1 /experiment_video_recorder/snapshot std_msgs/String "data: 'sugar grasped'"   # JPEG of every topic
rosservice call /experiment_video_recorder/status     # JSON: frames, recorded seconds, paths
rosservice call /experiment_video_recorder/stop       # closes the videos, writes summary.json, exits
```

The output folder (`<output_dir>/<timestamp>_<name>/`) also holds
`snapshots/NNN_<label>_<topic>.jpg`, `events.jsonl` (pause/resume/snapshot with
ROS and wall time) and `summary.json`.

Every topic also gets `<topic>_frames.jsonl`, one JSON line per frame written to
its two videos, so the videos of different topics (environment and robot camera)
and the snapshot markers of `events.jsonl` can be aligned on one timeline:

```json
{"frame": 0, "source_stamp": 1790621217.34, "wall_time": 1790621219.02, "ros_time": 1790621217.43, "label": "REC"}
```

`frame` is the 0-based index of the frame in `<topic>.mp4` and `<topic>_4x.mp4`
(both hold the same frames), `source_stamp` the ROS header stamp of the image,
`wall_time` and `ros_time` the recorder's clocks when it wrote the frame and
`label` the overlay label. A topic without a fresh image in a recorder tick
writes no frame and no line, so two topics' videos have different frame counts:
align them by `source_stamp` (or `ros_time`), not proportionally. A pause takes
no indices either; it shows as a jump in the times between two consecutive
frames. `summary.json` and the `status` service list the sidecar per topic as
`frames_index`.
