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

After Gazebo and the camera are running, start the pose tuner with:

```bash
roslaunch experiment_camera_recorder pose_tuner.launch
```

Its six sliders update the camera model in Gazebo while the live image is
shown. Clicking **OK** overwrites
`experiment_camera_recorder/launch/configured_camera.launch` in the workspace.
`demo_sim.launch` includes that file automatically, so the saved pose is used
the next time the simulator starts.
