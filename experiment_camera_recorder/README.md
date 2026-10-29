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
roslaunch experiment_camera_recorder video_recorder.launch name:=insert_sugar \
  command:="rosrun mobipick_gpt agent_experiment.py --goal 'insert the sugar'"
```

Video rules (2026-10-04): the experiment camera only, gripper / robot camera videos are forbidden (topics matching
`forbidden_topic_pattern`, default `gripper|eef|wrist|hand_cam|arm_cam`, are refused with a warning); no sped-up copy
(viewers such as the Agent Workbench play a video at 4x); every frame shows the video id and what was run: `command` (the shell command
that runs the experiment, as `$ <command>`) and the last goal that arrived on `command_topic` (default
`/recognized_speech`, where the GUI's speech input and `agent_experiment.py` send the GPT demo's goal to the LLM) as
`goal (model <profile>): <text>`; the model profile is the first parameter named `*/<model_label_param>` (default
`experiment_model_label`, set by `gpt_robot_demo.launch` from its `model_profile`). Each is wrapped to at most four
lines that use the frame width (measured as drawn), in small text (font scale 0.62 in a 1280 pixel wide frame). The text is never drawn over the camera picture: every frame grows in height by a black strip on top of the
picture and the text stands there in lime, first line the video id with the topic, time and `REC` state
(`overlay:=false` leaves the id only), then the command and the goal. The strip height fits the text of the moment the video
opens (the first written frame; the frame height stays even for the encoder) and is fixed for that video; while a goal
can still arrive two rows stay free for it, and later text that needs more rows is cut with `...`. Until either is known the frames say `NO COMMAND YET: waiting for the goal on <topic>`, `summary.json` gets
`"command_missing": true` and the recorder logs an error at stop; with `command` and `command_topic` both empty it
refuses to start. A GPT demo run therefore needs no `command`: the goal is taken from the run itself.

Videos go to `output_dir` (default `/data/experiment_recordings`, a host folder the
GUI mounts into its containers at the same path).

### One video per robot of a multi-robot sim

With `num_robots` 2 or 3 the tables demo sim (`demo_sim.launch`) gives every scene copy its own overview camera:
`/experiment_cameraK/image_raw` for robot K stands at the tuned pose of the first camera shifted along x by the copy
offset, so it sees its copy exactly as the first camera sees the first (`experiment_camera_copies:=false` leaves
the extra cameras out). One recorder records them all, one video per robot:

```bash
roslaunch experiment_camera_recorder video_recorder.launch \
  image_topics:="/experiment_camera/image_raw /experiment_camera2/image_raw" topic_robots:="mobipick mobipick2" \
  title:=two-robots-run-1 command:="<the command that was run>"
```

`topic_robots` names the robot each camera shows: every video gets its own id, the same text strip, a title that
starts with its robot (`<id>_mobipick2-two-robots-run-1.mp4`), and with `auto_pause` it is written while that robot
moves (`<ns>/cmd_vel`, `<ns>/joint_states`) and in the lead-in. `robot_namespaces` sets the robots whose motion
counts when `topic_robots` is empty (default `mobipick`); the `cmd_vel_topic` and `joint_states_topic` parameters
still override the motion topics, and then count for every video.

Per topic it writes `<id>_<topic>.mp4` (real time, `fps`, default 10). `<id>` is the video's unique id
`v<YYYYMMDD>-<HHMMSS>-<4 hex>` (start time on the recorder's clock, UTC in the GUI containers like the folder name, plus
4 random hex digits), shown first in the text strip of every frame, above the command, and appended as `<id>\t<absolute path>`
to `<output_dir>/video_ids.tsv` (append-only, one line per video, so an id finds its file).
`title:=<short title>` puts a readable name into the file name after the id,
`<id>_<title>.mp4` (several topics: `<id>_<title>_<topic>.mp4`), so a video can be told apart in a folder and
linked with words: the title is lower-cased, every run of characters other than `a-z`, `0-9` is one `-`, and it is cut
at 40 characters; empty (default) keeps `<id>_<topic>.mp4`. The `video_ids.tsv` line stays `<id>\t<absolute path>`
(the title is in the path), and the id in the frames is unchanged. Whoever starts the recorder for a test passes it
together with `command:=`, e.g. `title:="Baseline run 1: drill pick, sim"` (the link text shown to the user says the
same in plain words). Existing videos keep their names. With
`auto_pause:=true` (default) frames are written only while `/mobipick/cmd_vel`
is non-zero or an arm joint (`joint_pattern`, default `ur5`; the simulated gripper finger is noisy at rest) moves faster than
`joint_velocity_threshold`, kept for `motion_hold_s` after the last motion, so
idle phases (an agent thinking, perception) are cut. The first `lead_in_s` seconds (default 3) and the `lead_in_s`
seconds after each goal on `command_topic` are written also while the robot stands still, so a run in which the robot
never moves (e.g. a refused plan) still gives a short video with the command and the robot at rest. For a test video keep
`auto_pause` on and stop the recorder as soon as the command returns, in the same shell line
(`<command>; rosservice call /experiment_video_recorder/stop`), so the video ends with the task's result;
`auto_pause:=false` writes every idle second until the stop. Manual control:

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

The recorder can be killed without warning, e.g. together with the shell or container that started it (a closed
GUI shell is removed with `docker stop`, which ends in SIGKILL), and a plain MP4 written by OpenCV gets its index
only when it is closed: a killed recording was unplayable ("moov atom not found"). So with `crash_safe:=true`
(default) every video is a fragmented MP4 (H.264 through OpenCV's GStreamer backend, `bitrate_kbps`, default 2000,
one key frame and one fragment per second) that plays up to its last finished fragment after a kill, and
`summary.json` is rewritten while recording (on every pause/resume, at least every 5 s) with `"stopped": false`
until the stop writes the final one, so a killed run still has a summary. H.264 needs `x264enc` from
`gstreamer1.0-plugins-ugly`; without it the recorder writes VP8 (`vp8enc`), which survives a kill too but
VLC cannot play (transcode it with `ffmpeg -i <video> -c:v libx264 -pix_fmt yuv420p <copy>.mp4`), and warns;
without `qtmux` it falls back to `codec` (default `avc1`, H.264 through OpenCV's FFMPEG backend) and warns.
SIGINT, SIGTERM and SIGHUP close the videos cleanly. Before linking a video anywhere, check that `ffprobe` shows
`codec_name=h264` and that `ffmpeg -v error -i <video> -f null -` prints nothing.

Every topic also gets `<id>_<topic>_frames.jsonl`, one JSON line per frame written to
its video, so the videos of different topics (environment and robot camera)
and the snapshot markers of `events.jsonl` can be aligned on one timeline:

```json
{"frame": 0, "source_stamp": 1790621217.34, "wall_time": 1790621219.02, "ros_time": 1790621217.43, "label": "REC"}
```

`frame` is the 0-based index of the frame in `<id>_<topic>.mp4`, `source_stamp` the ROS header stamp of the image,
`wall_time` and `ros_time` the recorder's clocks when it wrote the frame and
`label` the overlay label. A topic without a fresh image in a recorder tick
writes no frame and no line, so two topics' videos have different frame counts:
align them by `source_stamp` (or `ros_time`), not proportionally. A pause takes
no indices either; it shows as a jump in the times between two consecutive
frames. `summary.json` and the `status` service list the sidecar per topic as
`frames_index`, with the topic's `id`, the `command` and the `goals` (text, model, topic; also `goal` lines in
`events.jsonl`).
