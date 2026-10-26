#!/usr/bin/env python3

"""Record ROS image topics into videos that only contain the moments the robot moves.

Every image topic in ``~image_topics`` is written to ``<output_dir>/<id>_<name>.mp4``; ``<id>`` is a unique video
id ``v<YYYYMMDD>-<HHMMSS>-<4 hex>`` for easy reference, and every frame shows that id and what was run as lime
text in a black strip added on top of the camera picture (the frames grow in height, the picture stays untouched):
``~command`` (the shell command of the experiment) and the last goal that arrived on ``~command_topic`` (default
``/recognized_speech``, where the GUI's speech input and ``agent_experiment.py`` send the GPT demo's goal to the LLM)
with the model profile (the ``~model_label_param`` parameter of the running agent); without either the frames say
so in capitals, and with neither ``~command`` nor ``~command_topic`` the recorder refuses to start; ``video_ids.tsv`` beside the recording folders gets one ``<id>\t<absolute path>`` line per
new video (append-only). There is no sped-up copy (the viewer plays at 4x), and topics that match
``~forbidden_topic_pattern`` (gripper / robot cameras, forbidden since 2026-10-04) are not recorded. With
``~auto_pause`` frames are only written while the robot moves (base command
velocity, arm or gripper joint velocities), so idle phases such as an agent
thinking are cut automatically; the ``~pause`` / ``~resume`` services override that
by hand. ``~snapshot`` (std_msgs/String label) saves the latest frame of every
topic as JPEG, ``~stop`` closes the videos. ``~status`` (std_srvs/Trigger) reports
the state as JSON; ``events.jsonl`` in the output directory records pauses,
resumes and snapshots with ROS and wall time, and ``<id>_<name>_frames.jsonl`` maps every
frame of a topic's video to its source image stamp and the recorder's wall and ROS
time, so videos of different topics can be aligned exactly.

A recorder can be killed without warning (it dies with the shell or container that started it), so with
``~crash_safe`` (default) the videos are fragmented MP4 files (VP8 through GStreamer, a fragment per second) that
stay playable up to the last fragment, and ``summary.json`` is kept current while recording (``"stopped": false``
until a clean stop).
"""

import json
import os
import re
import secrets
import signal
import threading
import time
from datetime import datetime

import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse


def _safe_name(text):
    return re.sub(r"[^A-Za-z0-9_.-]+", "_", text.strip("/")) or "topic"


def video_id(started, token=None):
    """The unique video id ``v<YYYYMMDD>-<HHMMSS>-<4 hex>`` of a video started at datetime ``started``."""
    return "v%s-%s" % (started.strftime("%Y%m%d-%H%M%S"), token or secrets.token_hex(2))


def split_forbidden(topics, pattern):
    """(allowed, refused) image topics: a topic matching regex ``pattern`` (case-insensitive) is a gripper / robot
    camera, whose videos are forbidden; an empty pattern refuses nothing."""
    refused = [t for t in topics if pattern and re.search(pattern, t, re.I)]
    return [t for t in topics if t not in refused], refused


COMMAND_MAX_LINES = 4


def overlay_parts(command, goal="", model="", goal_topic=""):
    """What the frames show about what was run: ``$ <shell command>`` and ``goal (model <m>): <goal text>``; when
    neither is known a capital NO COMMAND line that says what is missing (never a silent placeholder)."""
    command, goal, model = (" ".join(str(t or "").split()) for t in (command, goal, model))
    parts = []
    if command:
        parts.append("$ " + command)
    if goal:
        parts.append("goal%s: %s" % (" (model %s)" % model if model else "", goal))
    if not parts:
        parts.append("NO COMMAND YET: waiting for the goal on %s" % goal_topic if goal_topic
                     else "NO COMMAND GIVEN (pass command:=...)")
    return parts


def find_model_label(param_names, get_param, suffix):
    """The value of the first parameter whose name ends with ``/<suffix>`` (e.g. the GPT demo runner's
    ``experiment_model_label``, its model profile), or "" when there is none or ``suffix`` is empty."""
    if not suffix:
        return ""
    for name in sorted(param_names):
        if name.endswith("/" + suffix.strip("/")):
            try:
                value = str(get_param(name) or "").strip()
            except Exception:  # the parameter vanished meanwhile
                continue
            if value:
                return value
    return ""


def text_width(text, scale):
    """Width of ``text`` in pixels as the strip draws it."""
    return cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, scale, 1)[0][0]


def cut_to_width(line, max_width, scale):
    """``line`` shortened until it fits ``max_width`` pixels with "..." at its end."""
    while line and text_width(line + "...", scale) > max_width:
        line = line[:-1]
    return line.rstrip() + "..."


def command_lines(text, max_width, scale):
    """``text`` wrapped at spaces to lines of at most ``max_width`` pixels (measured as drawn, so the lines use the
    frame width; a word wider than a line is split), at most COMMAND_MAX_LINES lines (the last one cut with "...")."""
    lines, line = [], ""
    for word in text.split():
        candidate = (line + " " + word) if line else word
        if text_width(candidate, scale) <= max_width:
            line = candidate
            continue
        if line:
            lines.append(line)
        while len(word) > 1 and text_width(word, scale) > max_width:
            cut = len(word) - 1
            while cut > 1 and text_width(word[:cut], scale) > max_width:
                cut -= 1
            lines.append(word[:cut])
            word = word[cut:]
        line = word
    if line:
        lines.append(line)
    if len(lines) > COMMAND_MAX_LINES:
        lines = lines[:COMMAND_MAX_LINES - 1] + [cut_to_width(lines[COMMAND_MAX_LINES - 1], max_width, scale)]
    return lines


def crash_safe_pipeline(path, fps, bitrate_kbps):
    """GStreamer pipeline for OpenCV's VideoWriter that writes ``path`` as a fragmented MP4 (VP8, one key frame and
    one fragment per second): every finished fragment is playable, also when the recorder is killed (SIGKILL) and
    never closes the file, while a plain MP4 writes its index only when it is closed."""
    location = path.replace("\\", "\\\\").replace('"', '\\"')
    return ('appsrc ! videoconvert ! vp8enc deadline=1 cpu-used=4 threads=2 target-bitrate=%d keyframe-max-dist=%d ! '
            'qtmux fragment-duration=1000 streamable=true ! filesink location="%s"'
            % (int(bitrate_kbps * 1000), max(int(round(fps)), 1), location))


def write_json_atomically(path, data):
    """``data`` as JSON into ``path`` through a temporary file, so a reader (or a kill) never sees half a file."""
    temporary = path + ".tmp"
    with open(temporary, "w") as handle:
        json.dump(data, handle, indent=2)
    os.replace(temporary, path)


TEXT_COLOR = (0, 255, 0)      # BGR: pure lime green, on the black strip above the picture
STRIP_COLOR = (0, 0, 0)
STRIP_MARGIN = 8              # pixels around the text in the strip
GOAL_SPARE_ROWS = 2           # rows kept free in the strip for a goal that arrives after the video started


TEXT_SCALE = 0.62             # font scale of the strip text in a 1280 pixel wide frame (smaller frames: smaller text)
TEXT_SCALE_MIN = 0.4          # below this the text is not readable any more


def strip_layout(width, picture_height, rows):
    """(scale, row step, text width in pixels, strip height) of the text strip above a ``width`` pixel wide picture:
    the strip holds ``rows`` text lines and the whole frame (strip + picture) gets an even height for the encoder."""
    scale = max(TEXT_SCALE_MIN, min(TEXT_SCALE, TEXT_SCALE * width / 1280.0))
    (_, char_h), baseline = cv2.getTextSize("Mg", cv2.FONT_HERSHEY_SIMPLEX, scale, 1)
    step = int(char_h * 1.6) + 2
    height = 2 * STRIP_MARGIN + step * max(rows, 1) + int(baseline)
    if (height + picture_height) % 2:
        height += 1
    return scale, step, width - 2 * STRIP_MARGIN, height


def fit_rows(lines, rows, max_width, scale):
    """``lines`` cut to ``rows`` lines; a cut shows as "..." at the end of the last line."""
    if len(lines) <= rows:
        return list(lines)
    kept = list(lines[:max(rows, 1)])
    kept[-1] = cut_to_width(kept[-1], max_width, scale)
    return kept


class TopicRecorder:
    """Videos and snapshots of one image topic."""

    def __init__(self, topic, output_dir, fps, codec, overlay, video_id, command="", ids_path=None, crash_safe=True,
                 bitrate_kbps=2000):
        self.topic = topic
        self.name = _safe_name(topic)
        self.id = video_id
        self.command = command
        self.parts = overlay_parts(command)
        self.spare_rows = 0
        self.output_dir = output_dir
        self.fps = fps
        self.codec = codec
        self.crash_safe = crash_safe
        self.bitrate_kbps = bitrate_kbps
        self.overlay = overlay
        self.ids_path = ids_path
        self._header = None
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.latest = None
        self.latest_image = None
        self.writer = None
        self.size = None          # (width, height) of the video: the camera picture plus the text strip on top
        self.picture_size = None  # (width, height) of the camera picture
        self.layout = None        # strip_layout() of the video, fixed when the video opens
        self.rows = 0             # text rows of the strip
        self.frames = 0
        self.last_written_stamp = None
        self.video_path = os.path.join(output_dir, "%s_%s.mp4" % (video_id, self.name))
        # one JSON line per written frame: {"frame", "source_stamp", "wall_time", "ros_time", "label"}; the frame
        # index counts the frames of the video (0-based), skipped and duplicate images take no index and a pause
        # shows as a jump in the times, never as missing indices
        self.frames_index_path = os.path.join(output_dir, "%s_%s_frames.jsonl" % (video_id, self.name))
        self._frames_index = None
        self.subscriber = rospy.Subscriber(topic, Image, self._callback, queue_size=1, buff_size=2 ** 24)

    def _callback(self, message):
        with self.lock:
            self.latest = message

    def _convert(self, message):
        try:
            return self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        except CvBridgeError as error:
            rospy.logerr_throttle(10.0, "%s: could not convert image: %s", self.topic, error)
            return None

    def _open(self, image):
        height, width = image.shape[:2]
        self.picture_size = (width, height)
        with self.lock:
            parts, spare_rows = list(self.parts), self.spare_rows
        # the strip height cannot change within a video: room for the id line, the text known now and the spare rows
        scale, _, max_width, _ = strip_layout(width, height, 1)
        self.rows = 1 + sum(len(command_lines(part, max_width, scale)) for part in parts) + spare_rows
        self.layout = strip_layout(width, height, self.rows)
        self.size = (width, height + self.layout[3])
        self.writer = self._open_writer()
        rospy.loginfo("%s -> %s (%.3g fps)", self.topic, self.video_path, self.fps)
        if self.ids_path:                              # append-only "<id>\t<absolute path>" index of all videos
            try:
                with open(self.ids_path, "a") as handle:
                    handle.write("%s\t%s\n" % (self.id, os.path.abspath(self.video_path)))
            except OSError as error:
                rospy.logerr("could not add %s to %s: %s", self.id, self.ids_path, error)

    def _open_writer(self):
        """The crash-safe writer (crash_safe_pipeline), else OpenCV's FFMPEG writer with ``codec``, whose MP4 is
        unplayable when the recorder is killed before the stop."""
        if self.crash_safe:
            writer = cv2.VideoWriter(crash_safe_pipeline(self.video_path, self.fps, self.bitrate_kbps),
                                     cv2.CAP_GSTREAMER, 0, self.fps, self.size, True)
            if writer.isOpened():
                return writer
            rospy.logwarn("%s: no GStreamer VP8 / qtmux writer, falling back to %s: the video is unplayable if the "
                          "recorder is killed before it is stopped", self.topic, self.codec)
        writer = cv2.VideoWriter(self.video_path, cv2.VideoWriter_fourcc(*self.codec), self.fps, self.size)
        if not writer.isOpened():
            raise RuntimeError("could not open the video writer for %s in %s" % (self.topic, self.output_dir))
        return writer

    def set_overlay(self, parts, spare_rows=0):
        """Show ``parts`` (see overlay_parts) from the next frame on; ``spare_rows`` text rows stay free in the
        strip of a video that opens now, for text that arrives later (a goal)."""
        with self.lock:
            self.parts = list(parts)
            self.spare_rows = max(int(spare_rows), 0)
            self._header = None

    def _with_strip(self, image, status=""):
        """``image`` with the black text strip on top: the video id (and ``status``) in the first line, then what was
        run, in lime; the camera picture itself is not drawn on."""
        scale, step, max_width, strip = self.layout
        with self.lock:                                 # set_overlay may swap the parts from the goal callback
            if self._header is None:
                lines = [line for part in self.parts for line in command_lines(part, max_width, scale)]
                self._header = fit_rows(lines, self.rows - 1, max_width, scale)
            lines = [("%s   %s" % (self.id, status)).rstrip()] + self._header
        frame = cv2.copyMakeBorder(image, strip, 0, 0, 0, cv2.BORDER_CONSTANT, value=STRIP_COLOR)
        for row, line in enumerate(lines):
            cv2.putText(frame, line, (STRIP_MARGIN, STRIP_MARGIN + step * (row + 1) - 4), cv2.FONT_HERSHEY_SIMPLEX,
                        scale, TEXT_COLOR, 1, cv2.LINE_AA)
        return frame

    def write_latest(self, label):
        """Write the newest frame unless it was written already; returns True when a frame was written."""
        with self.lock:
            message = self.latest
        if message is None or message.header.stamp == self.last_written_stamp:
            return False
        image = self._convert(message)
        if image is None:
            return False
        self.last_written_stamp = message.header.stamp
        if self.writer is None:
            self._open(image)
        if image.shape[1::-1] != self.picture_size:
            image = cv2.resize(image, self.picture_size)
        status = ""
        if self.overlay:
            stamp = message.header.stamp if not message.header.stamp.is_zero() else rospy.Time.now()
            status = "%s  t=%.1fs  %s" % (self.name, stamp.to_sec(), label)
        self.writer.write(self._with_strip(image, status))
        self._record_frame_time(self.frames, message.header.stamp, label)
        self.frames += 1
        return True

    def _record_frame_time(self, index, stamp, label):
        """Append the acquisition and recording times of output frame ``index`` to the frames sidecar."""
        if self._frames_index is None:
            self._frames_index = open(self.frames_index_path, "a")
        entry = {"frame": index, "source_stamp": stamp.to_sec(), "wall_time": time.time(),
                 "ros_time": rospy.Time.now().to_sec(), "label": label}
        self._frames_index.write(json.dumps(entry) + "\n")
        self._frames_index.flush()

    def snapshot(self, label, index, quality):
        with self.lock:
            message = self.latest
        if message is None:
            return None
        image = self._convert(message)
        if image is None:
            return None
        path = os.path.join(self.output_dir, "snapshots", "%03d_%s_%s.jpg" % (index, _safe_name(label), self.name))
        os.makedirs(os.path.dirname(path), exist_ok=True)
        if not cv2.imwrite(path, image, [cv2.IMWRITE_JPEG_QUALITY, quality]):
            rospy.logerr("failed to write snapshot %s", path)
            return None
        return path

    def close(self):
        if self.writer is not None:
            self.writer.release()
        self.writer = None
        if self._frames_index is not None:
            self._frames_index.close()
            self._frames_index = None
        self.subscriber.unregister()
        return self.summary()

    def summary(self):
        return {"topic": self.topic, "id": self.id, "frames": self.frames, "seconds": round(self.frames / self.fps, 1),
                "video": self.video_path if self.frames else None,
                "frames_index": self.frames_index_path if self.frames else None}


class VideoRecorder:
    def __init__(self):
        topics = rospy.get_param("~image_topics", ["/experiment_camera/image_raw"])
        if isinstance(topics, str):
            topics = [t for t in re.split(r"[,\s]+", topics) if t]
        forbidden = str(rospy.get_param("~forbidden_topic_pattern", r"gripper|eef|wrist|hand_cam|arm_cam"))
        topics, refused = split_forbidden(topics, forbidden)
        if refused:
            rospy.logwarn("not recording %s: gripper / robot camera videos are forbidden (~forbidden_topic_pattern %r)",
                          refused, forbidden)
        if not topics:
            raise ValueError("no image topic left to record (refused: %s)" % refused)
        self.command = str(rospy.get_param("~command", "")).strip()
        self.command_topic = str(rospy.get_param("~command_topic", "/recognized_speech")).strip()
        self.model_label_param = str(rospy.get_param("~model_label_param", "experiment_model_label")).strip()
        if not self.command and not self.command_topic:
            raise ValueError("nothing tells what is run: pass command:=\"<the command>\" or command_topic:=<the goal "
                             "topic> (a video must show what was run, #349)")
        if not self.command:
            rospy.logwarn("~command is empty: the videos show the goal that arrives on %s (with the model profile) "
                          "and 'NO COMMAND YET' until then", self.command_topic)
        self.goals = []
        self.fps = float(rospy.get_param("~fps", 10.0))
        codec = str(rospy.get_param("~codec", "mp4v"))
        crash_safe = bool(rospy.get_param("~crash_safe", True))
        bitrate_kbps = float(rospy.get_param("~bitrate_kbps", 2000.0))
        self.summary_period_s = float(rospy.get_param("~summary_period_s", 5.0))
        overlay = bool(rospy.get_param("~overlay", True))
        self.jpeg_quality = int(rospy.get_param("~jpeg_quality", 90))
        self.auto_pause = bool(rospy.get_param("~auto_pause", True))
        self.motion_hold_s = float(rospy.get_param("~motion_hold_s", 1.5))
        self.joint_velocity_threshold = float(rospy.get_param("~joint_velocity_threshold", 0.02))
        self.joint_pattern = re.compile(str(rospy.get_param("~joint_pattern", "ur5")))
        cmd_vel_topic = rospy.get_param("~cmd_vel_topic", "/mobipick/cmd_vel")
        joint_states_topic = rospy.get_param("~joint_states_topic", "/mobipick/joint_states")
        if self.fps <= 0:
            raise ValueError("~fps must be positive")

        started = datetime.now()
        self.output_dir = self._resolve_output_dir(rospy.get_param("~output_dir", ""), rospy.get_param("~name", ""),
                                                   started)
        os.makedirs(self.output_dir, exist_ok=True)
        self.events_path = os.path.join(self.output_dir, "events.jsonl")
        self.summary_path = os.path.join(self.output_dir, "summary.json")
        self.summary_written = 0.0
        self.lock = threading.Lock()
        self.manual_paused = bool(rospy.get_param("~start_paused", False))
        self.last_motion = None
        self.moving = False
        self.recording = False
        self.stopped = False
        self.snapshots = 0
        self.started_wall = time.time()
        ids = set()
        while len(ids) < len(topics):                 # one id per video, unique also within this recording
            ids.add(video_id(started))
        ids_path = os.path.join(os.path.dirname(self.output_dir), "video_ids.tsv")
        self.recorders = [TopicRecorder(t, self.output_dir, self.fps, codec, overlay, i, self.command, ids_path,
                                        crash_safe, bitrate_kbps)
                          for t, i in zip(topics, sorted(ids))]
        self._set_overlays()
        self._write_summary()

        rospy.Subscriber(cmd_vel_topic, Twist, self._cmd_vel_callback, queue_size=1)
        rospy.Subscriber(joint_states_topic, JointState, self._joint_states_callback, queue_size=1)
        rospy.Subscriber("~snapshot", String, self._snapshot_callback, queue_size=10)
        if self.command_topic:
            rospy.Subscriber(self.command_topic, String, self._goal_callback, queue_size=10)
        rospy.Service("~pause", Trigger, self._pause_service)
        rospy.Service("~resume", Trigger, self._resume_service)
        rospy.Service("~stop", Trigger, self._stop_service)
        rospy.Service("~status", Trigger, self._status_service)
        self.status_pub = rospy.Publisher("~status", String, queue_size=1, latch=True)
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.fps), self._tick)
        self._log_event("start", topics=topics, ids=[r.id for r in self.recorders], command=self.command,
                        command_topic=self.command_topic, refused=refused, auto_pause=self.auto_pause, fps=self.fps)
        rospy.loginfo("recording %s into %s as %s (auto_pause=%s)", topics, self.output_dir,
                      [r.id for r in self.recorders], self.auto_pause)

    def _overlay_parts(self):
        goal = self.goals[-1] if self.goals else {}
        return overlay_parts(self.command, goal.get("goal", ""), goal.get("model", ""), self.command_topic)

    def _set_overlays(self):
        """Give every video its text; while a goal can still arrive, rows for it stay free in the strip."""
        spare_rows = GOAL_SPARE_ROWS if self.command_topic and not self.goals else 0
        for recorder in self.recorders:
            recorder.set_overlay(self._overlay_parts(), spare_rows)

    def _goal_callback(self, message):
        """A goal for the agent (the GPT demo's LLM): shown in every following frame with the model profile."""
        text = " ".join(str(message.data or "").split())
        if not text:
            return
        try:
            model = find_model_label(rospy.get_param_names(), rospy.get_param, self.model_label_param)
        except Exception as error:  # the parameter server is unreachable: show the goal without the model
            rospy.logwarn("could not read the model profile: %s", error)
            model = ""
        with self.lock:
            self.goals.append({"goal": text, "model": model, "topic": self.command_topic})
            self._log_event("goal", goal=text, model=model, topic=self.command_topic)
            self._set_overlays()
        rospy.loginfo("goal on %s (model %s): %s", self.command_topic, model or "unknown", text)

    @staticmethod
    def _resolve_output_dir(configured, name, started=None):
        stamp = (started or datetime.now()).strftime("%Y%m%d_%H%M%S")
        folder = "%s_%s" % (stamp, _safe_name(name)) if name else stamp
        if configured:
            return os.path.join(os.path.abspath(os.path.expanduser(configured)), folder)
        ros_home = os.environ.get("ROS_HOME", os.path.expanduser("~/.ros"))
        log_root = os.environ.get("ROS_LOG_DIR", os.path.join(ros_home, "log"))
        return os.path.join(log_root, "experiment_camera_videos", folder)

    # --- motion detection ---------------------------------------------------

    def _cmd_vel_callback(self, message):
        if any(abs(v) > 1e-3 for v in (message.linear.x, message.linear.y, message.angular.z)):
            self.last_motion = time.time()

    def _joint_states_callback(self, message):
        for name, velocity in zip(message.name, message.velocity):
            if self.joint_pattern.search(name) and abs(velocity) > self.joint_velocity_threshold:
                self.last_motion = time.time()
                return

    # --- recording loop -----------------------------------------------------

    def _tick(self, _event):
        with self.lock:
            if self.stopped:
                return
            now = time.time()
            moving = self.last_motion is not None and now - self.last_motion <= self.motion_hold_s
            should_record = not self.manual_paused and (moving or not self.auto_pause)
            if should_record != self.recording:
                self.recording = should_record
                self._log_event("resume" if should_record else "pause", reason="manual" if self.manual_paused else "motion")
                self._publish_status()
                self._write_summary()
            if not should_record:
                return
            label = "REC" if moving else "REC idle"
            for recorder in self.recorders:
                recorder.write_latest(label)
            if now - self.summary_written >= self.summary_period_s:
                self._write_summary()

    def _status(self):
        return {
            "output_dir": self.output_dir,
            "recording": self.recording,
            "manual_paused": self.manual_paused,
            "auto_pause": self.auto_pause,
            "stopped": self.stopped,
            "snapshots": self.snapshots,
            "elapsed_s": round(time.time() - self.started_wall, 1),
            "command": self.command,
            "goals": self.goals,
            "topics": [{"topic": r.topic, "id": r.id, "frames": r.frames, "recorded_s": round(r.frames / self.fps, 1),
                        "video": r.video_path, "frames_index": r.frames_index_path} for r in self.recorders],
        }

    def _publish_status(self):
        self.status_pub.publish(String(data=json.dumps(self._status())))

    def _summary(self, topics=None):
        return {"output_dir": self.output_dir, "stopped": self.stopped, "command": self.command, "goals": self.goals,
                "command_missing": not self.command and not self.goals, "snapshots": self.snapshots,
                "topics": topics if topics is not None else [r.summary() for r in self.recorders]}

    def _write_summary(self, summary=None):
        """summary.json as of now: also a recorder that is killed leaves one (``"stopped": false``, the frame counts
        of at most ``~summary_period_s`` ago)."""
        try:
            write_json_atomically(self.summary_path, summary or self._summary())
        except OSError as error:
            rospy.logerr_throttle(60.0, "could not write %s: %s", self.summary_path, error)
        self.summary_written = time.time()

    def _log_event(self, kind, **data):
        entry = {"event": kind, "wall_time": time.time(), "ros_time": rospy.Time.now().to_sec()}
        entry.update(data)
        with open(self.events_path, "a") as handle:
            handle.write(json.dumps(entry) + "\n")

    # --- services -----------------------------------------------------------

    def _pause_service(self, _request):
        with self.lock:
            self.manual_paused = True
            self._log_event("pause_requested")
        return TriggerResponse(success=True, message=json.dumps(self._status()))

    def _resume_service(self, _request):
        with self.lock:
            self.manual_paused = False
            self._log_event("resume_requested")
        return TriggerResponse(success=True, message=json.dumps(self._status()))

    def _status_service(self, _request):
        return TriggerResponse(success=True, message=json.dumps(self._status()))

    def _snapshot_callback(self, message):
        label = message.data or "snapshot"
        with self.lock:
            self.snapshots += 1
            index = self.snapshots
            paths = [p for p in (r.snapshot(label, index, self.jpeg_quality) for r in self.recorders) if p]
            self._log_event("snapshot", label=label, paths=paths)
        rospy.loginfo("snapshot %d %r: %s", index, label, paths)

    def _stop_service(self, _request):
        summary = self.stop()
        return TriggerResponse(success=True, message=json.dumps(summary))

    def stop(self):
        with self.lock:
            if self.stopped:
                return self._status()
            self.stopped = True
            self.timer.shutdown()
            summary = self._summary([r.close() for r in self.recorders])
            if summary["command_missing"]:
                rospy.logerr("no command and no goal on %s during the recording: the videos say 'NO COMMAND YET'",
                             self.command_topic)
            self._log_event("stop", **summary)
            self._write_summary(summary)
            self._publish_status()
        rospy.loginfo("recording stopped: %s", json.dumps(summary))
        rospy.Timer(rospy.Duration(0.5), lambda _e: rospy.signal_shutdown("recording stopped"), oneshot=True)
        return summary


if __name__ == "__main__":
    rospy.init_node("experiment_video_recorder")
    try:
        recorder = VideoRecorder()
    except (OSError, ValueError, RuntimeError) as error:
        rospy.logfatal(str(error))
        raise SystemExit(1)
    else:
        rospy.on_shutdown(recorder.stop)
        # rospy stops cleanly on SIGINT and SIGTERM; a closed terminal (SIGHUP) closes the videos the same way
        signal.signal(signal.SIGHUP, lambda _signum, _frame: rospy.signal_shutdown("SIGHUP"))
        rospy.spin()
