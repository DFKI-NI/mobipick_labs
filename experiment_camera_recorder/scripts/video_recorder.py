#!/usr/bin/env python3

"""Record ROS image topics into videos that only contain the moments the robot moves.

Every image topic in ``~image_topics`` is written to ``<output_dir>/<id>_<name>.mp4``; ``<id>`` is a unique video
id ``v<YYYYMMDD>-<HHMMSS>-<4 hex>`` for easy reference, and every frame shows that id and ``~command`` (the command
of the experiment) as text; ``video_ids.tsv`` beside the recording folders gets one ``<id>\t<absolute path>`` line per
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
"""

import json
import os
import re
import secrets
import textwrap
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


def command_lines(command, width_chars):
    """``command`` wrapped to ``width_chars`` per line, at most COMMAND_MAX_LINES lines (the last one cut with "...")."""
    text = " ".join(command.split()) or "command not given"
    lines = textwrap.wrap("$ " + text, max(width_chars, 10), break_long_words=True, break_on_hyphens=False)
    if len(lines) > COMMAND_MAX_LINES:
        lines = lines[:COMMAND_MAX_LINES]
        lines[-1] = lines[-1][:max(width_chars - 3, 1)] + "..."
    return lines


def _put_text(image, text, origin, scale):
    """White text with a black outline, readable on any background."""
    cv2.putText(image, text, origin, cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(image, text, origin, cv2.FONT_HERSHEY_SIMPLEX, scale, (255, 255, 255), 1, cv2.LINE_AA)


class TopicRecorder:
    """Videos and snapshots of one image topic."""

    def __init__(self, topic, output_dir, fps, codec, overlay, video_id, command="", ids_path=None):
        self.topic = topic
        self.name = _safe_name(topic)
        self.id = video_id
        self.command = command
        self.output_dir = output_dir
        self.fps = fps
        self.codec = codec
        self.overlay = overlay
        self.ids_path = ids_path
        self._header = None
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.latest = None
        self.latest_image = None
        self.writer = None
        self.size = None
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
        self.size = (width, height)
        fourcc = cv2.VideoWriter_fourcc(*self.codec)
        self.writer = cv2.VideoWriter(self.video_path, fourcc, self.fps, self.size)
        if not self.writer.isOpened():
            raise RuntimeError("could not open the video writer for %s in %s" % (self.topic, self.output_dir))
        rospy.loginfo("%s -> %s (%.3g fps)", self.topic, self.video_path, self.fps)
        if self.ids_path:                              # append-only "<id>\t<absolute path>" index of all videos
            try:
                with open(self.ids_path, "a") as handle:
                    handle.write("%s\t%s\n" % (self.id, os.path.abspath(self.video_path)))
            except OSError as error:
                rospy.logerr("could not add %s to %s: %s", self.id, self.ids_path, error)

    def _draw_header(self, image):
        """The video id and the command, top left, in every frame (wrapped to the frame width once per size)."""
        width = image.shape[1]
        if self._header is None or self._header[0] != width:
            scale = max(0.4, min(1.0, width / 1280.0))
            (char_w, char_h), _ = cv2.getTextSize("M" * 10, cv2.FONT_HERSHEY_SIMPLEX, scale, 1)
            lines = [self.id] + command_lines(self.command, int((width - 16) * 10 / max(char_w, 1)))
            self._header = (width, scale, int(char_h * 1.6) + 2, lines)
        _, scale, step, lines = self._header
        for row, line in enumerate(lines):
            _put_text(image, line, (8, 8 + step * (row + 1) - 4), scale)

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
        if image.shape[1::-1] != self.size:
            image = cv2.resize(image, self.size)
        self._draw_header(image)
        if self.overlay:
            stamp = message.header.stamp if not message.header.stamp.is_zero() else rospy.Time.now()
            _put_text(image, "%s  t=%.1fs  %s" % (self.name, stamp.to_sec(), label), (8, image.shape[0] - 10), 0.5)
        self.writer.write(image)
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
        self.command = str(rospy.get_param("~command", ""))
        if not self.command.strip():
            rospy.logwarn("~command is empty: the videos show 'command not given'; pass command:=\"<the command>\"")
        self.fps = float(rospy.get_param("~fps", 10.0))
        codec = str(rospy.get_param("~codec", "mp4v"))
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
        self.recorders = [TopicRecorder(t, self.output_dir, self.fps, codec, overlay, i, self.command, ids_path)
                          for t, i in zip(topics, sorted(ids))]

        rospy.Subscriber(cmd_vel_topic, Twist, self._cmd_vel_callback, queue_size=1)
        rospy.Subscriber(joint_states_topic, JointState, self._joint_states_callback, queue_size=1)
        rospy.Subscriber("~snapshot", String, self._snapshot_callback, queue_size=10)
        rospy.Service("~pause", Trigger, self._pause_service)
        rospy.Service("~resume", Trigger, self._resume_service)
        rospy.Service("~stop", Trigger, self._stop_service)
        rospy.Service("~status", Trigger, self._status_service)
        self.status_pub = rospy.Publisher("~status", String, queue_size=1, latch=True)
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.fps), self._tick)
        self._log_event("start", topics=topics, ids=[r.id for r in self.recorders], command=self.command,
                        refused=refused, auto_pause=self.auto_pause, fps=self.fps)
        rospy.loginfo("recording %s into %s as %s (auto_pause=%s)", topics, self.output_dir,
                      [r.id for r in self.recorders], self.auto_pause)

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
            if not should_record:
                return
            label = "REC" if moving else "REC idle"
            for recorder in self.recorders:
                recorder.write_latest(label)

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
            "topics": [{"topic": r.topic, "id": r.id, "frames": r.frames, "recorded_s": round(r.frames / self.fps, 1),
                        "video": r.video_path, "frames_index": r.frames_index_path} for r in self.recorders],
        }

    def _publish_status(self):
        self.status_pub.publish(String(data=json.dumps(self._status())))

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
            summary = {"output_dir": self.output_dir, "command": self.command, "snapshots": self.snapshots,
                       "topics": [r.close() for r in self.recorders]}
            self._log_event("stop", **summary)
            with open(os.path.join(self.output_dir, "summary.json"), "w") as handle:
                json.dump(summary, handle, indent=2)
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
    else:
        rospy.on_shutdown(recorder.stop)
        rospy.spin()
