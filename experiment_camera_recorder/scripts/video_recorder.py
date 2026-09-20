#!/usr/bin/env python3

"""Record ROS image topics into videos that only contain the moments the robot moves.

Every image topic in ``~image_topics`` is written to ``<output_dir>/<name>.mp4`` at
its own rate plus ``<name>_<speedup>x.mp4`` (same frames, higher frame rate), so a
4x video of the interesting parts is available without post-processing. With
``~auto_pause`` frames are only written while the robot moves (base command
velocity, arm or gripper joint velocities), so idle phases such as an agent
thinking are cut automatically; the ``~pause`` / ``~resume`` services override that
by hand. ``~snapshot`` (std_msgs/String label) saves the latest frame of every
topic as JPEG, ``~stop`` closes the videos. ``~status`` (std_srvs/Trigger) reports
the state as JSON; ``events.jsonl`` in the output directory records pauses,
resumes and snapshots with ROS and wall time.
"""

import json
import os
import re
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


class TopicRecorder:
    """Videos and snapshots of one image topic."""

    def __init__(self, topic, output_dir, fps, speedup, codec, overlay):
        self.topic = topic
        self.name = _safe_name(topic)
        self.output_dir = output_dir
        self.fps = fps
        self.speedup = speedup
        self.codec = codec
        self.overlay = overlay
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.latest = None
        self.latest_image = None
        self.writer = None
        self.writer_fast = None
        self.size = None
        self.frames = 0
        self.last_written_stamp = None
        self.video_path = os.path.join(output_dir, self.name + ".mp4")
        self.video_fast_path = os.path.join(output_dir, "%s_%gx.mp4" % (self.name, speedup))
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
        self.writer_fast = cv2.VideoWriter(self.video_fast_path, fourcc, self.fps * self.speedup, self.size)
        if not self.writer.isOpened() or not self.writer_fast.isOpened():
            raise RuntimeError("could not open video writers for %s in %s" % (self.topic, self.output_dir))
        rospy.loginfo("%s -> %s (%.3g fps) and %s (%gx)", self.topic, self.video_path, self.fps,
                      self.video_fast_path, self.speedup)

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
        if self.overlay:
            stamp = message.header.stamp if not message.header.stamp.is_zero() else rospy.Time.now()
            text = "%s  t=%.1fs  %s" % (self.name, stamp.to_sec(), label)
            cv2.putText(image, text, (8, image.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 3, cv2.LINE_AA)
            cv2.putText(image, text, (8, image.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
        if self.writer is None:
            self._open(image)
        if image.shape[1::-1] != self.size:
            image = cv2.resize(image, self.size)
        self.writer.write(image)
        self.writer_fast.write(image)
        self.frames += 1
        return True

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
        for writer in (self.writer, self.writer_fast):
            if writer is not None:
                writer.release()
        self.writer = self.writer_fast = None
        self.subscriber.unregister()
        return {"topic": self.topic, "frames": self.frames, "seconds": round(self.frames / self.fps, 1),
                "video": self.video_path if self.frames else None,
                "video_fast": self.video_fast_path if self.frames else None}


class VideoRecorder:
    def __init__(self):
        topics = rospy.get_param("~image_topics", ["/experiment_camera/image_raw"])
        if isinstance(topics, str):
            topics = [t for t in re.split(r"[,\s]+", topics) if t]
        self.fps = float(rospy.get_param("~fps", 10.0))
        self.speedup = float(rospy.get_param("~speedup", 4.0))
        codec = str(rospy.get_param("~codec", "mp4v"))
        overlay = bool(rospy.get_param("~overlay", True))
        self.jpeg_quality = int(rospy.get_param("~jpeg_quality", 90))
        self.auto_pause = bool(rospy.get_param("~auto_pause", True))
        self.motion_hold_s = float(rospy.get_param("~motion_hold_s", 1.5))
        self.joint_velocity_threshold = float(rospy.get_param("~joint_velocity_threshold", 0.02))
        self.joint_pattern = re.compile(str(rospy.get_param("~joint_pattern", "ur5")))
        cmd_vel_topic = rospy.get_param("~cmd_vel_topic", "/mobipick/cmd_vel")
        joint_states_topic = rospy.get_param("~joint_states_topic", "/mobipick/joint_states")
        if self.fps <= 0 or self.speedup < 1:
            raise ValueError("~fps must be positive and ~speedup at least 1")

        self.output_dir = self._resolve_output_dir(rospy.get_param("~output_dir", ""), rospy.get_param("~name", ""))
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
        self.recorders = [TopicRecorder(t, self.output_dir, self.fps, self.speedup, codec, overlay) for t in topics]

        rospy.Subscriber(cmd_vel_topic, Twist, self._cmd_vel_callback, queue_size=1)
        rospy.Subscriber(joint_states_topic, JointState, self._joint_states_callback, queue_size=1)
        rospy.Subscriber("~snapshot", String, self._snapshot_callback, queue_size=10)
        rospy.Service("~pause", Trigger, self._pause_service)
        rospy.Service("~resume", Trigger, self._resume_service)
        rospy.Service("~stop", Trigger, self._stop_service)
        rospy.Service("~status", Trigger, self._status_service)
        self.status_pub = rospy.Publisher("~status", String, queue_size=1, latch=True)
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.fps), self._tick)
        self._log_event("start", topics=topics, auto_pause=self.auto_pause, fps=self.fps, speedup=self.speedup)
        rospy.loginfo("recording %s into %s (auto_pause=%s, %gx copy)", topics, self.output_dir, self.auto_pause,
                      self.speedup)

    @staticmethod
    def _resolve_output_dir(configured, name):
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
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
            "topics": [{"topic": r.topic, "frames": r.frames, "recorded_s": round(r.frames / self.fps, 1),
                        "video": r.video_path, "video_fast": r.video_fast_path} for r in self.recorders],
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
            summary = {"output_dir": self.output_dir, "snapshots": self.snapshots,
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
