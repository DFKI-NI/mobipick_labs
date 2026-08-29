#!/usr/bin/env python3

"""Save low-frequency, timestamped JPEG frames from a ROS image topic."""

import os
import threading
from datetime import datetime

import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image


class FrameRecorder:
    def __init__(self):
        self._bridge = CvBridge()
        self._lock = threading.Lock()
        self._latest = None
        self._last_saved_stamp = None

        topic = rospy.get_param("~image_topic", "/experiment_camera/image_raw")
        frequency = float(rospy.get_param("~frequency", 0.2))
        if frequency <= 0.0:
            raise ValueError("~frequency must be greater than zero")
        self._jpeg_quality = int(rospy.get_param("~jpeg_quality", 90))
        self._output_dir = self._resolve_output_dir(rospy.get_param("~output_dir", ""))
        os.makedirs(self._output_dir, exist_ok=True)

        self._subscriber = rospy.Subscriber(topic, Image, self._image_callback, queue_size=1)
        self._timer = rospy.Timer(rospy.Duration(1.0 / frequency), self._save_latest)
        rospy.loginfo("Recording %.3g frame(s)/s from %s in %s", frequency, topic, self._output_dir)

    @staticmethod
    def _resolve_output_dir(configured_path):
        if configured_path:
            return os.path.abspath(os.path.expanduser(configured_path))
        ros_home = os.environ.get("ROS_HOME", os.path.expanduser("~/.ros"))
        log_root = os.environ.get("ROS_LOG_DIR", os.path.join(ros_home, "log"))
        run_stamp = datetime.now().astimezone().strftime("%Y%m%dT%H%M%S%z")
        return os.path.join(log_root, "experiment_camera_frames", run_stamp)

    def _image_callback(self, message):
        with self._lock:
            self._latest = message

    def _save_latest(self, _event):
        with self._lock:
            message = self._latest
            if message is None or message.header.stamp == self._last_saved_stamp:
                return
            self._last_saved_stamp = message.header.stamp

        try:
            image = self._bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        except CvBridgeError as error:
            rospy.logerr_throttle(10.0, "Could not convert camera image: %s", error)
            return

        stamp = message.header.stamp
        if stamp.is_zero():
            stamp = rospy.Time.now()
        filename = "frame_{:010d}_{:09d}.jpg".format(stamp.secs, stamp.nsecs)
        path = os.path.join(self._output_dir, filename)
        if not cv2.imwrite(path, image, [cv2.IMWRITE_JPEG_QUALITY, self._jpeg_quality]):
            rospy.logerr("Failed to write experiment frame %s", path)


if __name__ == "__main__":
    rospy.init_node("experiment_camera_frame_recorder")
    try:
        FrameRecorder()
        rospy.spin()
    except (OSError, ValueError) as error:
        rospy.logfatal(str(error))
