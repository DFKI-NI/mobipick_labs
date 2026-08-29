#!/usr/bin/env python3

"""Tune the experiment camera pose in Gazebo and generate a launch file."""

import os
import sys

import rospy
from cv_bridge import CvBridge, CvBridgeError
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from python_qt_binding.QtCore import Qt, Signal
from python_qt_binding.QtGui import QImage, QPixmap
from python_qt_binding.QtWidgets import (
    QApplication,
    QDialog,
    QDialogButtonBox,
    QFormLayout,
    QLabel,
    QMessageBox,
    QSlider,
    QVBoxLayout,
)
from sensor_msgs.msg import Image
from tf.transformations import euler_from_quaternion, quaternion_from_euler


class PoseTuner(QDialog):
    image_received = Signal(object)

    RANGES = {
        "x": (-20.0, 20.0, 0.01),
        "y": (-20.0, 20.0, 0.01),
        "z": (0.0, 10.0, 0.01),
        "roll": (-3.1416, 3.1416, 0.001),
        "pitch": (-3.1416, 3.1416, 0.001),
        "yaw": (-3.1416, 3.1416, 0.001),
    }

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Experiment camera pose tuner")
        self.resize(900, 760)
        self._bridge = CvBridge()
        self._model_name = rospy.get_param("~model_name", "experiment_camera")
        self._output_file = os.path.abspath(os.path.expanduser(
            rospy.get_param("~output_file")
        ))
        self._set_model_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
        self._sliders = {}
        self._value_labels = {}

        layout = QVBoxLayout(self)
        self._image_label = QLabel("Waiting for /experiment_camera/image_raw ...")
        self._image_label.setAlignment(Qt.AlignCenter)
        self._image_label.setMinimumSize(640, 360)
        layout.addWidget(self._image_label, 1)

        form = QFormLayout()
        for name, (minimum, maximum, step) in self.RANGES.items():
            slider = QSlider(Qt.Horizontal)
            slider.setRange(round(minimum / step), round(maximum / step))
            value_label = QLabel("0.000")
            value_label.setMinimumWidth(70)
            row = QVBoxLayout()
            row.addWidget(slider)
            row.addWidget(value_label)
            form.addRow(name, row)
            slider.valueChanged.connect(self._pose_changed)
            self._sliders[name] = slider
            self._value_labels[name] = value_label
        layout.addLayout(form)

        buttons = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        buttons.accepted.connect(self._write_launch_file)
        buttons.rejected.connect(self.reject)
        layout.addWidget(buttons)

        self.image_received.connect(self._show_image)
        topic = rospy.get_param("~image_topic", "/experiment_camera/image_raw")
        self._subscriber = rospy.Subscriber(topic, Image, self._image_callback, queue_size=1)
        self._load_current_pose()

    def _load_current_pose(self):
        try:
            from gazebo_msgs.srv import GetModelState
            get_state = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
            response = get_state(self._model_name, "world")
            if not response.success:
                raise RuntimeError(response.status_message)
            orientation = response.pose.orientation
            values = dict(zip(
                ("x", "y", "z", "roll", "pitch", "yaw"),
                (response.pose.position.x, response.pose.position.y, response.pose.position.z)
                + euler_from_quaternion((orientation.x, orientation.y, orientation.z, orientation.w)),
            ))
            for name, value in values.items():
                step = self.RANGES[name][2]
                self._sliders[name].blockSignals(True)
                self._sliders[name].setValue(round(value / step))
                self._sliders[name].blockSignals(False)
            self._pose_changed()
        except (rospy.ServiceException, RuntimeError) as error:
            QMessageBox.warning(self, "Gazebo model unavailable", str(error))

    def _values(self):
        return {
            name: slider.value() * self.RANGES[name][2]
            for name, slider in self._sliders.items()
        }

    def _pose_changed(self):
        values = self._values()
        for name, label in self._value_labels.items():
            label.setText("{:.3f}".format(values[name]))
        quaternion = quaternion_from_euler(values["roll"], values["pitch"], values["yaw"])
        state = ModelState()
        state.model_name = self._model_name
        state.reference_frame = "world"
        state.pose.position.x = values["x"]
        state.pose.position.y = values["y"]
        state.pose.position.z = values["z"]
        state.pose.orientation.x, state.pose.orientation.y, state.pose.orientation.z, state.pose.orientation.w = quaternion
        try:
            response = self._set_model_state(state)
            if not response.success:
                rospy.logwarn_throttle(2.0, "Could not move experiment camera: %s", response.status_message)
        except rospy.ServiceException as error:
            rospy.logwarn_throttle(2.0, "Could not move experiment camera: %s", error)

    def _image_callback(self, message):
        try:
            rgb = self._bridge.imgmsg_to_cv2(message, desired_encoding="rgb8")
            height, width, channels = rgb.shape
            image = QImage(rgb.data, width, height, channels * width, QImage.Format_RGB888).copy()
            self.image_received.emit(image)
        except CvBridgeError as error:
            rospy.logwarn_throttle(5.0, "Could not display camera image: %s", error)

    def _show_image(self, image):
        pixmap = QPixmap.fromImage(image)
        self._image_label.setPixmap(pixmap.scaled(
            self._image_label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation
        ))

    def _write_launch_file(self):
        values = self._values()
        code = """<?xml version=\"1.0\"?>
<!-- This file is updated by pose_tuner.py when its OK button is clicked. -->
<launch>
  <arg name=\"record\" default=\"false\" />
  <arg name=\"record_frequency\" default=\"0.2\" />
  <arg name=\"output_dir\" default=\"\" />

  <include file=\"$(find experiment_camera_recorder)/launch/camera.launch\">
    <arg name=\"x\" value=\"{x:.3f}\" />
    <arg name=\"y\" value=\"{y:.3f}\" />
    <arg name=\"z\" value=\"{z:.3f}\" />
    <arg name=\"roll\" value=\"{roll:.4f}\" />
    <arg name=\"pitch\" value=\"{pitch:.4f}\" />
    <arg name=\"yaw\" value=\"{yaw:.4f}\" />
    <arg name=\"record\" value=\"$(arg record)\" />
    <arg name=\"record_frequency\" value=\"$(arg record_frequency)\" />
    <arg name=\"output_dir\" value=\"$(arg output_dir)\" />
  </include>
</launch>
""".format(**values)
        try:
            output_dir = os.path.dirname(self._output_file)
            if output_dir:
                os.makedirs(output_dir, exist_ok=True)
            with open(self._output_file, "w") as output:
                output.write(code)
        except OSError as error:
            QMessageBox.critical(self, "Could not write launch file", str(error))
            return
        QMessageBox.information(self, "Camera pose saved", "Generated {}".format(self._output_file))
        self.accept()


if __name__ == "__main__":
    rospy.init_node("experiment_camera_pose_tuner")
    application = QApplication(sys.argv)
    dialog = PoseTuner()
    dialog.show()
    sys.exit(application.exec_())
