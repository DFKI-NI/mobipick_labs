#!/usr/bin/env python3

"""Tune the experiment camera pose in Gazebo and generate a launch file."""

import os
import re
import sys
import xml.etree.ElementTree as ElementTree

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
    QDoubleSpinBox,
    QFormLayout,
    QHBoxLayout,
    QLabel,
    QMessageBox,
    QPushButton,
    QSlider,
    QVBoxLayout,
)
from sensor_msgs.msg import Image
from tf.transformations import euler_from_quaternion, quaternion_from_euler

AXES = ("x", "y", "z", "roll", "pitch", "yaw")
WORLD_CONDITION = re.compile(r"world_config\s*==\s*'([^']+)'")


def read_world_poses(launch_file):
    """Return {world_config: {axis: value}} from a configured_camera.launch file."""
    poses = {}
    try:
        root = ElementTree.parse(launch_file).getroot()
    except (OSError, ElementTree.ParseError):
        return poses
    for include in root.iter("include"):
        match = WORLD_CONDITION.search(include.get("if", ""))
        if not match:
            continue
        values = {arg.get("name"): arg.get("value") for arg in include.findall("arg")}
        try:
            poses[match.group(1)] = {axis: float(values[axis]) for axis in AXES}
        except (KeyError, TypeError, ValueError):
            continue
    return poses


def launch_file_code(poses):
    """Generate configured_camera.launch with one camera include per world."""
    blocks = []
    for world, pose in poses.items():
        blocks.append("""  <include if=\"$(eval world_config == '{world}')\" file=\"$(find experiment_camera_recorder)/launch/camera.launch\">
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
""".format(world=world, **pose))
    worlds = ", ".join("'{}'".format(world) for world in poses)
    return """<?xml version=\"1.0\"?>
<!-- One camera pose per world. pose_tuner.py rewrites the pose of the world it tunes when its OK button is clicked. -->
<launch>
  <arg name=\"world_config\" default=\"moelk_tables\" />
  <arg name=\"record\" default=\"false\" />
  <arg name=\"record_frequency\" default=\"0.2\" />
  <arg name=\"output_dir\" default=\"\" />

{blocks}
  <!-- worlds without a tuned pose get the camera.launch default pose -->
  <include unless=\"$(eval world_config in [{worlds}])\" file=\"$(find experiment_camera_recorder)/launch/camera.launch\">
    <arg name=\"record\" value=\"$(arg record)\" />
    <arg name=\"record_frequency\" value=\"$(arg record_frequency)\" />
    <arg name=\"output_dir\" value=\"$(arg output_dir)\" />
  </include>
</launch>
""".format(blocks="\n".join(blocks), worlds=worlds)


class PoseTuner(QDialog):
    image_received = Signal(object)

    RANGES = {
        "x": (-50.0, 50.0, 0.01),
        "y": (-50.0, 50.0, 0.01),
        "z": (0.0, 10.0, 0.01),
        "roll": (-3.1416, 3.1416, 0.001),
        "pitch": (-3.1416, 3.1416, 0.001),
        "yaw": (-3.1416, 3.1416, 0.001),
    }

    def __init__(self):
        super().__init__()
        self._world = rospy.get_param("~world_config", "moelk_tables")
        self.setWindowTitle("Experiment camera pose tuner ({})".format(self._world))
        self.resize(900, 760)
        self._bridge = CvBridge()
        self._model_name = rospy.get_param("~model_name", "experiment_camera")
        self._output_file = os.path.abspath(os.path.expanduser(
            rospy.get_param("~output_file")
        ))
        self._set_model_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
        self._sliders = {}
        self._spin_boxes = {}

        layout = QVBoxLayout(self)
        self._image_label = QLabel("Waiting for /experiment_camera/image_raw ...")
        self._image_label.setAlignment(Qt.AlignCenter)
        self._image_label.setMinimumSize(640, 360)
        layout.addWidget(self._image_label, 1)

        form = QFormLayout()
        for name, (minimum, maximum, step) in self.RANGES.items():
            slider = QSlider(Qt.Horizontal)
            slider.setRange(round(minimum / step), round(maximum / step))
            spin_box = QDoubleSpinBox()
            spin_box.setRange(minimum, maximum)
            spin_box.setSingleStep(step)
            spin_box.setDecimals(3 if step >= 0.01 else 4)
            spin_box.setMinimumWidth(100)
            row = QHBoxLayout()
            row.addWidget(slider, 1)
            row.addWidget(spin_box)
            form.addRow(name, row)
            slider.valueChanged.connect(lambda value, name=name: self._slider_changed(name, value))
            spin_box.valueChanged.connect(lambda value, name=name: self._spin_box_changed(name, value))
            self._sliders[name] = slider
            self._spin_boxes[name] = spin_box
        layout.addLayout(form)

        buttons = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        saved_pose_button = QPushButton("Saved pose")
        saved_pose_button.setToolTip("Move the camera to the pose saved for {}".format(self._world))
        saved_pose_button.clicked.connect(self._load_saved_pose)
        buttons.addButton(saved_pose_button, QDialogButtonBox.ResetRole)
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
            self._set_values(dict(zip(
                AXES,
                (response.pose.position.x, response.pose.position.y, response.pose.position.z)
                + euler_from_quaternion((orientation.x, orientation.y, orientation.z, orientation.w)),
            )))
        except (rospy.ServiceException, RuntimeError) as error:
            QMessageBox.warning(self, "Gazebo model unavailable", str(error))

    def _load_saved_pose(self):
        pose = read_world_poses(self._output_file).get(self._world)
        if pose is None:
            QMessageBox.information(self, "No saved pose", "{} has no pose for {}".format(self._output_file, self._world))
            return
        self._set_values(pose)

    def _set_values(self, values):
        for name, value in values.items():
            step = self.RANGES[name][2]
            for widget, widget_value in ((self._sliders[name], round(value / step)), (self._spin_boxes[name], value)):
                widget.blockSignals(True)
                widget.setValue(widget_value)
                widget.blockSignals(False)
        self._pose_changed()

    def _slider_changed(self, name, value):
        self._spin_boxes[name].blockSignals(True)
        self._spin_boxes[name].setValue(value * self.RANGES[name][2])
        self._spin_boxes[name].blockSignals(False)
        self._pose_changed()

    def _spin_box_changed(self, name, value):
        self._sliders[name].blockSignals(True)
        self._sliders[name].setValue(round(value / self.RANGES[name][2]))
        self._sliders[name].blockSignals(False)
        self._pose_changed()

    def _values(self):
        return {name: spin_box.value() for name, spin_box in self._spin_boxes.items()}

    def _pose_changed(self):
        values = self._values()
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
        poses = read_world_poses(self._output_file)
        poses[self._world] = self._values()
        code = launch_file_code(poses)
        try:
            output_dir = os.path.dirname(self._output_file)
            if output_dir:
                os.makedirs(output_dir, exist_ok=True)
            with open(self._output_file, "w") as output:
                output.write(code)
        except OSError as error:
            QMessageBox.critical(self, "Could not write launch file", str(error))
            return
        QMessageBox.information(self, "Camera pose saved", "Saved the {} pose in {}".format(self._world, self._output_file))
        self.accept()


if __name__ == "__main__":
    rospy.init_node("experiment_camera_pose_tuner")
    application = QApplication(sys.argv)
    dialog = PoseTuner()
    dialog.show()
    sys.exit(application.exec_())
