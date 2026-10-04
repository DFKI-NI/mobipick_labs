#!/usr/bin/env python3
"""TopicRecorder's per-frame time sidecar (#165), video id, command text and refused gripper topics (#349). No ROS: python3 -m pytest test (cv2/rospy/cv_bridge stubbed)."""
import importlib.util
import json
import os
import sys
import types
from unittest.mock import Mock

import pytest


class Stamp:
    def __init__(self, seconds):
        self.seconds = seconds

    def to_sec(self):
        return self.seconds

    def is_zero(self):
        return self.seconds == 0

    def __eq__(self, other):
        return isinstance(other, Stamp) and other.seconds == self.seconds


def _stub(name, **attributes):
    module = types.ModuleType(name)
    for key, value in attributes.items():
        setattr(module, key, value)
    return module


@pytest.fixture
def recorder_module(monkeypatch):
    """scripts/video_recorder.py loaded with cv2, rospy and the ROS messages replaced by small stubs"""
    clock = {"ros": 100.0}
    rospy = _stub("rospy", Subscriber=Mock(), loginfo=Mock(), logerr_throttle=Mock(), logerr=Mock())
    rospy.Time = _stub("Time", now=lambda: Stamp(clock["ros"]))
    cv2 = _stub("cv2", VideoWriter_fourcc=lambda *a: 0, putText=Mock(), FONT_HERSHEY_SIMPLEX=0, LINE_AA=0,
                getTextSize=lambda text, font, scale, thickness: ((int(len(text) * 20 * scale), int(22 * scale)), 5),
                resize=lambda image, size: image, imwrite=Mock(return_value=True))
    writer = Mock()
    writer.isOpened.return_value = True
    cv2.VideoWriter = Mock(return_value=writer)
    bridge = Mock()
    bridge.imgmsg_to_cv2 = lambda message, desired_encoding: message.image
    monkeypatch.setitem(sys.modules, "cv2", cv2)
    monkeypatch.setitem(sys.modules, "rospy", rospy)
    monkeypatch.setitem(sys.modules, "cv_bridge", _stub("cv_bridge", CvBridge=Mock(return_value=bridge),
                                                        CvBridgeError=RuntimeError))
    monkeypatch.setitem(sys.modules, "geometry_msgs", _stub("geometry_msgs"))
    monkeypatch.setitem(sys.modules, "geometry_msgs.msg", _stub("geometry_msgs.msg", Twist=object))
    monkeypatch.setitem(sys.modules, "sensor_msgs", _stub("sensor_msgs"))
    monkeypatch.setitem(sys.modules, "sensor_msgs.msg", _stub("sensor_msgs.msg", Image=object, JointState=object))
    monkeypatch.setitem(sys.modules, "std_msgs", _stub("std_msgs"))
    monkeypatch.setitem(sys.modules, "std_msgs.msg", _stub("std_msgs.msg", String=object))
    monkeypatch.setitem(sys.modules, "std_srvs", _stub("std_srvs"))
    monkeypatch.setitem(sys.modules, "std_srvs.srv", _stub("std_srvs.srv", Trigger=object, TriggerResponse=object))
    path = os.path.join(os.path.dirname(__file__), "..", "scripts", "video_recorder.py")
    spec = importlib.util.spec_from_file_location("video_recorder_under_test", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    module.clock = clock
    return module


class Message:
    def __init__(self, seconds):
        self.header = types.SimpleNamespace(stamp=Stamp(seconds))
        self.image = FakeImage()


class FakeImage:
    shape = (480, 640, 3)


def _lines(path):
    with open(path) as handle:
        return [json.loads(line) for line in handle]


VID = "v20261004-115230-3fa9"


def _make(module, tmp_path, topic="/experiment_camera/image_raw", command="python3 run.py --n 1", ids_path=None):
    return module.TopicRecorder(topic, str(tmp_path), fps=5.0, codec="mp4v", overlay=False, video_id=VID,
                                command=command, ids_path=ids_path)


def test_every_written_frame_gets_one_line_with_its_times(recorder_module, tmp_path):
    recorder = _make(recorder_module, tmp_path)
    recorder.latest = Message(10.0)
    recorder_module.clock["ros"] = 10.2
    assert recorder.write_latest("REC")
    recorder.latest = Message(10.5)
    recorder_module.clock["ros"] = 10.7
    assert recorder.write_latest("REC idle")
    lines = _lines(recorder.frames_index_path)
    assert [line["frame"] for line in lines] == [0, 1]
    assert [line["source_stamp"] for line in lines] == [10.0, 10.5]
    assert [line["ros_time"] for line in lines] == [10.2, 10.7]
    assert [line["label"] for line in lines] == ["REC", "REC idle"]
    assert all(abs(line["wall_time"] - __import__("time").time()) < 60 for line in lines)


def test_skipped_and_duplicate_images_take_no_index(recorder_module, tmp_path):
    recorder = _make(recorder_module, tmp_path)
    assert not recorder.write_latest("REC")                      # no image yet: nothing written
    recorder.latest = Message(10.0)
    assert recorder.write_latest("REC")
    assert not recorder.write_latest("REC")                      # same stamp again: skipped
    broken = Message(10.5)
    broken.image = None                                          # conversion fails: skipped too
    recorder.latest = broken
    assert not recorder.write_latest("REC")
    recorder.latest = Message(11.0)
    assert recorder.write_latest("REC")
    lines = _lines(recorder.frames_index_path)
    assert [(line["frame"], line["source_stamp"]) for line in lines] == [(0, 10.0), (1, 11.0)]
    assert recorder.frames == 2


def test_a_pause_keeps_the_indices_continuous_and_shows_in_the_times(recorder_module, tmp_path):
    recorder = _make(recorder_module, tmp_path)
    recorder.latest = Message(10.0)
    recorder_module.clock["ros"] = 10.0
    recorder.write_latest("REC")
    # paused: the recorder loop does not call write_latest for 30 s while images keep arriving
    recorder.latest = Message(40.0)
    recorder_module.clock["ros"] = 40.1
    recorder.write_latest("REC")
    lines = _lines(recorder.frames_index_path)
    assert [line["frame"] for line in lines] == [0, 1]
    assert lines[1]["source_stamp"] - lines[0]["source_stamp"] == 30.0
    assert lines[1]["ros_time"] - lines[0]["ros_time"] == pytest.approx(30.1)


def test_summary_links_the_sidecar_only_when_frames_were_written(recorder_module, tmp_path):
    empty = _make(recorder_module, tmp_path, "/empty/image_raw")
    assert empty.close()["frames_index"] is None
    recorder = _make(recorder_module, tmp_path)
    recorder.latest = Message(10.0)
    recorder.write_latest("REC")
    summary = recorder.close()
    assert summary["frames"] == 1
    assert summary["frames_index"] == str(tmp_path / (VID + "_experiment_camera_image_raw_frames.jsonl"))
    assert summary["video"] == str(tmp_path / (VID + "_experiment_camera_image_raw.mp4"))
    assert summary["id"] == VID and "video_fast" not in summary
    assert recorder._frames_index is None                         # file closed with the videos


def test_one_video_writer_and_no_sped_up_copy(recorder_module, tmp_path):
    recorder = _make(recorder_module, tmp_path)
    recorder.latest = Message(10.0)
    recorder.write_latest("REC")
    paths = [call.args[0] for call in recorder_module.cv2.VideoWriter.call_args_list]
    assert paths == [str(tmp_path / (VID + "_experiment_camera_image_raw.mp4"))]


def test_every_frame_shows_the_id_and_the_command(recorder_module, tmp_path):
    recorder = _make(recorder_module, tmp_path, command="python3 0289_b_series.sh relay_1 3")
    for seconds in (10.0, 10.5):
        recorder_module.cv2.putText.reset_mock()
        recorder.latest = Message(seconds)
        recorder.write_latest("REC")
        texts = [call.args[1] for call in recorder_module.cv2.putText.call_args_list]
        assert VID in texts and "$ python3 0289_b_series.sh relay_1 3" in texts


def test_video_ids_are_unique_and_well_formed(recorder_module):
    from datetime import datetime
    started = datetime(2026, 10, 4, 11, 52, 30)
    ids = {recorder_module.video_id(started) for _ in range(50)}
    assert all(__import__("re").fullmatch(r"v20261004-115230-[0-9a-f]{4}", i) for i in ids)
    assert len(ids) > 40                                         # random 4 hex: collisions are rare
    assert recorder_module.video_id(started, "beef") == "v20261004-115230-beef"


def test_command_lines_wrap_cut_and_say_when_missing(recorder_module):
    assert recorder_module.command_lines("", 40) == ["$ command not given"]
    assert recorder_module.command_lines("a  b\n c", 40) == ["$ a b c"]
    long = " ".join("--option%d value" % i for i in range(40))
    lines = recorder_module.command_lines(long, 40)
    assert len(lines) == recorder_module.COMMAND_MAX_LINES and all(len(line) <= 40 for line in lines)
    assert lines[-1].endswith("...")


def test_gripper_and_robot_camera_topics_are_refused(recorder_module):
    pattern = "gripper|eef|wrist|hand_cam|arm_cam"
    allowed, refused = recorder_module.split_forbidden(
        ["/experiment_camera/image_raw", "/mobipick/eef_main_cam/rgb/image_raw", "/mobipick/gripper_cam/image"], pattern)
    assert allowed == ["/experiment_camera/image_raw"]
    assert refused == ["/mobipick/eef_main_cam/rgb/image_raw", "/mobipick/gripper_cam/image"]
    assert recorder_module.split_forbidden(["/mobipick/eef_main_cam/rgb/image_raw"], "")[1] == []


def test_a_new_video_is_appended_to_the_id_index(recorder_module, tmp_path):
    index = tmp_path / "video_ids.tsv"
    index.write_text("v20261001-000000-0000\t/old.mp4\n")
    recorder = _make(recorder_module, tmp_path, ids_path=str(index))
    recorder.latest = Message(10.0)
    recorder.write_latest("REC")
    recorder.latest = Message(10.5)
    recorder.write_latest("REC")                                 # one line per video, not per frame
    assert index.read_text().splitlines() == ["v20261001-000000-0000\t/old.mp4",
                                              VID + "\t" + str(tmp_path / (VID + "_experiment_camera_image_raw.mp4"))]
