#!/usr/bin/env python3
"""TopicRecorder's per-frame time sidecar (#165). No ROS: python3 -m pytest test (cv2/rospy/cv_bridge stubbed)."""
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


def _make(module, tmp_path, topic="/experiment_camera/image_raw"):
    return module.TopicRecorder(topic, str(tmp_path), fps=5.0, speedup=4, codec="mp4v", overlay=False)


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
    assert summary["frames_index"] == str(tmp_path / "experiment_camera_image_raw_frames.jsonl")
    assert summary["video"] == str(tmp_path / "experiment_camera_image_raw.mp4")
    assert recorder._frames_index is None                         # file closed with the videos
