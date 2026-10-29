#!/usr/bin/env python3
"""TopicRecorder's per-frame time sidecar, video id, command text in the strip on top and refused gripper topics. No ROS: python3 -m pytest test (cv2/rospy/cv_bridge stubbed)."""
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
    rospy = _stub("rospy", Subscriber=Mock(), loginfo=Mock(), logerr_throttle=Mock(), logerr=Mock(), logwarn=Mock())
    rospy.Time = _stub("Time", now=lambda: Stamp(clock["ros"]))
    def with_border(image, top, bottom, left, right, border_type, value=None):
        grown = FakeImage()
        grown.shape = (image.shape[0] + top + bottom, image.shape[1] + left + right, 3)
        grown.source, grown.border = image, (top, bottom, left, right, value)
        return grown

    cv2 = _stub("cv2", VideoWriter_fourcc=lambda *a: 0, CAP_GSTREAMER=1800, putText=Mock(), FONT_HERSHEY_SIMPLEX=0, LINE_AA=0,
                BORDER_CONSTANT=0, copyMakeBorder=with_border,
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
    os.makedirs(str(tmp_path), exist_ok=True)
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
    calls = recorder_module.cv2.VideoWriter.call_args_list
    assert len(calls) == 1
    assert 'location="%s"' % (tmp_path / (VID + "_experiment_camera_image_raw.mp4")) in calls[0].args[0]


def test_the_video_is_a_fragmented_mp4_that_survives_a_kill(recorder_module, tmp_path):
    recorder = _make(recorder_module, tmp_path)
    recorder.latest = Message(10.0)
    recorder.write_latest("REC")
    call = recorder_module.cv2.VideoWriter.call_args
    pipeline = call.args[0]
    assert call.args[1] == recorder_module.cv2.CAP_GSTREAMER
    assert "qtmux fragment-duration=1000" in pipeline and "x264enc" in pipeline
    assert "format=I420" in pipeline and "profile=main" in pipeline    # 4:2:0 main profile: every player decodes it
    assert "key-int-max=5" in pipeline                          # a key frame per second (fps 5) starts each fragment
    assert "bitrate=2000 " in pipeline                          # x264enc takes kbit/s


def test_without_x264enc_it_writes_vp8_and_warns(recorder_module, tmp_path):
    broken, vp8 = Mock(), Mock()
    broken.isOpened.return_value = False
    vp8.isOpened.return_value = True
    recorder_module.cv2.VideoWriter = Mock(side_effect=[broken, vp8])
    recorder = _make(recorder_module, tmp_path)
    recorder.latest = Message(10.0)
    recorder.write_latest("REC")
    assert recorder.writer is vp8
    pipeline = recorder_module.cv2.VideoWriter.call_args.args[0]
    assert "vp8enc" in pipeline and "keyframe-max-dist=5" in pipeline and "target-bitrate=2000000" in pipeline
    assert "VLC cannot play" in recorder_module.rospy.logwarn.call_args.args[0]


def test_without_gstreamer_it_falls_back_to_the_codec_and_warns(recorder_module, tmp_path):
    broken, plain = Mock(), Mock()
    broken.isOpened.return_value = False
    plain.isOpened.return_value = True
    recorder_module.cv2.VideoWriter = Mock(side_effect=[broken, broken, plain])
    recorder = _make(recorder_module, tmp_path)
    recorder.latest = Message(10.0)
    recorder.write_latest("REC")
    assert recorder.writer is plain
    assert recorder_module.cv2.VideoWriter.call_args.args[0] == str(tmp_path / (VID + "_experiment_camera_image_raw.mp4"))
    assert "unplayable" in recorder_module.rospy.logwarn.call_args.args[0]


def test_crash_safe_off_uses_the_codec_directly(recorder_module, tmp_path):
    recorder = recorder_module.TopicRecorder("/experiment_camera/image_raw", str(tmp_path), fps=5.0, codec="mp4v",
                                             overlay=False, video_id=VID, command="x", crash_safe=False)
    recorder.latest = Message(10.0)
    recorder.write_latest("REC")
    assert recorder_module.cv2.VideoWriter.call_args.args[0].endswith(".mp4")


def test_the_pipeline_quotes_the_path(recorder_module):
    pipeline = recorder_module.crash_safe_pipeline('/data/a "b"/v.mp4', 10.0, 2000)
    assert pipeline.endswith('location="/data/a \\"b\\"/v.mp4"')


def test_the_summary_is_written_atomically(recorder_module, tmp_path):
    path = str(tmp_path / "summary.json")
    recorder_module.write_json_atomically(path, {"topics": [], "stopped": False})
    assert json.load(open(path)) == {"topics": [], "stopped": False}
    assert os.listdir(str(tmp_path)) == ["summary.json"]           # no temporary file left behind


def test_the_summary_of_a_running_video_lists_its_frames(recorder_module, tmp_path):
    recorder = _make(recorder_module, tmp_path)
    recorder.latest = Message(10.0)
    recorder.write_latest("REC")
    summary = recorder.summary()
    assert summary["frames"] == 1 and summary["video"].endswith("_experiment_camera_image_raw.mp4")
    assert recorder.writer is not None                            # summary() does not close the video


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


def test_command_lines_wrap_by_the_drawn_width_and_cut(recorder_module):
    lines, width = recorder_module.command_lines, recorder_module.text_width
    assert lines("", 400, 0.5) == []
    assert lines("$ a  b\n c", 400, 0.5) == ["$ a b c"]
    long = " ".join("--option%d value" % i for i in range(40))
    wrapped = lines(long, 400, 0.5)                               # the stub draws 10 px per character at 0.5
    assert len(wrapped) == recorder_module.COMMAND_MAX_LINES and all(width(line, 0.5) <= 400 for line in wrapped)
    assert wrapped[-1].endswith("...")
    assert all(width(line, 0.5) > 400 - 170 for line in wrapped[:-1])     # filled up to the last word that fits
    assert lines("x" * 90, 400, 0.5) == ["x" * 40, "x" * 40, "x" * 10]     # a word wider than a line is split
    assert lines("aaaa bbbb", 1000, 0.5) == ["aaaa bbbb"]
    assert lines("aaaa bbbb", 1000, 2.5) == ["aaaa bbbb"] and lines("aaaa bbbb", 300, 2.5) == ["aaaa", "bbbb"]


def test_the_strip_text_is_small_and_scales_down_with_the_frame_width(recorder_module):
    layout = recorder_module.strip_layout
    assert layout(1280, 720, 1)[0] == recorder_module.TEXT_SCALE == 0.62
    assert layout(1920, 1080, 1)[0] == 0.62 and layout(640, 480, 1)[0] == recorder_module.TEXT_SCALE_MIN
    assert layout(1280, 720, 1)[2] == 1280 - 2 * recorder_module.STRIP_MARGIN


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


def test_overlay_shows_the_command_the_goal_and_the_model_and_never_a_silent_placeholder(recorder_module):
    parts = recorder_module.overlay_parts
    assert parts("python3 run.py") == ["$ python3 run.py"]
    assert parts("", "Pick the apple from table 1", "gemini-robotics-er-2-preview", "/recognized_speech") == [
        "goal (model gemini-robotics-er-2-preview): Pick the apple from table 1"]
    assert parts("rosrun x y.py", "Pick it", "", "/recognized_speech") == ["$ rosrun x y.py", "goal: Pick it"]
    assert parts("", "", "", "/recognized_speech") == ["NO COMMAND YET: waiting for the goal on /recognized_speech"]
    assert parts("  ", "") == ["NO COMMAND GIVEN (pass command:=...)"]
    assert not any("command not given" in p for p in parts("", "", "", "/recognized_speech"))


def test_the_model_profile_is_read_from_the_running_agent(recorder_module):
    params = {"/mobipick/mobipick_pi_robot_runner/experiment_model_label": "gemini-robotics-er-2-preview",
              "/other/experiment_model_label": "", "/rosdistro": "noetic"}
    find = recorder_module.find_model_label
    assert find(params, params.get, "experiment_model_label") == "gemini-robotics-er-2-preview"
    assert find(params, params.get, "") == ""
    assert find({"/rosdistro": "noetic"}, {"/rosdistro": "noetic"}.get, "experiment_model_label") == ""


def test_a_goal_arriving_later_replaces_the_waiting_line_in_the_next_frames(recorder_module, tmp_path):
    recorder = _make(recorder_module, tmp_path, command="")
    recorder.set_overlay(recorder_module.overlay_parts("", "", "", "/recognized_speech"))

    def texts(seconds):
        recorder_module.cv2.putText.reset_mock()
        recorder.latest = Message(seconds)
        recorder.write_latest("REC")
        return [call.args[1] for call in recorder_module.cv2.putText.call_args_list]

    assert "NO COMMAND YET: waiting for the goal on /recognized_speech" in texts(10.0)
    recorder.set_overlay(recorder_module.overlay_parts("", "Pick the apple", "deepseek-v4.1-flash", "/recognized_speech"))
    later = texts(10.5)
    assert "goal (model deepseek-v4.1-flash): Pick the apple" in later and VID in later
    assert not any("NO COMMAND" in t for t in later)


def _written(module):
    return module.cv2.VideoWriter.return_value.write.call_args.args[0]


def test_the_text_goes_into_a_black_strip_on_top_and_never_over_the_picture(recorder_module, tmp_path):
    recorder = _make(recorder_module, tmp_path)
    message = Message(10.0)
    recorder.latest = message
    recorder.write_latest("REC")
    frame = _written(recorder_module)
    strip = recorder.layout[3]
    assert frame.source is message.image                         # the camera picture goes in unchanged
    assert frame.border == (strip, 0, 0, 0, (0, 0, 0))           # black, on top only
    assert frame.shape[:2] == (480 + strip, 640) and frame.shape[0] % 2 == 0
    assert recorder_module.cv2.VideoWriter.call_args.args[4] == (640, 480 + strip)   # (pipeline, CAP_GSTREAMER, 0, fps, size)
    calls = recorder_module.cv2.putText.call_args_list
    assert len(calls) == 2                                       # the id line and the command, no outline pass
    for call in calls:
        assert call.args[0] is frame and call.args[5] == (0, 255, 0)      # lime, drawn on the grown frame
        assert call.args[2][1] <= strip                          # every baseline lies inside the strip


def test_the_strip_fits_the_text_lines_and_keeps_the_frame_height_even(recorder_module):
    layout = recorder_module.strip_layout
    heights = [layout(640, 480, rows)[3] for rows in (1, 2, 5)]
    assert heights[0] < heights[1] < heights[2]
    for picture_height in (480, 481, 720, 1079):
        for rows in range(1, 8):
            assert (layout(640, picture_height, rows)[3] + picture_height) % 2 == 0
    recorder_fit = recorder_module.fit_rows
    assert recorder_fit(["a", "b"], 3, 400, 0.5) == ["a", "b"]
    assert recorder_fit(["a", "b", "c"], 2, 400, 0.5) == ["a", "b..."]


def test_a_long_command_is_wrapped_and_makes_the_strip_taller(recorder_module, tmp_path):
    short = _make(recorder_module, tmp_path / "short")
    short.latest = Message(10.0)
    short.write_latest("REC")
    long = _make(recorder_module, tmp_path / "long", command=" ".join("--option%d value" % i for i in range(20)))
    long.latest = Message(10.0)
    long.write_latest("REC")
    assert long.rows > short.rows == 2 and long.layout[3] > short.layout[3]
    texts = [call.args[1] for call in recorder_module.cv2.putText.call_args_list[-long.rows:]]
    assert len(texts) == long.rows and all(recorder_module.text_width(t, long.layout[0]) <= long.layout[2] for t in texts[1:])


def test_rows_stay_free_for_a_goal_that_arrives_after_the_video_opened(recorder_module, tmp_path):
    recorder = _make(recorder_module, tmp_path, command="")
    recorder.set_overlay(recorder_module.overlay_parts("", "", "", "/recognized_speech"),
                         recorder_module.GOAL_SPARE_ROWS)
    recorder.latest = Message(10.0)
    recorder.write_latest("REC")
    size, rows = recorder.size, recorder.rows
    assert rows == 2 + recorder_module.GOAL_SPARE_ROWS
    recorder.set_overlay(recorder_module.overlay_parts("", " ".join(["word"] * 200), "m", "/recognized_speech"))
    recorder_module.cv2.putText.reset_mock()
    recorder.latest = Message(10.5)
    recorder.write_latest("REC")
    texts = [call.args[1] for call in recorder_module.cv2.putText.call_args_list]
    assert recorder.size == size and _written(recorder_module).shape[:2] == (size[1], size[0])   # same frame size
    assert len(texts) == rows and texts[-1].endswith("...")      # too long for the strip: cut, not drawn lower


@pytest.mark.parametrize("text, expected", [
    ("Baseline run 1: drill pick, sim", "baseline-run-1-drill-pick-sim"),
    ("  --Ünï/cödé!!  ", "n-c-d"),
    ("../../etc/passwd", "etc-passwd"),
    ("", ""),
    (None, ""),
    ("a" * 60, "a" * 40),
    ("word " * 10, "word-" * 7 + "word"),
])
def test_a_title_is_sanitised_for_the_file_name(recorder_module, text, expected):
    assert recorder_module.sanitize_title(text) == expected


def test_a_title_follows_the_id_in_the_file_name_and_the_id_index(recorder_module, tmp_path):
    index = tmp_path / "video_ids.tsv"
    recorder = recorder_module.TopicRecorder("/experiment_camera/image_raw", str(tmp_path), fps=5.0, codec="mp4v",
                                             overlay=False, video_id=VID, command="x", ids_path=str(index),
                                             title="Baseline run 1: drill pick, sim", title_only=True)
    assert recorder.video_path == str(tmp_path / (VID + "_baseline-run-1-drill-pick-sim.mp4"))
    assert recorder.frames_index_path.endswith(VID + "_baseline-run-1-drill-pick-sim_frames.jsonl")
    recorder.latest = Message(10.0)
    recorder.write_latest("REC")
    assert index.read_text().split("\t")[0] == VID


def test_with_several_topics_the_title_is_followed_by_the_topic_name(recorder_module, tmp_path):
    recorder = recorder_module.TopicRecorder("/experiment_camera/image_raw", str(tmp_path), fps=5.0, codec="mp4v",
                                             overlay=False, video_id=VID, command="x", title="drill pick")
    assert recorder.video_path == str(tmp_path / (VID + "_drill-pick_experiment_camera_image_raw.mp4"))


def test_without_a_title_the_file_name_is_unchanged(recorder_module, tmp_path):
    recorder = _make(recorder_module, tmp_path)
    assert recorder.video_path == str(tmp_path / (VID + "_experiment_camera_image_raw.mp4"))


def _ticking(module, monkeypatch, auto_pause=True, lead_in_s=3.0):
    """a VideoRecorder without ROS: only what _tick and _goal_callback use, the wall clock under the test's control"""
    clock = {"now": 1000.0}
    monkeypatch.setattr(module.time, "time", lambda: clock["now"])
    node = module.VideoRecorder.__new__(module.VideoRecorder)
    node.lock = module.threading.Lock()
    node.stopped = node.manual_paused = node.recording = False
    node.auto_pause, node.motion_hold_s, node.lead_in_s = auto_pause, 1.5, lead_in_s
    node.last_motion, node.lead_in_until = None, clock["now"] + lead_in_s
    node.summary_written, node.summary_period_s = clock["now"], 1e9
    node.recorders = [Mock()]
    node.goals, node.command_topic, node.model_label_param = [], "/recognized_speech", "experiment_model_label"
    node.events = []
    node._log_event = lambda kind, **data: node.events.append((kind, data.get("reason")))
    node._publish_status = node._write_summary = node._set_overlays = lambda *a, **k: None
    return node, clock


def _frames(node):
    return node.recorders[0].write_latest.call_count


def test_a_robot_that_never_moves_still_gives_the_lead_in_frames(recorder_module, monkeypatch):
    node, clock = _ticking(recorder_module, monkeypatch)
    for _ in range(100):          # 10 s at 10 Hz, the robot at rest
        node._tick(None)
        clock["now"] += 0.1
    assert 29 <= _frames(node) <= 31
    assert node.events == [("resume", "lead-in"), ("pause", "motion")]


def test_after_the_lead_in_only_motion_is_written(recorder_module, monkeypatch):
    node, clock = _ticking(recorder_module, monkeypatch)
    clock["now"] += 5.0
    node._tick(None)
    assert _frames(node) == 0
    node.last_motion = clock["now"]
    node._tick(None)
    assert _frames(node) == 1 and node.events[-1] == ("resume", "motion")


def test_a_goal_starts_another_lead_in(recorder_module, monkeypatch):
    node, clock = _ticking(recorder_module, monkeypatch)
    recorder_module.rospy.get_param_names = Mock(return_value=[])
    recorder_module.rospy.get_param = Mock()
    clock["now"] += 10.0
    node._tick(None)
    assert _frames(node) == 0
    node._goal_callback(types.SimpleNamespace(data="Pick the apple"))
    node._tick(None)
    clock["now"] += 3.5
    node._tick(None)
    assert _frames(node) == 1


def test_lead_in_zero_keeps_the_old_motion_only_behaviour(recorder_module, monkeypatch):
    node, clock = _ticking(recorder_module, monkeypatch, lead_in_s=0.0)
    node._tick(None)
    assert _frames(node) == 0 and node.events == []


def _subscribed_motion_topics(module, monkeypatch, tmp_path, params):
    """the cmd_vel and joint_states topics a VideoRecorder built with these private parameters subscribes to"""
    values = {"~image_topics": ["/experiment_camera/image_raw"], "~command": "run", "~output_dir": str(tmp_path)}
    values.update(params)
    module.rospy.get_param = lambda name, default=None: values.get(name, default)
    for name in ("Service", "Publisher", "Timer", "Duration"):
        setattr(module.rospy, name, Mock())
    module.rospy.Subscriber = Mock()
    monkeypatch.setattr(module, "TopicRecorder", Mock())
    monkeypatch.setattr(module.VideoRecorder, "_set_overlays", Mock())
    monkeypatch.setattr(module.VideoRecorder, "_write_summary", Mock())
    monkeypatch.setattr(module.VideoRecorder, "_log_event", Mock())
    module.VideoRecorder()
    topics = [call.args[0] for call in module.rospy.Subscriber.call_args_list]
    return [t for t in topics if t.endswith("cmd_vel")], [t for t in topics if t.endswith("joint_states")]


TWO_CAMERAS = {"~image_topics": "/experiment_camera/image_raw /experiment_camera2/image_raw",
               "~topic_robots": "mobipick mobipick2", "~title": "g382-run-1"}


def test_by_default_the_motion_of_the_first_robot_counts(recorder_module, monkeypatch, tmp_path):
    cmd_vel, joints = _subscribed_motion_topics(recorder_module, monkeypatch, tmp_path, {})
    assert (cmd_vel, joints) == (["/mobipick/cmd_vel"], ["/mobipick/joint_states"])


def test_every_robot_namespace_keeps_the_recording_going(recorder_module, monkeypatch, tmp_path):
    cmd_vel, joints = _subscribed_motion_topics(recorder_module, monkeypatch, tmp_path,
                                                {"~robot_namespaces": "mobipick /mobipick2"})
    assert cmd_vel == ["/mobipick/cmd_vel", "/mobipick2/cmd_vel"]
    assert joints == ["/mobipick/joint_states", "/mobipick2/joint_states"]


def test_explicit_motion_topics_override_the_namespaces(recorder_module, monkeypatch, tmp_path):
    cmd_vel, joints = _subscribed_motion_topics(recorder_module, monkeypatch, tmp_path,
                                                {"~robot_namespaces": "mobipick mobipick2",
                                                 "~cmd_vel_topic": "/a/cmd_vel"})
    assert cmd_vel == ["/a/cmd_vel"]
    assert joints == ["/mobipick/joint_states", "/mobipick2/joint_states"]


def test_one_video_per_robot_with_the_robot_first_in_its_title(recorder_module, monkeypatch, tmp_path):
    cmd_vel, joints = _subscribed_motion_topics(recorder_module, monkeypatch, tmp_path, TWO_CAMERAS)
    calls = recorder_module.TopicRecorder.call_args_list
    assert [call.args[0] for call in calls] == ["/experiment_camera/image_raw", "/experiment_camera2/image_raw"]
    assert [(call.args[10], call.args[11]) for call in calls] == [("mobipick-g382-run-1", True),
                                                                  ("mobipick2-g382-run-1", True)]
    assert len({call.args[5] for call in calls}) == 2            # each video its own id
    assert cmd_vel == ["/mobipick/cmd_vel", "/mobipick2/cmd_vel"]
    assert joints == ["/mobipick/joint_states", "/mobipick2/joint_states"]


def test_topic_robots_must_name_one_robot_per_camera(recorder_module, monkeypatch, tmp_path):
    with pytest.raises(ValueError, match="2 image topics"):
        _subscribed_motion_topics(recorder_module, monkeypatch, tmp_path, dict(TWO_CAMERAS, **{"~topic_robots": "x"}))


def test_each_robot_video_is_written_only_while_its_robot_moves(recorder_module, monkeypatch):
    node, clock = _ticking(recorder_module, monkeypatch)
    node.recorders = [Mock(), Mock()]
    node.topic_robots, node.robot_motion = ["mobipick", "mobipick2"], {}
    node.joint_pattern, node.joint_velocity_threshold = recorder_module.re.compile("ur5"), 0.02
    clock["now"] += 5.0                                          # after the lead-in
    node._joint_states_callback(types.SimpleNamespace(name=["mobipick2/ur5_wrist_3_joint"], velocity=[0.5]),
                                "mobipick2")
    node._tick(None)
    assert [r.write_latest.call_count for r in node.recorders] == [0, 1]
    node.recorders[1].write_latest.assert_called_with("REC")
    node._cmd_vel_callback(types.SimpleNamespace(linear=types.SimpleNamespace(x=0.2, y=0.0),
                                                 angular=types.SimpleNamespace(z=0.0)), "mobipick")
    node._tick(None)
    assert [r.write_latest.call_count for r in node.recorders] == [1, 2]


def test_in_the_lead_in_every_robot_video_is_written(recorder_module, monkeypatch):
    node, clock = _ticking(recorder_module, monkeypatch)
    node.recorders = [Mock(), Mock()]
    node.topic_robots, node.robot_motion = ["mobipick", "mobipick2"], {}
    node._tick(None)
    assert [r.write_latest.call_args.args for r in node.recorders] == [("REC idle",), ("REC idle",)]
