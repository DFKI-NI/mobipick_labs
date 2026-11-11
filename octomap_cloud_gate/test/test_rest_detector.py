"""RestDetector of octomap_cloud_gate.py on synthetic joint states (50 Hz) and cloud stamps (no recorded pair exists yet:
the tester's lag check records one). Run: python3 -m unittest discover test (needs rospy and sensor_msgs: a ROS shell)."""
import importlib.util
import unittest
from pathlib import Path

SCRIPT = Path(__file__).resolve().parent.parent / 'nodes' / 'octomap_cloud_gate.py'
spec = importlib.util.spec_from_file_location('octomap_cloud_gate', SCRIPT)
mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mod)

NAMES = ['mobipick/' + n for n in mod.ARM_JOINTS] + ['mobipick/gripper_finger_joint']


def drive(det, t0, t1, speed, hz=50.0, start=0.0, velocities=False):
    '''joint states from t0 to t1 with wrist_1 moving at speed rad/s (the gripper joint is never at rest).'''
    t = t0
    while t <= t1 + 1e-9:
        pos = [0.0] * 6 + [t * 3.0]
        pos[3] = start + speed * (t - t0)
        vel = [0.0] * 3 + [speed] + [0.0] * 2 + [3.0] if velocities else ()
        det.update(t, NAMES, pos, vel)
        t += 1.0 / hz
    return start + speed * (t1 - t0)



class RestDetectorTests(unittest.TestCase):
    def test_at_rest_after_rest_time(self):
        det = mod.RestDetector()
        drive(det, 0.0, 2.0, 0.0)
        self.assertTrue(det.at_rest(cloud_stamp=1.9, now=2.0))

    def test_moving_arm_drops_the_cloud(self):
        det = mod.RestDetector()
        drive(det, 0.0, 2.0, 0.5)
        self.assertFalse(det.at_rest(2.0, 2.0))

    def test_cloud_taken_before_the_arm_stopped_is_dropped_even_when_it_arrives_late(self):
        det = mod.RestDetector()
        end = drive(det, 0.0, 1.0, 0.5)
        drive(det, 1.0 + 0.02, 3.0, 0.0, start=end)
        # taken while moving (stamp 0.9), arrives at 3.0: the arm has been still for 2 s but the cloud shows it moving
        self.assertFalse(det.at_rest(0.9, 3.0))
        # taken 0.1 s after it stopped: too soon (the render lags the stamp); 0.5 s after: kept
        self.assertFalse(det.at_rest(1.1, 3.0))
        self.assertTrue(det.at_rest(1.5, 3.0))

    def test_rest_time_after_a_motion_starts_again(self):
        det = mod.RestDetector()
        drive(det, 0.0, 1.0, 0.0)
        end = drive(det, 1.0, 1.5, 0.4)   # the last motion is at 1.5
        drive(det, 1.52, 2.0, 0.0, start=end)
        self.assertFalse(det.at_rest(1.7, 2.0))   # taken 0.2 s after the stop
        self.assertTrue(det.at_rest(1.9, 2.0))    # taken 0.4 s after
        self.assertFalse(det.at_rest(1.9, 1.7))   # arrives while still inside rest_time

    def test_threshold(self):
        slow = mod.RestDetector()
        drive(slow, 0.0, 2.0, 0.03)
        self.assertTrue(slow.at_rest(1.9, 2.0))
        fast = mod.RestDetector()
        drive(fast, 0.0, 2.0, 0.08)
        self.assertFalse(fast.at_rest(1.9, 2.0))

    def test_velocity_field_is_used_when_present(self):
        det = mod.RestDetector()
        drive(det, 0.0, 2.0, 0.5, velocities=True)
        self.assertFalse(det.at_rest(2.0, 2.0))
        det = mod.RestDetector()
        drive(det, 0.0, 2.0, 0.0, velocities=True)
        self.assertTrue(det.at_rest(1.9, 2.0))

    def test_no_or_stale_joint_states_close_the_gate(self):
        det = mod.RestDetector()
        self.assertFalse(det.at_rest(1.0, 1.0))
        drive(det, 0.0, 1.0, 0.0)
        self.assertTrue(det.at_rest(0.9, 1.0))
        self.assertFalse(det.at_rest(0.9, 2.5))   # no state for 1.5 s

    def test_only_the_arm_joints_count(self):
        det = mod.RestDetector()
        drive(det, 0.0, 2.0, 0.0)   # the gripper joint moves at 3 rad/s all the time
        self.assertTrue(det.at_rest(1.9, 2.0))

    def test_time_jump_back_restarts(self):
        det = mod.RestDetector()
        drive(det, 100.0, 101.0, 0.5)
        drive(det, 0.0, 1.0, 0.0)   # sim reset
        self.assertTrue(det.at_rest(0.9, 1.0))


class LagSummaryTests(unittest.TestCase):
    def test_median_and_max_in_ms(self):
        self.assertEqual(mod.lag_summary([0.030, 0.010, 0.020]), 'lag median 20 ms, max 30 ms')

    def test_no_clouds(self):
        self.assertEqual(mod.lag_summary([]), 'no clouds')


if __name__ == '__main__':
    unittest.main()
