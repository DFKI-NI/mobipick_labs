"""Every pose selector and perceive log line of the panel names the robot it acts for, and the pose selector lines name
the selector the call really reached, so a log of two robots shows which robot's selector each call hit.

The panel's own methods run on a stand-in object (no ROS master or rqt); run it in the ROS image."""
import os
import sys
import types
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))
from rqt_tables_demo import rqt_tables_demo as panel_module  # noqa: E402

Panel = panel_module.RqtTablesDemo
SELECTOR = '/mobipick2/pick_pose_selector_node'


class Proxy:
    def __init__(self, name, answer=None):
        self.resolved_name = name
        self.answer = answer

    def __call__(self, *args):
        return self.answer


class FakePanel:
    def __init__(self):
        self.robot_ns = 'mobipick2'
        self._robot_api = types.SimpleNamespace(ns='mobipick')  # robot 1 was built after robot 2's command started
        self.is_pose_selector_available = True
        empty = types.SimpleNamespace(poses=types.SimpleNamespace(objects=[]))
        self.activate_pose_selector_srv = Proxy(SELECTOR + '/pose_selector_activate', 'success: True')
        self.pose_selector_clear_srv = Proxy(SELECTOR + '/pose_selector_clear')
        self.pose_selector_get_all_poses_srv = Proxy(SELECTOR + '/pose_selector_get_all', empty)
        self.pose_selector_class_query_srv = Proxy(SELECTOR + '/pose_selector_class_query')

    def _run(self, group, what, task):  # in the test thread
        task({'ns': self.robot_ns, 'robot': types.SimpleNamespace(arm=types.SimpleNamespace(move=lambda pose: None)),
              'activate': self.activate_pose_selector_srv})

    def _api_still_targets(self, ctx, what):
        return True


for _name in ('_perceive_objs_task', '_pose_selector_call', 'perception_ps_activate',
              'perception_ps_deactivate', 'perception_ps_clear', 'perception_ps_get_all_objs'):
    setattr(FakePanel, _name, getattr(Panel, _name))


class LogNamesRobotTest(unittest.TestCase):
    def setUp(self):
        self.lines = []
        for level in ('loginfo', 'logwarn', 'logerr'):
            real = getattr(panel_module.rospy, level)
            setattr(panel_module.rospy, level, lambda message, *args: self.lines.append(str(message)))
            self.addCleanup(setattr, panel_module.rospy, level, real)
        self.panel = FakePanel()
        panel_module.rospy.sleep, real_sleep = (lambda seconds: None), panel_module.rospy.sleep
        self.addCleanup(setattr, panel_module.rospy, 'sleep', real_sleep)

    def test_perceive_lines_name_the_robot_the_selector_and_the_api_robot(self):
        self.panel._perceive_objs_task({'ns': 'mobipick2', 'activate': self.panel.activate_pose_selector_srv,
                                        'robot': types.SimpleNamespace(arm=types.SimpleNamespace(move=lambda p: None))},
                                       ['observe100cm_right'])
        self.assertTrue(self.lines)
        for line in self.lines:
            self.assertIn('mobipick2', line)
        self.assertTrue(any('robot API serves mobipick' in line and line.endswith('mobipick)') for line in self.lines))
        self.assertTrue(any(SELECTOR + '/pose_selector_activate' in line and 'deactivating' in line
                            for line in self.lines))

    def test_pose_selector_buttons_name_the_robot_and_the_selector(self):
        for button in (self.panel.perception_ps_activate, self.panel.perception_ps_deactivate,
                       self.panel.perception_ps_clear, self.panel.perception_ps_get_all_objs):
            button()
        self.assertTrue(self.lines)
        for line in self.lines:
            self.assertIn(f'robot mobipick2: pose selector {SELECTOR}', line)

    def test_a_client_without_a_name_is_shown_as_none(self):
        self.assertEqual(panel_module.service_name(None), '(none)')


if __name__ == '__main__':
    unittest.main()
