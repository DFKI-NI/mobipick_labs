"""The panel launched for one robot can switch to the others, and the Mobipick API it builds for each robot reads the
process's private parameters. Overrides of the pose selector names are therefore scoped to their robot
(~robots/<robot>/<param>, as the launch file writes them): they reach that robot only, never another one the panel
switches to; an unscoped ~<param> still applies to the launch robot. No ROS master: rospy's parameter and service
waits are stand-ins; run it in the ROS image."""
import os
import sys
import types
import unittest
from unittest import mock

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))
from rqt_tables_demo import rqt_tables_demo as panel_module  # noqa: E402

Panel = panel_module.RqtTablesDemo


class FakePanel:
    def __init__(self, robot_ns, api_perception=None):
        self.launch_robot = 'mobipick3'
        self.robot_ns = robot_ns
        self.use_mobipick_api = api_perception is not None
        self.api_perception = api_perception
        self.robot = types.SimpleNamespace()   # no API gripper: the panel looks for the gripper services as well
        self.wait_for_services = 0.1
        self._widget = mock.MagicMock()


FakePanel._find_services = Panel._find_services


class PoseSelectorOverrideTest(unittest.TestCase):
    def selector_names(self, namespace, params, api_perception=None):
        """the pose selector services the panel waits for when robot `namespace` is chosen"""
        waited = []

        def wait_for_service(name, timeout=None):
            waited.append(name)
            raise RuntimeError('not running here')

        fake_rospy = types.SimpleNamespace(
            get_param=lambda name, default=None: params.get(name, default), wait_for_service=wait_for_service,
            loginfo=lambda *a: None, logwarn=lambda *a: None, ServiceProxy=mock.Mock())
        with mock.patch.object(panel_module, 'rospy', fake_rospy):
            panel = FakePanel(namespace, api_perception)
            panel._find_services(namespace, panel.robot, api_perception)
        return waited[0]  # the activate service comes first; the others are not reached when it is missing

    def test_the_launch_files_scoped_override_reaches_its_robot_only(self):
        params = {'~robots/mobipick3/pose_selector_activate_srv_name': '/elsewhere/pose_selector_activate'}
        self.assertEqual(self.selector_names('mobipick3', params), '/elsewhere/pose_selector_activate')
        self.assertEqual(self.selector_names('mobipick2', params),
                         '/mobipick2/pick_pose_selector_node/pose_selector_activate')
        self.assertEqual(self.selector_names('mobipick', params), '/pick_pose_selector_node/pose_selector_activate')

    def test_an_unscoped_override_applies_to_the_launch_robot_only(self):
        params = {'~pose_selector_activate_srv_name': '/custom/pose_selector_activate'}
        self.assertEqual(self.selector_names('mobipick3', params), '/custom/pose_selector_activate')
        self.assertEqual(self.selector_names('mobipick2', params),
                         '/mobipick2/pick_pose_selector_node/pose_selector_activate')

    def test_a_scoped_override_also_reaches_a_robot_switched_to(self):
        params = {'~robots/mobipick2/pose_selector_activate_srv_name': '/second/pose_selector_activate'}
        self.assertEqual(self.selector_names('mobipick2', params), '/second/pose_selector_activate')

    def test_with_the_mobipick_api_each_robot_switched_to_uses_its_own_api_perceptions_names(self):
        # the Mobipick API reads the same scoped names (its own tests); the panel takes them from the API perception it
        # built for the chosen robot, never from the launch robot's
        params = {'~robots/mobipick3/pose_selector_activate_srv_name': '/mobipick3/pick_pose_selector_node/x',
                  '~pose_selector_activate_srv_name': '/unscoped/pose_selector_activate'}
        for robot in ('mobipick', 'mobipick2', 'mobipick3'):
            api = types.SimpleNamespace(**{f'pose_selector_{name}_srv_name': f'/{robot}/api/{name}'
                                           for name in ('activate', 'class_query', 'get_all_poses', 'clear')},
                                        activate_pose_selector_srv=None, pose_selector_class_query_srv=None,
                                        pose_selector_get_all_poses_srv=None, pose_selector_clear_srv=None)
            self.assertEqual(self.selector_names(robot, params, api), f'/{robot}/api/activate')


if __name__ == '__main__':
    unittest.main()
