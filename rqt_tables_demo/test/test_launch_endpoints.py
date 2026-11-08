"""rqt_tables_demo.launch writes the perception endpoints of every robot the panel can switch to as private parameters
scoped to that robot (robots/<robot>/<name>): all five pose selector services and the four perception actions. Which
pose selector a robot uses is a launch choice (robot 1's is the global one also when robot 1 is renamed), so the launch
writes it explicitly instead of anyone deriving it from the robot name. Needs roslaunch (run it in the ROS image)."""
import os
import shutil
import subprocess
import unittest

import yaml

LAUNCH = os.path.join(os.path.dirname(__file__), '..', 'launch', 'rqt_tables_demo.launch')
NAMES = ('pose_selector_activate_srv_name', 'pose_selector_class_query_srv_name',
         'pose_selector_get_all_poses_srv_name', 'pose_selector_delete_srv_name', 'pose_selector_clear_srv_name',
         'perceive_action_name', 'inspect_object_action_name', 'detect_objects_action_name',
         'verify_objects_action_name')


@unittest.skipIf(shutil.which('roslaunch') is None, 'needs roslaunch')
class LaunchEndpointsTest(unittest.TestCase):
    def endpoints(self, *args):
        """{robot: {name: value}} of the scoped parameters the launch writes for the panel node"""
        out = subprocess.run(['roslaunch', '--dump-params', LAUNCH, *args], capture_output=True, text=True, check=True)
        params = yaml.safe_load(out.stdout)
        robots = {}
        for key, value in params.items():
            head, sep, tail = key.partition('/rqt_tables_demo/robots/')
            if sep:
                robot, _, name = tail.rpartition('/')
                robots.setdefault(robot, {})[name] = value
        return robots

    def selector(self, robots, robot):
        return robots[robot]['pose_selector_activate_srv_name'].rpartition('/pick_pose_selector_node/')[0]

    def test_every_robot_gets_all_nine_endpoints_and_robot_1_the_global_selector(self):
        robots = self.endpoints('namespace:=mobipick')
        self.assertEqual(sorted(robots), ['mobipick', 'mobipick2', 'mobipick3'])
        for robot, names in robots.items():
            self.assertEqual(sorted(names), sorted(NAMES), robot)
            self.assertEqual(names['perceive_action_name'], f'/{robot}/perceive')
        self.assertEqual(robots['mobipick']['pose_selector_delete_srv_name'],
                         '/pick_pose_selector_node/pose_selector_delete')
        self.assertEqual(self.selector(robots, 'mobipick2'), '/mobipick2')
        self.assertEqual(self.selector(robots, 'mobipick3'), '/mobipick3')

    def test_a_panel_launched_for_robot_3_maps_the_same_robots(self):
        self.assertEqual(self.endpoints('namespace:=mobipick3'), self.endpoints('namespace:=mobipick'))

    def test_a_renamed_robot_1_keeps_the_global_selector(self):
        robots = self.endpoints('namespace:=alpha', 'robot_1:=alpha')
        self.assertEqual(sorted(robots), ['alpha', 'alpha2', 'alpha3'])
        self.assertEqual(self.selector(robots, 'alpha'), '')
        self.assertEqual(robots['alpha']['perceive_action_name'], '/alpha/perceive')
        self.assertEqual(self.selector(robots, 'alpha2'), '/alpha2')

    def test_a_nested_namespace_keeps_its_inner_segments(self):
        robots = self.endpoints('namespace:=/team/alpha/', 'robot_1:=team/alpha', 'selectable_robots:=1')
        self.assertEqual(sorted(robots), ['team/alpha'])
        self.assertEqual(robots['team/alpha']['verify_objects_action_name'], '/team/alpha/verify_objects')
        self.assertEqual(self.selector(robots, 'team/alpha'), '')

    def test_a_custom_selector_of_the_launch_robot_covers_all_five_services(self):
        robots = self.endpoints('namespace:=mobipick2', 'pose_selector_ns:=elsewhere')
        for name in NAMES[:5]:
            self.assertTrue(robots['mobipick2'][name].startswith('/elsewhere/pick_pose_selector_node/'), name)
        self.assertEqual(self.selector(robots, 'mobipick'), '')


if __name__ == '__main__':
    unittest.main()
