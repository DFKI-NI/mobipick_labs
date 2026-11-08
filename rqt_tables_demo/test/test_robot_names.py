import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src', 'rqt_tables_demo'))
import robot_names as rn  # noqa: E402


class RobotNamesTest(unittest.TestCase):
    def test_robot_one_keeps_the_global_pose_selector_and_the_plain_map(self):
        self.assertEqual(rn.pose_selector_services('mobipick')['activate'],
                         '/pick_pose_selector_node/pose_selector_activate')
        self.assertEqual(rn.map_frame('/mobipick/'), 'map')

    def test_other_robots_get_their_own_names(self):
        self.assertEqual(rn.pose_selector_services('mobipick2')['clear'],
                         '/mobipick2/pick_pose_selector_node/pose_selector_clear')
        self.assertEqual(rn.gripper_services('mobipick3'),
                         {'open': '/mobipick3/pose_teacher/open_gripper',
                          'close': '/mobipick3/pose_teacher/close_gripper'})
        self.assertEqual(rn.action_server('mobipick2', 'pick_object'), '/mobipick2/pick_object')
        self.assertEqual(rn.map_frame('mobipick2'), 'mobipick2/map')

    def test_empty_namespace_is_robot_one(self):
        self.assertEqual(rn.normalize('/'), 'mobipick')
        self.assertEqual(rn.prefix(''), '/mobipick')

    def test_the_selection_topic_is_global(self):
        self.assertEqual(rn.SELECTED_ROBOT_TOPIC, '/selected_robot')

    def test_a_namespace_with_slashes_is_normalized(self):
        self.assertEqual(rn.normalize('/mobipick2/'), 'mobipick2')


    def test_the_robots_are_the_ones_with_a_move_base_in_natural_order(self):
        topics = ['/mobipick10/move_base/status', '/mobipick2/move_base/status', '/mobipick/move_base/status',
                  '/mobipick2/scan', '/move_base/status']
        self.assertEqual(rn.robots_from_topics(topics), ['mobipick', 'mobipick2', 'mobipick10'])
        self.assertEqual(rn.robots_from_topics([], '/mobipick2/'), ['mobipick2'])

    def test_a_robot_in_a_nested_namespace_is_listed_too(self):
        topics = ['/fleet/robot3/move_base/status', '/mobipick/move_base/status', '/fleet/robot3/scan']
        self.assertEqual(rn.robots_from_topics(topics), ['mobipick', 'fleet/robot3'])

    def test_a_choice_from_elsewhere_is_followed_once(self):
        self.assertEqual(rn.to_follow('mobipick', '/mobipick2'), 'mobipick2')
        self.assertIsNone(rn.to_follow('mobipick2', 'mobipick2/'))


    def test_the_owner_runs_when_a_node_listens_on_the_request_topic(self):
        self.assertTrue(rn.owner_running(([], [['/selected_robot/request', ['/selected_robot']]], [])))
        self.assertFalse(rn.owner_running(([], [['/selected_robot', ['/rqt']]], [])))


if __name__ == '__main__':
    unittest.main()
