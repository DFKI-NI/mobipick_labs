#!/usr/bin/env python3
"""One tables demo planner per robot: the ROS names written for robot 1 point at the planner's robot (its pose
selector, parameters, frames, joint states), robot 1 keeps them unchanged. Needs the ROS workspace (run in the image)."""
import importlib.util
import os
import unittest

import rospy
from tables_demo_planning.tables_demo_api import TablesDemoAPI, default_map_frame

_spec = importlib.util.spec_from_file_location(
    'tables_demo_node', os.path.join(os.path.dirname(__file__), '..', 'nodes', 'tables_demo_node.py'))
node = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(node)


def planner(robot_namespace, map_frame=None):
    api = TablesDemoAPI.__new__(TablesDemoAPI)  # no ROS: only the naming
    api.robot_namespace, api.map_frame = robot_namespace, map_frame or default_map_frame(robot_namespace)
    return api


class RobotNamespaceTest(unittest.TestCase):
    def test_robot_1_keeps_its_names(self):
        api = planner('mobipick')
        self.assertEqual(api.map_frame, 'map')
        for name in ('/mobipick/tables_demo_planning', '/pick_pose_selector_node/pose_selector_delete', '/map',
                     '/mobipick/base_link', '/mobipick/joint_states'):
            self.assertEqual(api.ros_name(name), name)

    def test_robot_2_gets_its_own_names(self):
        api = planner('mobipick2')
        self.assertEqual(api.map_frame, 'mobipick2/map')
        self.assertEqual(api.ros_name('/mobipick/tables_demo_planning'), '/mobipick2/tables_demo_planning')
        self.assertEqual(api.ros_name('/pick_pose_selector_node/pose_selector_class_query'),
                         '/mobipick2/pick_pose_selector_node/pose_selector_class_query')
        self.assertEqual(api.ros_name('/mobipick/pick_object_node/planning_scene_boxes'),
                         '/mobipick2/pick_object_node/planning_scene_boxes')
        self.assertEqual(api.ros_name('/map'), '/mobipick2/map')
        self.assertEqual(api.ros_name('/mobipick/base_link'), '/mobipick2/base_link')
        self.assertEqual(api.ros_name('/mobipick/joint_states'), '/mobipick2/joint_states')

    def test_robot_3_gets_its_own_names(self):
        api = planner('mobipick3')
        self.assertEqual(api.map_frame, 'mobipick3/map')
        self.assertEqual(api.ros_name('/pick_pose_selector_node/pose_selector_delete'),
                         '/mobipick3/pick_pose_selector_node/pose_selector_delete')
        self.assertEqual(api.ros_name('/mobipick/tables_demo_planning'), '/mobipick3/tables_demo_planning')
        self.assertEqual(api.ros_name('/map'), '/mobipick3/map')
        self.assertEqual(api.ros_name('/mobipick/joint_states'), '/mobipick3/joint_states')

    def test_a_given_map_frame_wins(self):
        self.assertEqual(planner('mobipick2', 'world').ros_name('/map'), '/world')

    def test_robot_2_navigates_in_its_own_map_frame(self):
        api = planner('/mobipick2/')
        calls = []
        api.mobipick_api = type('R', (), {'base': type('B', (), {
            'move': staticmethod(lambda pose, **kwargs: calls.append(kwargs) or 3)})()})()
        self.assertEqual(api.move_base_to('pose'), 3)
        self.assertEqual(calls, [{'frame_id': 'mobipick2/map'}])
        api1 = planner('mobipick')
        api1.mobipick_api = api.mobipick_api
        api1.move_base_to('pose')
        self.assertEqual(calls[-1], {}, 'robot 1 calls base.move exactly as before')


class NodeNamespaceTest(unittest.TestCase):
    def namespace(self, param, node_namespace):
        real = rospy.get_param, rospy.get_namespace
        rospy.get_param = lambda name, default=None: param or default
        rospy.get_namespace = lambda: node_namespace
        try:
            return node.robot_namespace()
        finally:
            rospy.get_param, rospy.get_namespace = real

    def test_the_robot_comes_from_the_param_the_node_namespace_or_defaults_to_robot_1(self):
        self.assertEqual(self.namespace('', '/'), 'mobipick')
        self.assertEqual(self.namespace('', '/mobipick2/'), 'mobipick2')
        self.assertEqual(self.namespace('', '/mobipick3/'), 'mobipick3')
        self.assertEqual(self.namespace('/mobipick3/', '/'), 'mobipick3')


if __name__ == '__main__':
    unittest.main()
