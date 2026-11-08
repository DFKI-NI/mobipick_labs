"""Navigation goals and teletransports go in the map frame the launch file wrote for the chosen robot
(~robots/<robot>/map_frame), not one guessed from its name: robot 1 keeps the global map also when it is renamed, and
only without the parameter the default sim's convention applies. No ROS master: rospy's parameters are a stand-in."""
import os
import sys
import types
import unittest
from unittest import mock

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))
from rqt_tables_demo import rqt_tables_demo as panel_module  # noqa: E402


class FakePanel:
    _map_frame = panel_module.RqtTablesDemo._map_frame


class MapFrameTest(unittest.TestCase):
    def frame(self, namespace, params):
        fake_rospy = types.SimpleNamespace(get_param=lambda name, default=None: params.get(name, default))
        with mock.patch.object(panel_module, 'rospy', fake_rospy):
            return FakePanel()._map_frame(namespace)

    def test_a_renamed_robot_1_navigates_in_the_global_map(self):
        params = {'~robots/alpha/map_frame': 'map', '~robots/alpha2/map_frame': 'alpha2/map'}
        self.assertEqual(self.frame('alpha', params), 'map')       # the name-based guess said alpha/map
        self.assertEqual(self.frame('/alpha2/', params), 'alpha2/map')

    def test_a_nested_robot_takes_its_frame_from_the_launch(self):
        self.assertEqual(self.frame('team/alpha', {'~robots/team/alpha/map_frame': 'map'}), 'map')

    def test_without_the_parameter_the_default_sim_convention_applies(self):
        self.assertEqual(self.frame('mobipick', {}), 'map')
        self.assertEqual(self.frame('mobipick3', {}), 'mobipick3/map')


if __name__ == '__main__':
    unittest.main()
