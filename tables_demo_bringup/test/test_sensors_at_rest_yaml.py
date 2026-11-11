'''The octomap sensors yaml used with octomap_update_only_at_rest is the sim sensors yaml of mobipick_moveit_config with
one change, the gated cloud topic: any other value that drifts makes the flag test something else than the plain sim.
Needs mobipick_moveit_config on the ROS package path (skipped without it).'''
import unittest
from pathlib import Path

import yaml

AT_REST = Path(__file__).resolve().parents[1] / 'config' / 'sensors_gripper_cam_pointcloud_sim_at_rest.yaml'


def plain_sim_yaml():
    try:
        import rospkg
        return Path(rospkg.RosPack().get_path('mobipick_moveit_config')) / 'config' / 'sensors_gripper_cam_pointcloud_sim.yaml'
    except Exception:
        return None


class TestSensorsAtRestYaml(unittest.TestCase):
    def test_only_the_cloud_topic_differs_from_the_plain_sim_yaml(self):
        plain = plain_sim_yaml()
        if plain is None or not plain.exists():
            self.skipTest('mobipick_moveit_config not found')
        expected = yaml.safe_load(plain.read_text())
        at_rest = yaml.safe_load(AT_REST.read_text())
        for sensor in expected['sensors']:
            sensor['point_cloud_topic'] += '_at_rest'
        self.assertEqual(at_rest, expected)


if __name__ == '__main__':
    unittest.main()
