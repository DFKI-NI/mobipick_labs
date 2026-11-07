'''The nine grasplan feature switches that used to follow /use_sim_time are plain launch arguments now: the defaults are
the real-robot values, every include passes on only what the included file declares, and the nodes that read a switch
get it as a parameter. No ROS needed (XML only).'''
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

LAUNCH = Path(__file__).resolve().parents[1] / 'launch'
DEFAULTS = {
    'external_grasp_ik_timeout': '0.1', 'open_set_pick_from_current_state': 'false',
    'drop_floating_boxes_at_target': '0.0', 'place_sample_within_reach': 'false',
    'place_free_footprints_first': 'false', 'place_drop_implausible_boxes': 'false',
    'mtc_cable_guard_named_moves': 'log', 'mtc_place_reject_octomap_occupied': 'false',
    'mtc_grasp_check_squeeze': 'false',
}
FILES = ['demo_sim.launch', 'bringup.launch', 'includes/robot_stack_sim.launch', 'includes/grasp_planning.launch',
         'includes/place.launch', 'includes/insert.launch']
READERS = {'includes/grasp_planning.launch': {'external_grasp_ik_timeout', 'open_set_pick_from_current_state',
                                              'drop_floating_boxes_at_target', 'mtc_cable_guard_named_moves',
                                              'mtc_place_reject_octomap_occupied', 'mtc_grasp_check_squeeze'},
           'includes/place.launch': {'place_sample_within_reach', 'place_free_footprints_first',
                                     'place_drop_implausible_boxes', 'mtc_cable_guard_named_moves',
                                     'mtc_place_reject_octomap_occupied', 'mtc_grasp_check_squeeze'}}
READERS['includes/insert.launch'] = READERS['includes/place.launch'] - {'drop_floating_boxes_at_target'}


def root(name):
    return ET.parse(LAUNCH / name).getroot()


class TestUseSimTimeFreeDefaults(unittest.TestCase):
    def test_defaults_are_the_real_robot_values(self):
        for name in FILES:
            declared = {a.get('name'): a.get('default') for a in root(name).findall('arg')}
            for flag, default in DEFAULTS.items():
                if flag in declared:
                    self.assertEqual(declared[flag], default, f'{name} {flag}')

    def test_every_chain_file_declares_what_it_needs(self):
        for name in FILES[:3]:
            declared = {a.get('name') for a in root(name).findall('arg')}
            self.assertLessEqual(set(DEFAULTS), declared, name)

    def test_includes_pass_only_declared_arguments(self):
        for name in FILES:
            for include in root(name).iter('include'):
                target = include.get('file', '')
                for known in FILES:
                    if target.endswith('/' + known.split('/')[-1]) and known in FILES:
                        declared = {a.get('name') for a in root(known).findall('arg')}
                        passed = {a.get('name') for a in include.findall('arg')} & set(DEFAULTS)
                        self.assertLessEqual(passed, declared, f'{name} -> {known}')

    def test_reader_nodes_get_their_parameters(self):
        for name, flags in READERS.items():
            params = {p.get('name') for p in root(name).iter('param')}
            self.assertLessEqual(flags, params, name)

    def test_nothing_reads_the_sim_clock(self):
        for name in FILES:
            self.assertNotIn('use_sim_time', (LAUNCH / name).read_text().replace('$(arg use_sim_time)', ''), name)


if __name__ == '__main__':
    unittest.main()
