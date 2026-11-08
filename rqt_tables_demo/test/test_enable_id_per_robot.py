import os
import sys
import types
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))
from rqt_tables_demo import rqt_tables_demo as panel  # noqa: E402


class Combo:
    def __init__(self):
        self.items = []

    def clear(self):
        self.items = []

    def addItems(self, items):
        self.items += list(items)


class Check:
    def __init__(self):
        self.checked, self.tip, self.text = False, '', '-'

    def isChecked(self):
        return self.checked

    def setToolTip(self, tip):
        self.tip = tip

    def setText(self, text):
        self.text = text


def selector(*objects):
    """a pose selector's get_all service holding (class, instance) objects"""
    poses = types.SimpleNamespace(objects=[types.SimpleNamespace(class_id=c, instance_id=i) for c, i in objects])
    return lambda: types.SimpleNamespace(poses=poses)


class EnableIdPerRobotTest(unittest.TestCase):
    def setUp(self):
        self.panel = panel.RqtTablesDemo.__new__(panel.RqtTablesDemo)
        self.panel._widget = types.SimpleNamespace(comboPickObj=Combo(), comboInsertHole=Combo(),
                                                   chkPickEnableId=Check())
        self.panel.ignore_from_ps_chks = [Check() for _ in range(8)]
        self.panel.objects_of_interest = ['klt', 'screwdriver']
        self.panel.hole_objects = ['klt']
        self.selectors = {'mobipick': selector(('klt', 1), ('screwdriver', 2)),
                          'mobipick2': selector(('power_drill_with_grip', 7))}

    def use(self, robot):
        """what a robot switch does: the panel's service client now is the chosen robot's"""
        self.panel.robot_ns = robot
        self.panel.pose_selector_get_all_poses_srv = self.selectors[robot]

    def pick_list(self):
        return self.panel._widget.comboPickObj.items

    def test_each_robot_lists_its_own_ids_when_switching_back_and_forth(self):
        self.panel._widget.chkPickEnableId.checked = True
        for robot, expected in (('mobipick', ['klt_1', 'screwdriver_2']), ('mobipick2', ['power_drill_with_grip_7']),
                                ('mobipick', ['klt_1', 'screwdriver_2']), ('mobipick2', ['power_drill_with_grip_7'])):
            self.use(robot)
            self.panel.manipulation_update()
            self.assertEqual(self.pick_list(), expected, robot)

    def test_without_enable_id_both_robots_list_the_classes(self):
        for robot in ('mobipick', 'mobipick2', 'mobipick'):
            self.use(robot)
            self.panel.manipulation_update()
            self.assertEqual(self.pick_list(), ['klt', 'screwdriver'], robot)

    def test_an_empty_selector_does_not_keep_the_other_robots_ids_and_says_why(self):
        self.panel._widget.chkPickEnableId.checked = True
        self.use('mobipick')
        self.panel.manipulation_update()
        self.selectors['mobipick2'] = selector()
        self.use('mobipick2')
        self.panel.manipulation_update()
        self.assertEqual(self.pick_list(), ['klt', 'screwdriver'])
        self.assertIn('robot mobipick2 ', self.panel._widget.chkPickEnableId.tip)
        self.use('mobipick')
        self.panel.manipulation_update()
        self.assertEqual(self.pick_list(), ['klt_1', 'screwdriver_2'])
        self.assertEqual(self.panel._widget.chkPickEnableId.tip, '')

    def test_without_pose_selector_services_the_panel_says_so_instead_of_raising(self):
        self.use('mobipick')
        self.panel.pose_selector_get_all_poses_srv = None
        self.panel.manipulation_update()
        self.assertIn('robot mobipick:', self.panel._widget.chkPickEnableId.tip)
        self.assertIn('not found', self.panel._widget.chkPickEnableId.tip)

    def test_the_enable_id_label_says_when_the_selector_is_empty_or_unreachable_and_clears_when_filled(self):
        check = self.panel._widget.chkPickEnableId
        check.checked = True
        self.selectors['mobipick'] = selector()
        self.use('mobipick')
        self.panel.manipulation_update()
        self.assertIn('no perceived objects', check.text)
        self.assertIn('mobipick', check.text)
        self.panel.pose_selector_get_all_poses_srv = None
        self.panel.manipulation_update()
        self.assertIn('not reachable', check.text)
        self.use('mobipick2')
        self.panel.manipulation_update()
        self.assertEqual(check.text, '')

    def test_a_robot_switch_refreshes_the_ticked_list_and_clears_the_label_of_the_other_robot(self):
        check = self.panel._widget.chkPickEnableId
        check.checked = True
        self.selectors['mobipick'] = selector()
        self.use('mobipick')
        self.panel.manipulation_update()
        self.assertIn('no perceived objects of mobipick', check.text)
        self.panel._switching_to = 'mobipick2'
        self.panel._apply_services = lambda found: self.use('mobipick2')
        self.panel._show_robot_name = self.panel._update_busy_ui = lambda: None
        self.panel._robot_built(('mobipick2', object(), None, {}))
        self.assertEqual(check.text, '')
        self.assertEqual(self.pick_list(), ['power_drill_with_grip_7'])

    def test_a_robot_switch_with_enable_id_off_only_clears_the_label(self):
        check = self.panel._widget.chkPickEnableId
        check.text, check.tip = 'no perceived objects of mobipick', 'old'
        self.panel._switching_to = 'mobipick2'
        self.panel.robot_ns = 'mobipick'
        self.panel._apply_services = lambda found: self.use('mobipick2')
        self.panel._show_robot_name = self.panel._update_busy_ui = lambda: None
        self.panel._robot_built(('mobipick2', object(), None, {}))
        self.assertEqual((check.text, check.tip), ('', ''))
        self.assertEqual(self.pick_list(), [])


if __name__ == '__main__':
    unittest.main()
