import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src', 'rqt_tables_demo'))
import pick_list as pl  # noqa: E402


class PickListTest(unittest.TestCase):
    def test_enable_id_lists_the_perceived_objects_with_their_ids(self):
        self.assertEqual(pl.pick_items(['klt_1', 'power_drill_with_grip_2'], True, ['klt', 'power_drill_with_grip']),
                         ['klt_1', 'power_drill_with_grip_2'])

    def test_without_enable_id_the_classes_of_interest_are_listed(self):
        self.assertEqual(pl.pick_items(['klt_1'], False, ['klt', 'screwdriver']), ['klt', 'screwdriver'])

    def test_empty_hint_names_the_robot_and_the_cause(self):
        for ns in ('mobipick', '/mobipick/', '', None):
            self.assertIn('robot mobipick ', pl.empty_hint(ns))
        self.assertIn('robot mobipick2 ', pl.empty_hint('mobipick2'))
        self.assertIn('perceive', pl.empty_hint('mobipick2'))

    def test_unreachable_hint_carries_the_service_error(self):
        self.assertIn('service gone', pl.unreachable_hint('mobipick2', 'service gone'))


if __name__ == '__main__':
    unittest.main()
