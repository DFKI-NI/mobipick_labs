"""The panel's robot switch: a robot chosen here goes out on the latched /selected_robot topic; a robot chosen elsewhere
is followed, and once this panel has published it publishes the other GUI's choice again, so every latched message names
the same robot. The panel makes its /selected_robot publisher only for its first publish, so a panel that only follows is
no writer of the topic. With the sim's selected-robot owner node running, a choice here is only a request to it, the
panel follows the owner's answer and withdraws a publisher made before the owner came up. The panel's own methods run on a stand-in object (no ROS master or rqt); run it in the
ROS image."""
import os
import sys
import types
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))
from rqt_tables_demo import rqt_tables_demo as panel_module  # noqa: E402

Panel = panel_module.RqtTablesDemo


class Publisher:
    made = []  # every publisher the panel made, in order

    def __init__(self, *args, **kwargs):
        self.sent = []
        self.unregistered = False
        Publisher.made.append(self)

    def publish(self, msg):
        self.sent.append(msg.data)

    def unregister(self):
        self.unregistered = True


class FakePanel:
    def __init__(self):
        self.robot_ns = 'mobipick'
        self._switching_to = None
        self._owner_running = False
        self._published = None
        self._selected_robot_pub = None  # made on the first publish
        self._request_pub = types.SimpleNamespace(sent=[], publish=lambda msg: self._request_pub.sent.append(msg.data))
        self.labelRobot = types.SimpleNamespace(setText=lambda text: None)
        self.selected = []
        self.shown = []

    def _show_robot_name(self, namespace=None, note=''):
        self.shown.append(namespace or self.robot_ns)

    def _select_robot(self, namespace):
        self.selected.append(namespace)


for _name in ('_choose_robot', '_heard_robot', '_publish_choice', '_drop_own_choice'):
    setattr(FakePanel, _name, getattr(Panel, _name))


class RobotSwitchTest(unittest.TestCase):
    def setUp(self):
        Publisher.made = []
        real_publisher = panel_module.rospy.Publisher
        panel_module.rospy.Publisher = Publisher
        self.addCleanup(setattr, panel_module.rospy, 'Publisher', real_publisher)
        self.panel = FakePanel()

    def sent(self):
        """everything the panel published on /selected_robot, over all its publishers"""
        return [data for publisher in Publisher.made for data in publisher.sent]

    def test_a_panel_that_only_follows_has_no_publisher(self):
        self.panel._heard_robot('/mobipick2')
        self.panel._heard_robot('mobipick')
        self.assertEqual(Publisher.made, [])
        self.assertEqual(self.panel.selected, ['mobipick2', 'mobipick'])

    def test_a_choice_here_is_published_and_selected(self):
        self.panel._choose_robot('mobipick2/')
        self.panel._choose_robot('mobipick3')
        self.assertEqual(len(Publisher.made), 1)  # one publisher, kept
        self.assertEqual(self.sent(), ['mobipick2', 'mobipick3'])
        self.assertEqual(self.panel.selected, ['mobipick2', 'mobipick3'])

    def test_after_publishing_a_choice_from_elsewhere_is_published_again_once(self):
        self.panel._choose_robot('mobipick2')
        self.panel._heard_robot('mobipick2')  # its own message comes back: no echo
        self.panel._heard_robot('mobipick3')  # the sim window chose robot 3
        self.panel._heard_robot('mobipick3')  # its echo comes back
        self.assertEqual(self.sent(), ['mobipick2', 'mobipick3'])
        self.assertEqual(self.panel._published, 'mobipick3')

    def test_with_the_owner_node_a_choice_is_only_a_request(self):
        self.panel._owner_running = True
        self.panel._choose_robot('mobipick2')
        self.assertEqual(self.panel._request_pub.sent, ['mobipick2'])
        self.assertEqual(self.panel.selected, [])  # the robot changes when the owner says so
        self.panel._heard_robot('mobipick2')
        self.assertEqual(self.panel.selected, ['mobipick2'])
        self.assertEqual(Publisher.made, [])

    def test_when_the_owner_node_comes_up_the_panel_withdraws_its_publisher(self):
        self.panel._choose_robot('mobipick2')  # published before the owner started
        self.panel._owner_running = True
        self.panel._heard_robot('mobipick3')  # the owner's first latched message
        self.assertEqual(self.sent(), ['mobipick2'])  # no echo
        self.assertTrue(Publisher.made[0].unregistered)
        self.assertIsNone(self.panel._selected_robot_pub)
        self.assertEqual(self.panel.selected, ['mobipick2', 'mobipick3'])

    def test_after_the_owner_node_stops_the_panel_publishes_again_on_a_choice_here(self):
        self.panel._choose_robot('mobipick2')
        self.panel._owner_running = True
        self.panel._choose_robot('mobipick3')  # asks the owner, withdraws the old publisher
        self.panel._owner_running = False
        self.panel._heard_robot('mobipick3')  # only follows: nothing published yet in this owner-less phase
        self.assertEqual(len(Publisher.made), 1)
        self.panel._choose_robot('mobipick')
        self.assertEqual(len(Publisher.made), 2)
        self.assertEqual(Publisher.made[1].sent, ['mobipick'])

    def test_a_refused_request_shows_the_chosen_robot_again(self):
        self.panel._owner_running = True
        self.panel._choose_robot('mobipick4')
        self.panel._heard_robot('mobipick')  # the owner refused and sent the current robot again
        self.assertEqual(self.panel.shown[-1], 'mobipick')


if __name__ == '__main__':
    unittest.main()
