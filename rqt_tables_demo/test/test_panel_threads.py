"""The panel runs every robot command in a worker thread with a busy state per robot: while one robot navigates, the
window answers and the other robots take commands, and Cancel acts on the chosen robot only.

The panel's own methods run on a stand-in object (fake widgets, fake robots), so no ROS master, rqt or robot is needed;
the module still needs the ROS workspace on the path (run it in the ROS image)."""
import os
import sys
import threading
import time
import types
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))
from rqt_tables_demo import rqt_tables_demo as panel_module  # noqa: E402

Panel = panel_module.RqtTablesDemo


class Box:
    def __init__(self, text=''):
        self.enabled = True
        self.text = text

    def setEnabled(self, enabled):
        self.enabled = enabled

    def setText(self, text):
        self.text = text

    def currentText(self):
        return self.text

    def isChecked(self):
        return True


class Base:
    def __init__(self, gate):
        self.gate = gate
        self.goals = []

    def move(self, x, y, yaw, **kwargs):
        self.goals.append((x, y, yaw, kwargs))
        self.gate.wait(5)
        return 3


class Robot:
    def __init__(self, namespace, gate):
        self.namespace = namespace
        self.base = Base(gate)
        self.arm = types.SimpleNamespace(moves=[])
        self.arm.move = self.arm.moves.append


class NavClient:
    """the panel's own move_base client of one robot: a goal runs until the robot's gate opens"""
    def __init__(self, gate):
        self.gate = gate
        self.goals = []
        self.cancels = 0

    def wait_for_server(self, timeout=None):
        return True

    def send_goal_and_wait(self, goal, execute_timeout=None):
        self.goals.append(goal)
        self.gate.wait(5)
        return 3

    def get_goal_status_text(self):
        return 'Goal reached.'

    def cancel_all_goals(self):
        self.cancels += 1


class Signal:
    def __init__(self, slot):
        self.slot = slot

    def emit(self, *args):
        self.slot(*args)


class FakePanel:
    """the attributes the panel's command methods use, with the methods taken from the real class"""
    COMMAND_GROUPS = Panel.COMMAND_GROUPS
    NAVIGATION_TIMEOUT = Panel.NAVIGATION_TIMEOUT

    def __init__(self, robots, gates):
        self.robots = robots
        self.robot_ns = 'mobipick'
        self.robot = robots['mobipick']
        self.launch_robot = 'mobipick'
        self.api_perception = None
        self.is_pose_selector_available = False
        self.is_gripper_srv_available = False
        self.activate_pose_selector_srv = self.open_gripper_srv = self.close_gripper_srv = None
        self.use_mobipick_api = False
        self._busy = {}
        self._busy_lock = threading.Lock()
        self._action_clients = {}
        self._api_busy_robots = {}
        self._nav_clients = {ns: NavClient(gates[ns]) for ns in robots}
        self._api_ns = 'mobipick'
        self._api_lock = threading.Lock()
        self._api_holder = ''
        self._switching_to = None
        self._build_lock = threading.Lock()
        self.wp_dic = {'table_1': [12.21, 2.1, 0.0, 0.0, 0.0, 0.707, 0.707]}
        self._widget = types.SimpleNamespace(comboNavigationWaypoints=Box('table_1'), optNavigationNavigate=Box(),
                                             **{name: Box() for name in Panel.COMMAND_GROUPS})
        self.labelBusy = Box()
        self.busy_changed = Signal(self._update_busy_ui)

    def choose(self, namespace):
        """what _robot_built does once a robot's clients are ready"""
        self.robot_ns, self.robot, self._api_ns = namespace, self.robots[namespace], namespace
        self._update_busy_ui()


for _name in ('_run', '_is_busy', '_with_robot_api', '_update_busy_ui', 'navigation_go', '_navigate', '_map_frame', 'cancel',
              '_nav_client', '_robot_built', '_perceive_objs_task', 'manipulation_go', 'perceive_objs'):
    setattr(FakePanel, _name, getattr(Panel, _name))


def wait_until(condition, timeout=5.0):
    end = time.time() + timeout
    while time.time() < end:
        if condition():
            return True
        time.sleep(0.01)
    return False


class PanelThreadsTest(unittest.TestCase):
    def setUp(self):
        self.gates = {ns: threading.Event() for ns in ('mobipick', 'mobipick2')}
        self.panel = FakePanel({ns: Robot(ns, gate) for ns, gate in self.gates.items()}, self.gates)
        real_now = panel_module.rospy.Time.now
        panel_module.rospy.Time.now = staticmethod(lambda: panel_module.rospy.Time(0))  # no ROS node in the test
        self.addCleanup(setattr, panel_module.rospy.Time, 'now', real_now)
        real_get_param = panel_module.rospy.get_param
        panel_module.rospy.get_param = lambda name, default=None: default  # no parameter server: the default map frame
        self.addCleanup(setattr, panel_module.rospy, 'get_param', real_get_param)

    def goals(self, ns):
        return self.panel._nav_clients[ns].goals

    def tearDown(self):
        for gate in self.gates.values():
            gate.set()

    def navigation_box(self):
        return self.panel._widget.Navigation_groupBox

    def test_navigate_returns_at_once_and_blocks_only_its_robot(self):
        started = time.time()
        self.panel.navigation_go()  # robot 1 now drives until its gate opens
        self.assertLess(time.time() - started, 1.0, 'the Qt thread waited for the robot')
        self.assertTrue(wait_until(lambda: self.goals('mobipick')))
        self.assertFalse(self.navigation_box().enabled)
        self.assertIn('navigate to table_1', self.panel.labelBusy.text)
        self.panel.choose('mobipick2')
        self.assertTrue(self.navigation_box().enabled, 'robot 2 is blocked by robot 1')
        self.assertIn('other robots: mobipick: navigate to table_1', self.panel.labelBusy.text)
        self.panel.navigation_go()  # robot 2 drives at the same time
        self.assertTrue(wait_until(lambda: self.goals('mobipick2')))
        self.assertEqual(self.goals('mobipick2')[0].target_pose.header.frame_id, 'mobipick2/map')
        self.assertEqual(self.panel.robots['mobipick2'].base.goals, [], 'navigation used the robot API')
        self.gates['mobipick'].set()
        self.assertTrue(wait_until(lambda: not self.panel._is_busy('mobipick', 'navigate')))
        self.assertFalse(self.navigation_box().enabled, 'robot 2 still drives')
        self.gates['mobipick2'].set()
        self.assertTrue(wait_until(lambda: self.navigation_box().enabled))
        self.assertEqual(self.panel.labelBusy.text, 'idle')

    def test_a_second_navigate_of_a_busy_robot_is_refused(self):
        self.panel.navigation_go()
        self.assertTrue(wait_until(lambda: self.goals('mobipick')))
        self.panel.navigation_go()
        time.sleep(0.2)
        self.assertEqual(len(self.goals('mobipick')), 1)

    def test_cancel_stops_only_the_chosen_robots_navigation(self):
        self.panel.navigation_go()
        self.panel.choose('mobipick2')
        self.panel.navigation_go()
        self.assertTrue(wait_until(lambda: self.goals('mobipick2')))
        self.panel.cancel()
        self.assertEqual(self.panel._nav_clients['mobipick2'].cancels, 1)
        self.assertEqual(self.panel._nav_clients['mobipick'].cancels, 0)

    def test_cancel_without_a_running_navigation_sends_nothing(self):
        self.panel.cancel()
        self.assertEqual(self.panel._nav_clients['mobipick'].cancels, 0)

    def test_no_command_starts_while_a_chosen_robot_is_being_built(self):
        self.panel._switching_to = 'mobipick2'
        self.panel.navigation_go()
        time.sleep(0.2)
        self.assertEqual(self.goals('mobipick'), [])

    def test_a_build_replaced_by_a_newer_choice_is_dropped(self):
        self.panel._switching_to = 'mobipick'
        self.panel._robot_built(('mobipick2', self.panel.robots['mobipick2'], None, {}))
        self.assertEqual(self.panel.robot_ns, 'mobipick')
        self.assertEqual(self.panel._switching_to, 'mobipick')

    def test_a_robot_api_command_does_not_start_when_the_robot_api_moved_to_another_robot(self):
        ctx = {'ns': 'mobipick', 'robot': self.panel.robots['mobipick'], 'activate': lambda on: None}
        self.panel._api_ns = 'mobipick2'  # robot 2 was built while this command waited for its turn
        self.panel._with_robot_api(ctx, 'perceive', lambda: self.panel._perceive_objs_task(ctx, ['observe100cm_right']))
        self.assertEqual(self.panel.robots['mobipick'].arm.moves, [])
        self.assertFalse(self.panel._api_lock.locked())

    def test_a_robot_api_command_holds_the_robot_api_until_it_ends(self):
        release = threading.Event()
        self.panel.robots['mobipick'].arm.move = lambda pose: release.wait(5)
        self.panel._widget.comboArmPoses = Box('home')
        self.panel.manipulation_go()
        self.assertTrue(wait_until(lambda: self.panel._api_lock.locked()))
        self.assertEqual(self.panel._api_holder, 'robot mobipick: arm to home')
        release.set()
        self.assertTrue(wait_until(lambda: not self.panel._api_lock.locked()))

    def test_navigation_does_not_hold_the_robot_api(self):
        self.panel.navigation_go()
        self.assertTrue(wait_until(lambda: self.goals('mobipick')))
        self.assertFalse(self.panel._api_lock.locked(), 'a drive would block every robot switch')

    def test_a_failing_command_frees_its_robot(self):
        def boom(ctx):
            raise RuntimeError('server gone')
        self.panel._run('Manipulation_groupBox', 'pick klt', boom)
        self.assertTrue(wait_until(lambda: self.panel._widget.Manipulation_groupBox.enabled))
        self.assertFalse(self.panel._is_busy('mobipick', 'pick'))


if __name__ == '__main__':
    unittest.main()
