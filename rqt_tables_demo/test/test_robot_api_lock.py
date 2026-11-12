"""The robot API (robot_api) serves one robot per process: building a robot re-points its single ROS wrapper, and a
client still being connected for the robot before lands in the new robot's table, so that robot's next arm move would
go to the other arm. The panel holds the robot API for a whole arm, gripper, perceive or handover command and builds a
newly chosen robot only in between; the switch says what it waits for.

The session case of 2026-10-30: robot 2 perceives (its arm connects), the user switches to robot 1, robot 1 perceives.
Also: a robot build that fails part-way has already re-pointed the wrapper, so the robot it served before must not
trust it, and a second panel of the same process (rqt loads its plugins into one process) shares the one wrapper, so
the lock and the record of the robot served are the process's, not the panel's. A command whose robot the wrapper no
longer serves builds its robot again (pointing the wrapper back) before it runs.

A fake robot API with robot_api's one shared wrapper stands in; the panel's own methods run on a stand-in object (no
ROS master or rqt); run it in the ROS image."""
import os
import sys
import threading
import time
import types
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))
from rqt_tables_demo import rqt_tables_demo as panel_module  # noqa: E402

Panel = panel_module.RqtTablesDemo


class Wrapper:
    """robot_api's one ROS wrapper per process: namespace and client table of the robot built last"""
    def __init__(self):
        self.namespace = ''
        self.clients = {}
        self.connecting = threading.Event()  # set while a connect waits for its server
        self.server_answers = threading.Event()

    def init(self, namespace):
        self.namespace, self.clients = namespace, {}

    def connect(self, name):
        if name not in self.clients:
            client = self.namespace + name  # made for the robot the wrapper serves now
            self.connecting.set()
            self.server_answers.wait(5)
            self.clients[name] = client  # into the table of the robot the wrapper serves NOW
        return self.clients[name]


class Arm:
    def __init__(self, wrapper, moves):
        self.wrapper, self.moves = wrapper, moves

    def move(self, pose):
        self.moves.append((self.wrapper.connect('/moveit_macros'), pose))


class FakePanel:
    def __init__(self, robot_ns='mobipick2', wrapper=None, owner=None, moves=None):
        """a panel; two panels of one process share `wrapper` (robot_api's) and `owner` (the panel module's)"""
        self.wrapper = wrapper or Wrapper()
        self.moves = [] if moves is None else moves
        self.robot_ns = robot_ns
        self.launch_robot = 'mobipick'
        self.robot_api_class = 'robot_api.Robot'
        self.broken = set()  # robots whose build fails after re-pointing the wrapper (no semantic description)
        self._switching_to = None
        self._build_lock = threading.Lock()
        self._robot_api = owner or panel_module.RobotApiOwner()
        self.hints = []
        self.built = []
        self.robot_built = types.SimpleNamespace(emit=self.built.append)
        self.build_waiting = types.SimpleNamespace(emit=lambda ns, note: self.hints.append((ns, note)))
        with self._robot_api.lock:
            self.robot = self._point_robot_api(robot_ns)

    def make_robot(self, api_class_path, namespace):
        self.wrapper.init(f'/{namespace}')  # robot_api re-points its wrapper first (Arm.__init__)
        if namespace in self.broken:
            raise AssertionError(f"Error: Cannot parse 'robot_description_semantic' of /{namespace}")
        return types.SimpleNamespace(arm=Arm(self.wrapper, self.moves))

    def _find_services(self, namespace, robot, api_perception):
        return {}

    def _nav_client(self, namespace):
        return None


for _name in ('_build_robot', '_with_robot_api', '_point_robot_api'):
    setattr(FakePanel, _name, getattr(Panel, _name))


def wait_until(condition, timeout=5.0):
    end = time.time() + timeout
    while time.time() < end:
        if condition():
            return True
        time.sleep(0.01)
    return False


class RobotApiLockTest(unittest.TestCase):
    def setUp(self):
        real = panel_module.ApiPerception
        panel_module.ApiPerception = None
        self.addCleanup(setattr, panel_module, 'ApiPerception', real)
        self.panel = FakePanel()

    def perceive(self, ns, robot):
        ctx = {'ns': ns, 'robot': robot}
        return threading.Thread(target=self.panel._with_robot_api, args=(
            ctx, 'perceive', lambda: robot.arm.move('observe100cm_right')), daemon=True)

    def test_a_switch_during_another_robots_arm_connect_waits_and_each_arm_gets_its_own_move(self):
        robot2 = self.panel.robot
        perceive2 = self.perceive('mobipick2', robot2)
        perceive2.start()
        self.assertTrue(self.panel.wrapper.connecting.wait(5))  # robot 2's arm client waits for its server
        self.panel._switching_to = 'mobipick'  # the user picks robot 1
        build = threading.Thread(target=self.panel._build_robot, args=('mobipick',), daemon=True)
        build.start()
        self.assertTrue(wait_until(lambda: self.panel.hints))
        self.assertEqual(self.panel.hints, [('mobipick', 'waiting: robot mobipick2: perceive')])
        self.assertEqual(self.panel.wrapper.namespace, '/mobipick2', 'robot 1 was built during robot 2\'s connect')
        self.panel.wrapper.server_answers.set()
        perceive2.join(5)
        build.join(5)
        robot1 = self.panel.built[0][1]
        self.assertEqual(self.panel._robot_api.ns, 'mobipick')
        perceive1 = self.perceive('mobipick', robot1)
        perceive1.start()
        perceive1.join(5)
        self.assertEqual(self.panel.moves, [('/mobipick2/moveit_macros', 'observe100cm_right'),
                                            ('/mobipick/moveit_macros', 'observe100cm_right')])

    def test_a_switch_with_no_robot_api_command_running_builds_at_once(self):
        self.panel._switching_to = 'mobipick'
        self.panel._build_robot('mobipick')
        self.assertEqual(self.panel.hints, [])
        self.assertEqual(self.panel.wrapper.namespace, '/mobipick')
        self.assertFalse(self.panel._robot_api.lock.locked())

    def command(self, panel, ns, robot, pose='home'):
        ctx = {'ns': ns, 'robot': robot}
        return panel._with_robot_api(ctx, f'arm to {pose}', lambda: ctx['robot'].arm.move(pose) or 'done')

    def test_a_command_of_the_robot_left_behind_reaches_its_own_robot_not_the_new_one(self):
        robot2 = self.panel.robot
        self.panel._switching_to = 'mobipick'
        self.panel._build_robot('mobipick')
        self.assertEqual(self.command(self.panel, 'mobipick2', robot2), 'done')
        self.assertEqual(self.panel.moves, [('/mobipick2/moveit_macros', 'home')])
        self.assertEqual(self.panel._robot_api.ns, 'mobipick2')

    def test_a_failed_build_leaves_the_robot_api_unknown_and_the_next_command_points_it_back(self):
        robot2 = self.panel.robot
        self.panel.broken.add('mobipick3')
        self.panel._switching_to = 'mobipick3'
        self.panel._build_robot('mobipick3')
        self.assertIsNone(self.panel.built[0][1], 'the failed build must not become the chosen robot')
        self.assertEqual(self.panel.wrapper.namespace, '/mobipick3', 'robot_api re-pointed before failing')
        self.assertIsNone(self.panel._robot_api.ns)
        self.assertEqual(self.command(self.panel, 'mobipick2', robot2), 'done')
        self.assertEqual(self.panel.moves, [('/mobipick2/moveit_macros', 'home')], 'went to the half-built robot')
        self.assertFalse(self.panel._robot_api.lock.locked())

    def test_a_command_whose_robot_cannot_be_built_again_does_not_run(self):
        robot2 = self.panel.robot
        self.panel.broken.update({'mobipick2', 'mobipick3'})
        self.panel._switching_to = 'mobipick3'
        self.panel._build_robot('mobipick3')
        self.assertIsNone(self.command(self.panel, 'mobipick2', robot2))
        self.assertEqual(self.panel.moves, [])
        self.assertIsNone(self.panel._robot_api.ns)
        self.assertFalse(self.panel._robot_api.lock.locked())

    def test_two_panels_of_one_process_share_the_robot_api(self):
        a = FakePanel('mobipick')
        b = FakePanel('mobipick2', wrapper=a.wrapper, owner=a._robot_api, moves=a.moves)
        self.assertEqual(a.wrapper.namespace, '/mobipick2', 'panel b was built last')
        robot1 = a.robot
        run_a = threading.Thread(target=self.command, args=(a, 'mobipick', robot1, 'observe100cm_right'), daemon=True)
        run_a.start()  # points the wrapper back to robot 1, then its arm client waits for its server
        self.assertTrue(a.wrapper.connecting.wait(5))
        b._switching_to = 'mobipick3'
        switch_b = threading.Thread(target=b._build_robot, args=('mobipick3',), daemon=True)
        switch_b.start()
        self.assertTrue(wait_until(lambda: b.hints), 'panel b built while panel a\'s command ran')
        self.assertEqual(b.hints, [('mobipick3', 'waiting: robot mobipick: arm to observe100cm_right')])
        a.wrapper.server_answers.set()
        run_a.join(5)
        switch_b.join(5)
        self.assertEqual(a.moves, [('/mobipick/moveit_macros', 'observe100cm_right')])
        self.assertEqual(a._robot_api.ns, 'mobipick3')
        self.assertEqual(self.command(a, 'mobipick', robot1), 'done')
        self.assertEqual(a.moves[-1], ('/mobipick/moveit_macros', 'home'))


if __name__ == '__main__':
    unittest.main()
