import importlib
import os
import rospy
import rosgraph
import rospkg
import tf
import actionlib
import threading

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QComboBox, QHBoxLayout, QLabel
from PyQt5.QtGui import QPixmap
from PyQt5.QtCore import Qt, QMetaObject, Q_ARG, QTimer, pyqtSignal

from grasplan.tools.common import objectToPick
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from std_srvs.srv import Empty, SetBool, Trigger
from pose_selector.srv import ClassQuery, GetPoses
from grasplan.msg import PickObjectAction, PickObjectGoal, PlaceObjectAction
from grasplan.msg import PlaceObjectGoal, InsertObjectAction, InsertObjectGoal

import robot_api
from std_msgs.msg import String

from rqt_tables_demo import pick_list, robot_names

try:
    from mobipick_gazebo import robot_colours  # the colour names and dots of the sim window's robot selector
except ImportError:
    robot_colours = None

try:
    from mobipick_api.perception import Perception as ApiPerception
except ImportError:  # mobipick_api is optional; the open-set, perceive and inspect group boxes need it
    ApiPerception = None


def service_name(proxy):
    """the ROS name a service client calls, for the log: which robot's pose selector a call really reached"""
    return getattr(proxy, 'resolved_name', None) or '(none)'


class RqtTablesDemo(Plugin):
    robot_selected = pyqtSignal(str)  # the robot named on /selected_robot (the subscriber thread -> the Qt thread)
    robot_built = pyqtSignal(object)  # the clients of a newly chosen robot are ready (worker thread -> Qt thread)
    busy_changed = pyqtSignal()  # a robot command started or ended (worker thread -> Qt thread)
    robots_listed = pyqtSignal(object)  # the robots with a running move_base (worker thread -> Qt thread)

    # the group boxes a running command blocks, for its own robot only: the other robots keep theirs
    COMMAND_GROUPS = ('Navigation_groupBox', 'Manipulation_groupBox', 'Perception_groupBox', 'OpenSet_groupBox',
                      'Perceive_groupBox', 'Inspect_groupBox')

    def __init__(self, context):
        super(RqtTablesDemo, self).__init__(context)
        rospy.loginfo('Initializing rqt_tables_demo, have a happy pick, place and move and more!')

        self.setObjectName('RqtTablesDemo')
        # Create QWidget
        self._widget = QWidget()
        # Get path to UI file (xml description of the gui window created with qtcreator)
        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_tables_demo'), 'config', 'rqt_tables_demo.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        self._widget.setObjectName('rqt_tables_demo.ui')
        # Show _widget.windowTitle on left-top of each plugin (when
        # it's set in _widget). This is useful when you open multiple
        # plugins at once. Also if you open multiple instances of your
        # plugin at once, these lines add number to make it easy to
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))

        # ::: class variables
        self.tf_listener = tf.TransformListener()
        # the robot the commands go to: ~robot_namespace, else the namespace the plugin was launched in, 'mobipick'
        # (robot 1) otherwise; changed with the robot selector at the top. The robot API class (module.Class) must offer
        # base.move(x, y, yaw), arm.move(pose_name) and arm.pose_names like robot_api.Robot does.
        self.launch_robot = robot_names.normalize(rospy.get_param('~robot_namespace', rospy.get_namespace()))
        self.robot_ns = self.launch_robot
        # ~use_mobipick_api (feature flag, off by default): pose selector, pick, place and insert go through the
        # Mobipick API (its Perception and Manipulation, which take their names from the robot namespace) instead of
        # this plugin's own service and action clients
        self.use_mobipick_api = rospy.get_param('~use_mobipick_api', False)
        self.robot_api_class = rospy.get_param('~robot_api_class', '') or (
            'mobipick_api.Robot' if self.use_mobipick_api else 'robot_api.Robot')
        self.robot = self.make_robot(self.robot_api_class, self.robot_ns)
        if self.use_mobipick_api and not hasattr(self.robot.arm, 'pick_object'):
            rospy.logwarn(f'{self.robot_api_class} has no pick_object: ~use_mobipick_api is ignored')
            self.use_mobipick_api = False
        # Gazebo model of the launch robot, for teletransportation in simulation (another robot: its namespace)
        self.launch_gazebo_model_name = rospy.get_param('~gazebo_model_name', self.launch_robot)
        # a flag to know if pose selector is available or not
        self.is_pose_selector_available = False
        # mobipick_api perception client: AnyGrasp's open-set DetectObjects action, perceive (served by
        # mobipick_active_perception, closed-set only without it) and InspectObject (testing only)
        self.api_perception = None
        if ApiPerception is not None:
            self.api_perception = ApiPerception(robot_names.prefix(self.robot_ns) + '/', self.robot.arm, None)
        self.is_gripper_srv_available = False
        # every robot command runs in a worker thread, so the window never waits for a robot and the other robots
        # stay commandable; per robot: {group box or command id: what runs}, guarded by the lock
        self._busy = {}
        self._busy_lock = threading.Lock()
        self._action_clients = {}  # robot -> its pick, place or insert action client while one runs (cancel)
        self._api_busy_robots = {}  # robot -> its API robot object while a pick, place or insert through it runs
        self._nav_cancel_clients = {}  # robot -> a move_base client used only to cancel the panel's navigation
        # robot_api keeps one ROS wrapper per process: the robot built last receives every later robot_api call, so a
        # command of several robot_api calls stops when another robot is built in between
        self._api_ns = self.robot_ns
        self._switching_to = None  # the robot whose clients are being built after a selection
        self._build_lock = threading.Lock()

        # make array of checkboxes that represent objects to ignore from planning scene when picking
        self.ignore_from_ps_chks = []
        self.ignore_from_ps_chks.append(self._widget.chkPickIgnore1)
        self.ignore_from_ps_chks.append(self._widget.chkPickIgnore2)
        self.ignore_from_ps_chks.append(self._widget.chkPickIgnore3)
        self.ignore_from_ps_chks.append(self._widget.chkPickIgnore4)
        self.ignore_from_ps_chks.append(self._widget.chkPickIgnore5)
        self.ignore_from_ps_chks.append(self._widget.chkPickIgnore6)
        self.ignore_from_ps_chks.append(self._widget.chkPickIgnore7)
        self.ignore_from_ps_chks.append(self._widget.chkPickIgnore8)

        # ::: parameters

        # navigation and teletransportation base poses in simulation, base waypoints/goals, but also define surfaces
        default_wp_dic = {
            'pick_pose_name': 'base_table_2_pose',  # not used by this node
            'place_pose_name': 'base_table_3_pose',  # not used by this node
            # moelk_tables values by default, order : [[x, y, z], [qx, qy, qz, qw]]
            'poses': {
                'base_handover_pose': [[10.16, 1.76, 0.0], [0.0, 0.0, 0.707, 0.707]],
                'base_home_pose': [[12.0, 2.0, 0.0], [0.0, 0.0, 0.0, 1.0]],
                'base_table_1_pose': [[12.21, 2.1, 0.0], [0.0, 0.0, 0.707, 0.707]],
                'base_table_2_pose': [[11.85, 2.45, 0.0], [0.0, 0.0, 1.0, 0.0]],
                'base_table_3_pose': [[10.25, 2.45, 0.0], [0.0, 0.0, 1.0, 0.0]],
                'handover_planned': True,  # not used by this node
                'world_name': 'moelk',  # not used by this node
            },
        }

        # read poses from parameter server and extract useful information
        self.wp_dic = self.convert_pose_dict(rospy.get_param('~navigation_waypoints', default_wp_dic))

        manipulation_areas = []
        navigation_areas = []
        for key in self.wp_dic:
            # ignore base_handover_pose and base_home_pose from manipulation areas
            if key != 'base_handover_pose' and key != 'base_home_pose':
                manipulation_areas.append(key)
            navigation_areas.append(key)
        self._widget.comboNavigationWaypoints.addItems(navigation_areas)
        self._widget.comboPickSurfaces.addItems(manipulation_areas)
        self._widget.comboPlaceSurfaces.addItems(manipulation_areas)

        # manipulation "predefined arm configurations" or "arm poses": all poses of the arm group, or the
        # ones listed in ~arm_poses
        selectable_arm_poses = rospy.get_param('~arm_poses', self.robot.arm.pose_names)
        unknown_arm_poses = [pose for pose in selectable_arm_poses if pose not in self.robot.arm.pose_names]
        if unknown_arm_poses:
            rospy.logwarn(f'~arm_poses lists poses the arm does not know (not in the SRDF): {unknown_arm_poses}')
        self._widget.comboArmPoses.addItems(selectable_arm_poses)

        # get moveit arm poses from robot api, filter in only the ones containing 'observe'
        default_arm_observation_poses = ['']  # add a first empty element
        for arm_pose in self.robot.arm.pose_names:
            if 'observe' in arm_pose:
                default_arm_observation_poses.append(arm_pose)

        default_arm_observation_poses = rospy.get_param('arm_observation_poses', default_arm_observation_poses)
        self._widget.comboPerceptionObs1.addItems(default_arm_observation_poses[1:])  # skip first (empty) element
        self._widget.comboPerceptionObs2.addItems(default_arm_observation_poses)
        self._widget.comboPerceptionObs3.addItems(default_arm_observation_poses)

        # the amount of time to wait for pose selector services to become available
        self.wait_for_services = rospy.get_param('wait_for_services', 2.0)

        # adding demo objects to pose selector class query and others
        default_objects_of_interest = [
            'multimeter',
            'klt',
            'power_drill_with_grip',
            'relay',
            'screwdriver',
            'hot_glue_gun',
        ]
        self.objects_of_interest = rospy.get_param(
            '~objects_of_interest', default_objects_of_interest
        )  # TODO: test!!!!!!!!!!!!!!!!!!!!
        self._widget.comboPerceptionPSClassQuery.addItems(self.objects_of_interest)
        self._widget.comboPickObj.addItems(self.objects_of_interest)

        # hole_objects: objects where other objects can be inserted, e.g. a box
        self.hole_objects = rospy.get_param('hole_objects', ['klt'])

        # ::: services
        self._apply_services(self._find_services(self.robot_ns, self.robot, self.api_perception))
        self._nav_cancel_client(self.robot_ns)
        self._add_robot_selector()
        self.robot_built.connect(self._robot_built)
        self.busy_changed.connect(self._update_busy_ui)

        # make a connection between the qt objects and this class methods
        self._widget.cmdNavigationGo.clicked.connect(self.navigation_go)
        self._widget.cmdManipulationGo.clicked.connect(self.manipulation_go)
        self._widget.cmdOpenGripper.clicked.connect(self.open_gripper)
        self._widget.cmdCloseGripper.clicked.connect(self.close_gripper)
        self._widget.cmdPerceiveObjs.clicked.connect(self.perceive_objs)
        self._widget.cmdPerceptionPSActivate.clicked.connect(self.perception_ps_activate)
        self._widget.cmdPerceptionPSDeActivate.clicked.connect(self.perception_ps_deactivate)
        self._widget.cmdPerceptionPSClear.clicked.connect(self.perception_ps_clear)
        self._widget.cmdPerceptionPSGetAllObjs.clicked.connect(self.perception_ps_get_all_objs)
        self._widget.cmdPerceptionPSClassQuery.clicked.connect(self.perception_ps_class_query)
        self._widget.cmdOpenSetDetect.clicked.connect(self.open_set_detect)
        self._widget.txtOpenSetPrompt.returnPressed.connect(self.open_set_detect)
        self._widget.cmdPerceive.clicked.connect(self.perceive_targets)
        self._widget.txtPerceiveTargets.returnPressed.connect(self.perceive_targets)
        self._widget.cmdInspect.clicked.connect(self.inspect_object)
        self._widget.txtInspectQueries.returnPressed.connect(self.inspect_object)
        self._widget.cmdPickObj.clicked.connect(self.pick_object)
        self._widget.cmdManipUpdate.clicked.connect(self.manipulation_update)
        self._widget.cmdPlaceObj.clicked.connect(self.place_object)
        self._widget.cmdInsertObj.clicked.connect(self.insert_object)
        self._widget.cmdHandOverObject.clicked.connect(self.hand_over_object)
        self._widget.cmdCancel.clicked.connect(self.cancel)

        self._widget.chkPickEnableId.stateChanged.connect(self.chk_pick_enable_id_changed)

        context.add_widget(self._widget)
        rospy.loginfo('rqt_tables_demo initialization finished')
        # end of constructor

    # ::::::::::::::  class methods

    @staticmethod
    def make_robot(api_class_path, namespace):
        '''instantiate the robot API class given as "module.Class" (e.g. robot_api.Robot) for the namespace'''
        module_name, _, class_name = api_class_path.rpartition('.')
        if not module_name:
            raise ValueError(f'~robot_api_class must be "module.Class", got "{api_class_path}"')
        try:
            api_class = getattr(importlib.import_module(module_name), class_name)
        except (ImportError, AttributeError) as error:
            raise ImportError(f'cannot load the robot API class {api_class_path}: {error}') from error
        rospy.loginfo(f'controlling the robot in namespace /{namespace} with {api_class_path}')
        return api_class(namespace)

    def _find_services(self, namespace, robot, api_perception):
        """the pose selector and gripper service clients of robot `namespace` as a dict for _apply_services; waits for
        the services (up to a few seconds), so after startup it runs in a worker thread, never in the Qt thread"""
        # the names follow the chosen robot; private params override them: ~robots/<robot>/<param> for that robot
        # (as the Mobipick API reads them), else the unscoped ~<param> for the robot the plugin was launched for only
        ps = robot_names.pose_selector_services(namespace)
        gripper = robot_names.gripper_services(namespace)
        own = namespace == self.launch_robot

        def override(param, default):
            scoped = rospy.get_param(f'~robots/{robot_names.normalize(namespace)}/{param}', None)
            if scoped is not None:
                return scoped
            return rospy.get_param(f'~{param}', default) if own else default
        names = {key: override(param, ps[key]) for key, param in (
            ('activate', 'pose_selector_activate_srv_name'), ('class_query', 'pose_selector_class_query_srv_name'),
            ('get_all', 'pose_selector_get_all_poses_srv_name'), ('clear', 'pose_selector_clear_srv_name'))}
        if self.use_mobipick_api and api_perception is not None:
            names = {'activate': api_perception.pose_selector_activate_srv_name,
                     'class_query': api_perception.pose_selector_class_query_srv_name,
                     'get_all': api_perception.pose_selector_get_all_poses_srv_name,
                     'clear': api_perception.pose_selector_clear_srv_name}
        rospy.loginfo(f'waiting for pose selector services: {", ".join(names.values())}')
        found = {'pose_selector': False, 'gripper_srv': False, 'gripper_via_api': False}
        try:
            rospy.wait_for_service(names['activate'], self.wait_for_services)
            for key in ('class_query', 'get_all', 'clear'):
                rospy.wait_for_service(names[key], 0.5)
            found.update(pose_selector=True,
                         activate=rospy.ServiceProxy(names['activate'], SetBool),
                         class_query=rospy.ServiceProxy(names['class_query'], ClassQuery),
                         get_all=rospy.ServiceProxy(names['get_all'], GetPoses),
                         clear=rospy.ServiceProxy(names['clear'], Trigger))
            rospy.loginfo('found pose selector services')
        except Exception:
            rospy.logwarn(f'pose selector of robot {namespace} not available, this functionality will not be available')
        if self.use_mobipick_api and api_perception is not None:
            # one source of the names and the proxies: the API's perception of this robot
            found.update(activate=api_perception.activate_pose_selector_srv,
                         class_query=api_perception.pose_selector_class_query_srv,
                         get_all=api_perception.pose_selector_get_all_poses_srv,
                         clear=api_perception.pose_selector_clear_srv)

        # services to actuate (open/close) gripper
        # without them, ~gripper_via_api lets the buttons use the robot API's gripper.open() and gripper.close()
        if self.use_mobipick_api and hasattr(robot, 'gripper'):
            # the buttons use the chosen robot's API gripper: no service name of this plugin
            found['gripper_via_api'] = True
            return found
        open_gripper_srv_name = rospy.get_param('~open_gripper_srv_name', gripper['open']) if own else gripper['open']
        close_gripper_srv_name = rospy.get_param('~close_gripper_srv_name', gripper['close']) if own \
            else gripper['close']
        try:
            rospy.wait_for_service(open_gripper_srv_name, self.wait_for_services)
            rospy.wait_for_service(close_gripper_srv_name, 0.5)
            found.update(gripper_srv=True, open=rospy.ServiceProxy(open_gripper_srv_name, Empty),
                         close=rospy.ServiceProxy(close_gripper_srv_name, Empty))
            rospy.loginfo('found gripper open/close services')
        except Exception:
            if rospy.get_param('~gripper_via_api', False) and hasattr(robot, 'gripper'):
                found['gripper_via_api'] = True
                rospy.loginfo('no gripper open/close services, the gripper buttons use the robot API instead')
            else:
                rospy.logwarn(
                    'Could not found service to open/close gripper, this functionality will not be available'
                )
        return found

    def _apply_services(self, found):
        """hand the clients _find_services found to the commands and the widgets (Qt thread)"""
        self.is_pose_selector_available = found['pose_selector']
        for attr, key in (('activate_pose_selector_srv', 'activate'), ('pose_selector_class_query_srv', 'class_query'),
                          ('pose_selector_get_all_poses_srv', 'get_all'), ('pose_selector_clear_srv', 'clear')):
            setattr(self, attr, found.get(key))
        self._widget.groupMoveAndActivatePS.setEnabled(found['pose_selector'])
        self._widget.PoseSelector_groupBox.setEnabled(found['pose_selector'])
        self.is_gripper_srv_available = found['gripper_srv']
        self.gripper_via_api = found['gripper_via_api']
        self.open_gripper_srv, self.close_gripper_srv = found.get('open'), found.get('close')
        gripper = found['gripper_srv'] or found['gripper_via_api']
        self._widget.cmdOpenGripper.setEnabled(gripper)
        self._widget.cmdCloseGripper.setEnabled(gripper)

    def _nav_cancel_client(self, namespace):
        """a move_base client of robot `namespace` used only to cancel a navigation of this panel; made when the robot
        is chosen, so its cancel topic is connected by the time Cancel is pressed"""
        if namespace not in self._nav_cancel_clients:
            self._nav_cancel_clients[namespace] = actionlib.SimpleActionClient(
                robot_names.action_server(namespace, 'move_base'), MoveBaseAction)
        return self._nav_cancel_clients[namespace]

    def _add_robot_selector(self):
        """the robot switch: the robots of the sim by their colour names. A choice here asks the sim's selected-robot
        owner node (the only writer of the latched /selected_robot); without one the panel publishes /selected_robot
        itself. The panel changes robot when /selected_robot names one, so the sim window's selector and the other
        panels switch together. The label next to it shows what the panel's commands are busy with."""
        row = QHBoxLayout()
        row.addWidget(QLabel('Robot:'))
        self.dotRobot = QLabel()
        row.addWidget(self.dotRobot)
        self.comboRobot = QComboBox()
        self.comboRobot.setSizeAdjustPolicy(QComboBox.AdjustToContents)
        self.comboRobot.setToolTip('every command of this panel goes to this robot; the sim window and the other '
                                   'panels switch with it')
        self.comboRobot.activated.connect(lambda index: self._choose_robot(self.comboRobot.itemData(index)))
        row.addWidget(self.comboRobot)
        self.labelRobot = QLabel()  # "(connecting)" while the chosen robot's clients are being made
        row.addWidget(self.labelRobot)
        self.labelBusy = QLabel()
        self.labelBusy.setToolTip('the commands this panel runs, per robot; the buttons of a running command are '
                                  'blocked for its robot only')
        row.addWidget(self.labelBusy, 1)
        self._widget.main_left_verticalLayout.insertLayout(1, row)
        self._fill_robots([self.robot_ns])
        self._show_robot_name()
        self.robots_listed.connect(self._fill_robots)
        self._robot_list_timer = QTimer(self._widget)
        self._robot_list_timer.timeout.connect(self._list_robots)
        self._robot_list_timer.start(3000)
        self._list_robots()
        # without the owner node: the /selected_robot publisher is made when a robot is chosen here (a panel that only
        # follows must not latch a choice, nor count as a writer of the topic)
        self._owner_running = False
        self._published = None
        self._selected_robot_pub = None
        self._request_pub = rospy.Publisher(robot_names.REQUEST_TOPIC, String, queue_size=1)
        self.robot_selected.connect(self._heard_robot)
        self._selected_robot_sub = rospy.Subscriber(robot_names.SELECTED_ROBOT_TOPIC, String,
                                                    lambda msg: self.robot_selected.emit(msg.data))

    def _list_robots(self):
        """ask the master for the robots and the owner node in a worker thread (a slow or remote master must not
        freeze the window)"""
        def task():
            try:
                master = rosgraph.Master(rospy.get_name())
                topics = [name for name, _ in master.getPublishedTopics('/')]
                self._owner_running = robot_names.owner_running(master.getSystemState())
            except Exception as error:  # no master answer: keep the list
                rospy.logwarn_throttle(30, f'could not list the robots: {error}')
                return
            self.robots_listed.emit(robot_names.robots_from_topics(topics, self.robot_ns))
        threading.Thread(target=task, daemon=True, name='list robots').start()

    def _fill_robots(self, robots):
        """Qt thread: the switch lists `robots` (plus the chosen one) by colour name; the namespace is the item data"""
        robots = list(robots)
        for robot in (self.robot_ns, self._switching_to):
            if robot and robot not in robots:
                robots.append(robot)
        if robots == [self.comboRobot.itemData(i) for i in range(self.comboRobot.count())]:
            return
        self.comboRobot.clear()
        for namespace in robots:
            self._add_robot_item(namespace)
        self._show_robot_name(self._switching_to, 'connecting' if self._switching_to else '')

    def _add_robot_item(self, namespace):
        name, icon = namespace, None
        if robot_colours is not None:
            name, icon = robot_colours.display_name(namespace), robot_colours.colour_icon(namespace)
        if icon is not None:
            self.comboRobot.addItem(icon, name, namespace)
        else:
            self.comboRobot.addItem(name, namespace)

    def _choose_robot(self, namespace):
        """Qt thread: a robot chosen in this panel's switch: ask the owner node (the panel follows its answer on
        /selected_robot), or without one tell every GUI and make the robot's clients"""
        namespace = robot_names.normalize(namespace)
        if self._owner_running:
            self._drop_own_choice()
            self._request_pub.publish(String(data=namespace))
            self.labelRobot.setText('(asking)')
            return
        self._publish_choice(namespace)
        self._select_robot(namespace)

    def _heard_robot(self, namespace):
        """Qt thread: /selected_robot names a robot (the owner node, the sim window, another panel or this one).
        Without the owner node, once this panel has published, it publishes a choice made elsewhere again, so all
        latched messages name the same robot."""
        namespace = robot_names.normalize(namespace)
        if self._owner_running:
            self._drop_own_choice()
        elif self._published is not None and robot_names.to_follow(self._published, namespace) is not None:
            self._publish_choice(namespace)
        self._select_robot(namespace)
        # a refused request comes back as the robot already chosen: the switch shows that robot again
        self._show_robot_name(self._switching_to, 'connecting' if self._switching_to else '')

    def _publish_choice(self, namespace):
        """Qt thread, without the owner node: latch `namespace` on /selected_robot for every GUI"""
        if self._selected_robot_pub is None:
            self._selected_robot_pub = rospy.Publisher(robot_names.SELECTED_ROBOT_TOPIC, String, queue_size=1,
                                                       latch=True)
        self._published = namespace
        self._selected_robot_pub.publish(String(data=namespace))

    def _drop_own_choice(self):
        """Qt thread: the owner node runs: withdraw this panel's publisher (made before the owner came up), so the
        owner is the only writer and no latched message of this panel contradicts it"""
        if self._selected_robot_pub is not None:
            self._selected_robot_pub.unregister()
            self._selected_robot_pub = None
            self._published = None

    def _show_robot_name(self, namespace=None, note=''):
        """the switch shows the robot by the colour name and dot of the sim window's selector (the bare namespace
        without the mobipick_gazebo package); `note` e.g. "connecting" while its clients are being made"""
        namespace = namespace or self.robot_ns
        if self.comboRobot.findData(namespace) < 0:  # chosen elsewhere before the robot list showed it
            self._add_robot_item(namespace)
        self.comboRobot.setCurrentIndex(self.comboRobot.findData(namespace))
        icon = robot_colours.colour_icon(namespace) if robot_colours is not None else None
        self.labelRobot.setText(f'({note})' if note else '')
        self.dotRobot.setPixmap(icon.pixmap(12, 12) if icon is not None else QPixmap())

    def _select_robot(self, namespace):
        """build every client for the robot `namespace` in a worker thread; the commands keep going to the robot they
        went to until the new one is ready, and the command buttons wait meanwhile"""
        namespace = robot_names.normalize(namespace)
        if namespace == (self._switching_to or self.robot_ns):
            return
        rospy.loginfo(f'commands go to robot {namespace} once its clients are ready (now {self.robot_ns})')
        self._switching_to = namespace
        self._show_robot_name(namespace, 'connecting')
        self._update_busy_ui()
        threading.Thread(target=self._build_robot, args=(namespace,), daemon=True,
                         name=f'select robot {namespace}').start()

    def _build_robot(self, namespace):
        """worker thread: the robot API, perception, services and navigation cancel client of robot `namespace`"""
        with self._build_lock:  # one build at a time, and none for a choice that a newer one replaced
            if namespace != self._switching_to:
                return
            try:  # when this fails, the commands stay with the robot they went to before
                robot = self.make_robot(self.robot_api_class, namespace)
                self._api_ns = namespace
                api_perception = None
                if ApiPerception is not None:
                    api_perception = ApiPerception(robot_names.prefix(namespace) + '/', robot.arm, None)
                found = self._find_services(namespace, robot, api_perception)
                self._nav_cancel_client(namespace)
            except Exception as error:
                rospy.logerr(f'cannot control robot {namespace} ({error}): commands stay with robot {self.robot_ns}')
                robot = api_perception = found = None
            self.robot_built.emit((namespace, robot, api_perception, found))

    def _robot_built(self, built):
        """Qt thread: the commands go to the robot whose clients _build_robot made"""
        namespace, robot, api_perception, found = built
        if namespace != self._switching_to:
            return  # a newer choice is being built
        self._switching_to = None
        if robot is not None:
            rospy.loginfo(f'commands now go to robot {namespace} (was {self.robot_ns})')
            self.robot_ns, self.robot, self.api_perception = namespace, robot, api_perception
            self._apply_services(found)
        self._show_robot_name()
        self._update_busy_ui()

    def _run(self, group, what, task):
        """run task(ctx) for the chosen robot in a worker thread: the window never waits for a robot. `ctx` is a
        snapshot of the chosen robot's clients, so a later robot choice does not redirect the running command. `group`
        (one of COMMAND_GROUPS, or None) is blocked for this robot until the task ends; the other robots keep it."""
        if self._switching_to is not None:
            rospy.logwarn(f'{what}: not started, the clients of robot {self._switching_to} are still being made')
            return
        ns = self.robot_ns
        ctx = {'ns': ns, 'robot': self.robot, 'api_perception': self.api_perception,
               'activate': self.activate_pose_selector_srv if self.is_pose_selector_available else None,
               'open': self.open_gripper_srv if self.is_gripper_srv_available else None,
               'close': self.close_gripper_srv if self.is_gripper_srv_available else None}
        key = group or object()  # a command without a group is shown as busy but blocks nothing
        with self._busy_lock:
            busy = self._busy.setdefault(ns, {})
            if group is not None and group in busy:
                rospy.logwarn(f'{what}: not started, robot {ns} still runs {busy[group]}')
                return
            busy[key] = what
        self._update_busy_ui()

        def worker():
            try:
                task(ctx)
            except Exception as error:  # a dead server or a bad request: say so instead of a silent thread death
                rospy.logerr(f'{what} on robot {ns} failed: {type(error).__name__}: {error}')
            finally:
                with self._busy_lock:
                    busy.pop(key, None)
                self.busy_changed.emit()
        threading.Thread(target=worker, daemon=True, name=f'{ns}: {what}').start()

    def _is_busy(self, ns, what_prefix):
        with self._busy_lock:
            return any(what.startswith(what_prefix) for what in self._busy.get(ns, {}).values())

    def _api_still_targets(self, ctx, what):
        """False (and logged) when robot_api now serves another robot: the next step of a multi-step command would go
        there (robot_api keeps one ROS wrapper per process, the robot built last owns it)"""
        if self._api_ns == ctx['ns']:
            return True
        rospy.logerr(f'{what} on robot {ctx["ns"]}: stopped, robot {self._api_ns} was chosen meanwhile and the '
                     'robot API now acts on it; choose the robot again and repeat')
        return False

    def _update_busy_ui(self):
        """Qt thread: the command group boxes follow the chosen robot's running commands; the label lists them"""
        with self._busy_lock:
            busy = {ns: list(commands.items()) for ns, commands in self._busy.items() if commands}
        mine = busy.get(self.robot_ns, [])
        blocked = {key for key, _ in mine}
        for name in self.COMMAND_GROUPS:
            getattr(self._widget, name).setEnabled(self._switching_to is None and name not in blocked)
        others = [f'{ns}: {", ".join(what for _, what in commands)}' for ns, commands in busy.items()
                  if ns != self.robot_ns]
        text = 'busy: ' + ', '.join(what for _, what in mine) if mine else 'idle'
        self.labelBusy.setText(text + (f'   (other robots: {"; ".join(others)})' if others else ''))

    def convert_pose_dict(self, input_dict):
        '''
        this function converts a base pose dictionary from the format used in the
        planning yaml file to the format used by the rqt tables demo
        '''
        output_dict = {}
        for key, value in input_dict['poses'].items():
            if 'table' in key:
                new_key = key.replace('base_', '').replace('_pose', '')
            else:
                new_key = key

            if isinstance(value, list) and len(value) == 2:
                flattened_value = value[0] + value[1]
                output_dict[new_key] = flattened_value

        return output_dict

    def chk_pick_enable_id_changed(self):
        self.manipulation_update()

    def manipulation_update(self):
        # query pose selector, update labels and combo boxes accordingly
        detected_objects_names = []
        detected_objects = []
        try:
            if self.pose_selector_get_all_poses_srv is None:
                raise rospy.ServiceException('its pose selector services were not found')
            resp = self.pose_selector_get_all_poses_srv()
        except rospy.ServiceException as error:
            message = pick_list.unreachable_hint(self.robot_ns, error)
            rospy.logerr(message)
            self._widget.chkPickEnableId.setToolTip(message)
            return
        for obj in resp.poses.objects:
            object_to_pick = objectToPick()
            object_to_pick.set_object_class(obj.class_id)
            object_to_pick.set_id(obj.instance_id)
            detected_objects.append(object_to_pick)
            detected_objects_names.append(obj.class_id + '_' + str(obj.instance_id))

        for ignore_chk in self.ignore_from_ps_chks:
            ignore_chk.setText('-')

        if not len(detected_objects_names) > 0:
            # the list of another robot or an earlier update must not stay as if it were this robot's
            message = pick_list.empty_hint(self.robot_ns)
            rospy.logwarn(message)
            self._widget.chkPickEnableId.setToolTip(message)
            self._widget.comboPickObj.clear()
            self._widget.comboPickObj.addItems(self.objects_of_interest)
            self._widget.comboInsertHole.clear()
            return
        self._widget.chkPickEnableId.setToolTip('')

        break_count = len(detected_objects_names)
        for i, ignore_chk in enumerate(self.ignore_from_ps_chks):
            ignore_chk.setText(detected_objects_names[i])
            if i >= break_count - 1:
                break

        self._widget.comboPickObj.clear()
        self._widget.comboPickObj.addItems(
            pick_list.pick_items(detected_objects_names, self._widget.chkPickEnableId.isChecked(),
                                 self.objects_of_interest))

        # update insert combo
        self._widget.comboInsertHole.clear()
        for obj in detected_objects:
            if obj.obj_class in self.hole_objects:  # hole_objects: objects where other objects can be inserted
                self._widget.comboInsertHole.addItems([obj.get_object_class_and_id_as_string()])

        rospy.loginfo(f'robot {self.robot_ns}: pick list updated from pose selector '
                      f'{service_name(self.pose_selector_get_all_poses_srv)}: {", ".join(detected_objects_names)}')

    def transform_pose(self, input_pose, target_reference_frame):
        if input_pose is None:
            rospy.logerr('failed to transform pose: input pose cannot be None')
            return None
        # transform to target reference frame
        current_reference_frame = input_pose.header.frame_id
        try:
            now = rospy.Time.now()
            self.tf_listener.waitForTransform(current_reference_frame, target_reference_frame, now, rospy.Duration(1.0))
            (trans, rot) = self.tf_listener.lookupTransform(current_reference_frame, target_reference_frame, now)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn(f'failed to lookup transform from {current_reference_frame} to {target_reference_frame}')
            return None
        return self.tf_listener.transformPose(target_reference_frame, input_pose)

    def set_model_pose(self, model_name, desired_pose):
        # transform to world reference frame
        desired_pose_in_world_frame = self.transform_pose(desired_pose, 'world')
        srv_name = '/gazebo/set_model_state'
        rospy.wait_for_service(srv_name)
        try:
            set_model_state = rospy.ServiceProxy(srv_name, SetModelState)
            request_msg = ModelState()
            request_msg.model_name = model_name
            request_msg.pose = desired_pose_in_world_frame.pose
            request_msg.reference_frame = 'world'

            resp1 = set_model_state(request_msg)
            return resp1.success
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

    def navigation_go(self):
        waypoint_as_text = self._widget.comboNavigationWaypoints.currentText()
        if self._widget.optNavigationNavigate.isChecked():
            self._run('Navigation_groupBox', f'navigate to {waypoint_as_text}',
                      lambda ctx: self._navigate(ctx, waypoint_as_text))
        elif self._widget.optNavigationTeletransport.isChecked():
            self._run('Navigation_groupBox', f'teletransport to {waypoint_as_text}',
                      lambda ctx: self._teletransport(ctx, waypoint_as_text))

    def _navigate(self, ctx, waypoint_as_text):
        rospy.loginfo(f'Navigating robot {ctx["ns"]} to waypoint: {waypoint_as_text}')
        # Move the robot's base using move_base.
        angular_q = self.wp_dic[waypoint_as_text][3:7]
        angular_rpy = list(tf.transformations.euler_from_quaternion(angular_q))
        frame = robot_names.map_frame(ctx['ns'])
        frame_kwargs = {} if frame == 'map' else {'frame_id': frame}  # a robot API's base.move need not take a frame
        base = ctx['robot'].base
        state = base.move(self.wp_dic[waypoint_as_text][0], self.wp_dic[waypoint_as_text][1], angular_rpy[2],
                          **frame_kwargs)
        why = getattr(base, 'last_navigation_message', '') or f'goal state {state}'
        rospy.loginfo(f'robot {ctx["ns"]} navigation to {waypoint_as_text} ended: {why}')

    def _teletransport(self, ctx, waypoint_as_text):
        rospy.loginfo(f'Teletransporting robot {ctx["ns"]} to waypoint: {waypoint_as_text}')
        pose_stamped_msg = PoseStamped()
        pose_stamped_msg.header.frame_id = robot_names.map_frame(ctx['ns'])
        pose_stamped_msg.pose.position.x = self.wp_dic[waypoint_as_text][0]
        pose_stamped_msg.pose.position.y = self.wp_dic[waypoint_as_text][1]
        pose_stamped_msg.pose.position.z = self.wp_dic[waypoint_as_text][2]
        pose_stamped_msg.pose.orientation.x = self.wp_dic[waypoint_as_text][3]
        pose_stamped_msg.pose.orientation.y = self.wp_dic[waypoint_as_text][4]
        pose_stamped_msg.pose.orientation.z = self.wp_dic[waypoint_as_text][5]
        pose_stamped_msg.pose.orientation.w = self.wp_dic[waypoint_as_text][6]
        model = self.launch_gazebo_model_name if ctx['ns'] == self.launch_robot else ctx['ns']
        self.set_model_pose(model, pose_stamped_msg)

    def manipulation_go(self):
        arm_pose = self._widget.comboArmPoses.currentText()

        def task(ctx):
            rospy.loginfo(f'Moving arm of robot {ctx["ns"]} to pose: {arm_pose}')
            ctx['robot'].arm.move(arm_pose)
        self._run('Manipulation_groupBox', f'arm to {arm_pose}', task)

    def perceive_objs(self):
        # read the widgets in the GUI thread; the task runs in the background
        observation_list = [combo.currentText() for combo in (
            self._widget.comboPerceptionObs1, self._widget.comboPerceptionObs2, self._widget.comboPerceptionObs3)
            if combo.currentText() != '']
        self._run('Perception_groupBox', 'perceive', lambda ctx: self._perceive_objs_task(ctx, observation_list))

    def _perceive_objs_task(self, ctx, observation_list):
        ns = ctx['ns']
        if ctx['activate'] is None:
            rospy.logerr(f"robot {ns}: pose selector is not available, can't perceive")
            return
        selector = service_name(ctx['activate'])
        rospy.loginfo(f'perceiving objects with robot {ns}, pose selector {selector}')
        rospy.loginfo(f'robot {ns}: observation_list : {observation_list}')
        # iterate over observation poses and go one at a time
        for observation_pose in observation_list:
            if not self._api_still_targets(ctx, 'perceive'):
                return
            # the robot API serves the robot built last in this process: say which one, a switch may have moved it
            rospy.loginfo(f'robot {ns}: moving arm to {observation_pose} (robot API serves {self._api_ns})')
            # move arm to observation pose
            ctx['robot'].arm.move(observation_pose)
            # activate pose selector
            rospy.loginfo(f'robot {ns}: activating pose selector {selector}')
            resp = ctx['activate'](True)
            rospy.loginfo(f'robot {ns}: pose selector response to activation request: {resp}')
            # wait until pose selector gets updates
            rospy.sleep(3.0)
            # deactivate pose selector detections
            rospy.loginfo(f'robot {ns}: deactivating pose selector {selector}')
            resp = ctx['activate'](False)

    def _pose_selector_call(self, what, call):
        """a pose selector service call of the chosen robot in a worker thread (a selector that does not answer must
        not freeze the window)"""
        if not self.is_pose_selector_available:
            rospy.logerr(f"robot {self.robot_ns}: pose selector is not available, can't {what}")
            return
        proxies = {'activate': self.activate_pose_selector_srv, 'clear': self.pose_selector_clear_srv,
                   'get_all': self.pose_selector_get_all_poses_srv, 'class_query': self.pose_selector_class_query_srv}
        self._run(None, f'pose selector {what}', lambda ctx: call(
            proxies, f'robot {ctx["ns"]}: pose selector {service_name(proxies["activate"]).rsplit("/", 1)[0]}'))

    def perception_ps_activate(self):
        def call(ps, who):
            rospy.loginfo(f'{who}: activating')
            rospy.loginfo(f'{who}: response: {ps["activate"](True)}')
        self._pose_selector_call('activate', call)

    def perception_ps_deactivate(self):
        def call(ps, who):
            rospy.loginfo(f'{who}: deactivating')
            rospy.loginfo(f'{who}: response: {ps["activate"](False)}')
        self._pose_selector_call('deactivate', call)

    def perception_ps_clear(self):
        def call(ps, who):
            rospy.loginfo(f'{who}: clearing')
            rospy.loginfo(f'{who}: response: {ps["clear"]()}')
        self._pose_selector_call('clear', call)

    def perception_ps_get_all_objs(self):
        def call(ps, who):
            rospy.loginfo(f'{who}: getting all objects')
            resp = ps['get_all']()
            if len(resp.poses.objects) > 0:
                for pose_selector_object in resp.poses.objects:
                    rospy.loginfo(f'{who}: {pose_selector_object}')
            else:
                rospy.logwarn(f'{who}: empty')
        self._pose_selector_call('get all objects', call)

    def perception_ps_class_query(self):
        object_class = self._widget.comboPerceptionPSClassQuery.currentText()

        def call(ps, who):
            rospy.loginfo(f'{who}: class query of object: {object_class}')
            resp = ps['class_query'](object_class)
            if len(resp.poses) == 0:
                rospy.logwarn(f'{who}: object of class {object_class} is not in it')
            else:
                rospy.loginfo(f'{who}: found {len(resp.poses)} instances of class {object_class}')
                rospy.loginfo(resp.poses)
        self._pose_selector_call('query class', call)

    def open_set_detect(self):
        # read the widgets in the GUI thread; the task runs in the background
        request = (
            self._widget.txtOpenSetPrompt.text().strip(),
            self._widget.chkOpenSetVLM.isChecked(),
            self._widget.spinOpenSetBoxThreshold.value(),
            self._widget.spinOpenSetAcceptThreshold.value(),
        )
        self._run('OpenSet_groupBox', 'open-set detection', lambda ctx: self._open_set_detect_task(ctx, request))

    def _set_open_set_result(self, text):
        QMetaObject.invokeMethod(self._widget.lblOpenSetResult, "setText", Qt.QueuedConnection, Q_ARG(str, text))

    def _open_set_detect_task(self, ctx, request):
        api_perception = ctx['api_perception']
        if api_perception is None:
            rospy.logerr('mobipick_api is not available, cannot run open-set detection')
            self._set_open_set_result('mobipick_api not available')
            return
        object_name, use_vlm, box_threshold, accept_threshold = request
        if not object_name:
            rospy.logwarn('type an object description first, e.g. "coke can"')
            self._set_open_set_result('type an object description first')
            return
        rospy.loginfo(f'robot {ctx["ns"]}: open-set detection of {object_name!r} (VLM verifier: {use_vlm}, '
                      f'box >= {box_threshold:.2f}, accept >= {accept_threshold:.2f}; 0 = node default)')
        self._set_open_set_result(f'detecting {object_name!r}...')
        result = api_perception.detect_open_set(
            object_name, use_vlm_verifier=use_vlm, box_threshold=box_threshold, accept_threshold=accept_threshold)
        if result is None:
            self._set_open_set_result('detection action unavailable or timed out (is AnyGrasp running?)')
            return
        lines = [result.message]
        for detection in result.detections:
            line = f'{detection.label} {detection.score:.2f} ' + ('accepted' if detection.accepted else 'rejected')
            if detection.verifier_label:
                line += f' | VLM: {detection.verifier_label} {detection.verifier_confidence:.2f}'
            lines.append(line)
        for obj in result.objects:
            p = obj.pose.position
            lines.append(f'pose selector: {obj.class_id}_{obj.instance_id} at ({p.x:.2f}, {p.y:.2f}, {p.z:.2f})')
        rospy.loginfo(f'robot {ctx["ns"]}: ' + '\n'.join(lines))
        self._set_open_set_result('\n'.join(lines))

    @staticmethod
    def _split_list(text):
        return [item.strip() for item in text.split(',') if item.strip()]

    def _set_text(self, widget, text):
        QMetaObject.invokeMethod(widget, "setPlainText", Qt.QueuedConnection, Q_ARG(str, text))

    def perceive_targets(self):
        # read the widgets in the GUI thread; the task runs in the background
        request = (
            self._split_list(self._widget.txtPerceiveTargets.text()),
            self._widget.comboPerceiveConfidence.currentText(),
            self._widget.chkPerceiveVLM.isChecked(),
            self._widget.chkPerceiveAlign.isChecked(),
            self._widget.chkPerceiveDopeOnly.isChecked(),
            self._widget.chkPerceiveGdOnly.isChecked(),
        )
        self._run('Perceive_groupBox', 'perceive targets', lambda ctx: self._perceive_targets_task(ctx, request))

    def _perceive_targets_task(self, ctx, request):
        widget = self._widget.txtPerceiveResult
        api_perception = ctx['api_perception']
        if api_perception is None:
            self._set_text(widget, 'mobipick_api not available')
            return
        targets, confidence, use_vlm, align, dope_only, gd_only = request
        if not targets:
            self._set_text(widget, 'type the targets first, e.g. table_2, multimeter, tennis ball')
            return
        self._set_text(widget, f'perceiving {targets} with {confidence} confidence...')
        rospy.loginfo(f'robot {ctx["ns"]}: perceiving {targets} through '
                      f'{getattr(api_perception, "perceive_action_name", "(unknown action)")}')
        try:
            text = api_perception.perceive(targets, confidence=confidence, use_vlm=use_vlm, align=align,
                                           dope_only=dope_only, gd_only=gd_only)
        except Exception as exc:  # invalid request or a service failure: show it instead of dying
            text = f'failed:\n{type(exc).__name__}: {exc}'
        rospy.loginfo(f'robot {ctx["ns"]}: perceive result:\n{text}')
        self._set_text(widget, text or '-')

    def inspect_object(self):
        # read the widgets in the GUI thread; the task runs in the background
        request = (
            self._split_list(self._widget.txtInspectQueries.text()),
            self._widget.txtInspectTable.text().strip(),
            self._widget.chkInspectVLM.isChecked(),
            self._widget.chkInspectAlign.isChecked(),
        )
        self._run('Inspect_groupBox', 'inspect', lambda ctx: self._inspect_object_task(ctx, request))

    def _inspect_object_task(self, ctx, request):
        widget = self._widget.txtInspectResult
        api_perception = ctx['api_perception']
        if api_perception is None:
            self._set_text(widget, 'mobipick_api not available')
            return
        queries, table, use_vlm, align = request
        if not queries:
            self._set_text(widget, 'type the open-set objects first, e.g. sugar box, coke')
            return
        self._set_text(widget, f'inspecting {queries}...')
        try:
            result = api_perception.inspect_object(
                queries, table=table, align=align, use_vlm=use_vlm,
                feedback_cb=lambda fb: self._set_text(widget, f'inspecting {queries}: {fb.stage}'))
        except Exception as exc:  # e.g. mobipick_active_perception messages missing in this workspace
            self._set_text(widget, f'{type(exc).__name__}: {exc}')
            return
        if result is None:
            self._set_text(widget, 'InspectObject unavailable or timed out (is mobipick_active_perception running?)')
            return
        lines = [result.message]
        for object_id, pose in zip(result.accepted, result.poses):
            p = pose.pose.position
            lines.append(f'{object_id} at map ({p.x:.2f}, {p.y:.2f}, {p.z:.2f})')
        lines += list(result.outcomes)
        if result.alignment_cm:
            lines.append(f'base aligned by {result.alignment_cm:.1f} cm')
        rospy.loginfo(f'robot {ctx["ns"]}: inspect result:\n' + '\n'.join(lines))
        self._set_text(widget, '\n'.join(lines))

    def _timeout(self, widget):
        try:
            return float(widget.toPlainText())
        except ValueError:
            rospy.logerr('Invalid timeout value. Please enter a valid number.')
            return None

    def _run_action(self, ctx, action_name, action_type, goal, timeout, what):
        """send `goal` to the chosen robot's grasplan action server and wait for it in this worker thread; the client is
        kept per robot while it runs, so Cancel stops this robot's goal only"""
        server_name = robot_names.action_server(ctx['ns'], action_name)
        client = actionlib.SimpleActionClient(server_name, action_type)

        # Wait for the action server to be available
        rospy.loginfo(f'Waiting for {server_name} action server...')
        if not client.wait_for_server(timeout=rospy.Duration.from_sec(self.wait_for_services)):
            rospy.logerr(f'Action server {server_name} not available within the timeout period')
            return
        rospy.loginfo(f'Connected to {server_name} action server, sending goal: {what}')
        self._action_clients[ctx['ns']] = client
        try:
            # Send the goal to the action server
            client.send_goal(goal)

            # Wait for result with specified timeout
            rospy.loginfo(f'Waiting for result from {server_name} action server...')
            if client.wait_for_result(rospy.Duration.from_sec(timeout)):
                result = client.get_result()
                rospy.loginfo(f'{server_name} completed execution with result = "{result}"')
                if result is not None and result.success:
                    rospy.loginfo(f'{what}: succeeded')
                else:
                    rospy.logerr(f'{what}: failed ({client.get_goal_status_text() or "no status text"})')
            else:
                # Timeout handling with goal cancellation
                client.cancel_goal()
                rospy.logerr(f'{what}: not done within {timeout:.0f} s, goal cancellation was sent.')
        finally:
            self._action_clients.pop(ctx['ns'], None)

    def pick_object(self):
        # read the widgets in the GUI thread; the task runs in the background
        manually_entered_object = self._widget.txtPickObjectName.text().strip()
        object_to_pick = manually_entered_object or self._widget.comboPickObj.currentText()
        support_surface_name = self._widget.comboPickSurfaces.currentText()
        ignore_list = [chk.text() for chk in self.ignore_from_ps_chks if chk.isChecked() and chk.text() != '-']
        timeout = self._timeout(self._widget.txtPickTimeout)
        if timeout is None:
            return
        what = f'pick {object_to_pick}'

        def task(ctx):
            if self.use_mobipick_api:
                self._manipulate_via_api(ctx, what, lambda robot: robot.arm.pick_object(
                    object_to_pick, support_surface_name, ignore_list, timeout=timeout))
                return
            goal = PickObjectGoal()
            goal.object_name = object_to_pick
            goal.support_surface_name = support_surface_name
            goal.ignore_object_list = ignore_list
            # Log ignored objects if any
            if goal.ignore_object_list:
                rospy.logwarn(f'Ignoring these objects in planning: {goal.ignore_object_list}')
            else:
                rospy.loginfo('All objects will be considered in the planning scene')
            self._run_action(ctx, 'pick_object', PickObjectAction, goal, timeout,
                             f'{what} from {support_surface_name}')
        self._run('Manipulation_groupBox', what, task)

    def _manipulate_via_api(self, ctx, what, call):
        """run a pick, place or insert of the Mobipick API for the robot of `ctx` and log why it failed"""
        # the robot that was chosen when the command started stays its robot: Cancel and the failure text must be those
        # of the running one, whichever robot is chosen meanwhile
        robot = self._api_busy_robots[ctx['ns']] = ctx['robot']
        rospy.loginfo(f'{what}: through the Mobipick API of robot {ctx["ns"]}')
        try:
            if call(robot):
                rospy.loginfo(f'{what}: succeeded')
            else:
                rospy.logerr(f'{what}: failed: {robot.arm.last_manipulation_message}')
        finally:
            self._api_busy_robots.pop(ctx['ns'], None)

    def _gripper(self, what, service_key, command):
        """open or close the chosen robot's gripper in a worker thread: a gripper driver that does not answer must not
        freeze the window"""
        if self.is_gripper_srv_available:
            self._run(None, what, lambda ctx: ctx[service_key]())
        elif self.gripper_via_api:
            def task(ctx):
                gripper = ctx['robot'].gripper
                if command(gripper):
                    rospy.loginfo(f'{what}: done')
                else:
                    rospy.logerr(f'{what}: failed: {gripper.last_gripper_message}')
            self._run(None, what, task)
        else:
            rospy.logerr('gripper service was not available when node started and therefore is unavailable')

    def open_gripper(self):
        # rosservice call /<robot namespace>/pose_teacher/open_gripper
        self._gripper('open gripper', 'open', lambda gripper: gripper.open())

    def close_gripper(self):
        # rosservice call /<robot namespace>/pose_teacher/close_gripper
        self._gripper('close gripper', 'close', lambda gripper: gripper.close())

    def place_object(self):
        # read the widgets in the GUI thread; the task runs in the background
        support_surface_name = self._widget.comboPlaceSurfaces.currentText()
        observe_before_place = self._widget.chkPlaceObjObserveBeforePlacing.isChecked()
        timeout = self._timeout(self._widget.txtPlaceTimeout)
        if timeout is None:
            return

        def task(ctx):
            if self.use_mobipick_api:
                self._manipulate_via_api(ctx, 'place', lambda robot: robot.arm.place_object(
                    support_surface_name, observe_before_place, timeout=timeout))
                return
            goal = PlaceObjectGoal()
            goal.support_surface_name = support_surface_name
            goal.observe_before_place = observe_before_place
            self._run_action(ctx, 'place_object', PlaceObjectAction, goal, timeout, f'place on {support_surface_name}')
        self._run('Manipulation_groupBox', 'place', task)

    def insert_object(self):
        # read the widgets in the GUI thread; the task runs in the background
        support_surface_name = self._widget.comboInsertHole.currentText()
        observe_before_insert = self._widget.chkObserveBeforeInsert.isChecked()
        timeout = self._timeout(self._widget.txtInsertTimeout)
        if timeout is None:
            return

        # Check if support surface is available when observation is not selected
        if not observe_before_insert and not support_surface_name:
            rospy.logerr('Cannot insert, container object has not been perceived.')
            return

        def task(ctx):
            if self.use_mobipick_api:
                self._manipulate_via_api(ctx, 'insert', lambda robot: robot.arm.insert_object(
                    support_surface_name, observe_before_insert, timeout=timeout))
                return
            goal = InsertObjectGoal()
            goal.support_surface_name = support_surface_name
            goal.observe_before_insert = observe_before_insert
            self._run_action(ctx, 'insert_object', InsertObjectAction, goal, timeout,
                             f'insert into {support_surface_name or "the observed container"}')
        self._run('Manipulation_groupBox', 'insert', task)

    def hand_over_object(self):
        self._run('Manipulation_groupBox', 'hand over', self._hand_over_task)

    def _hand_over_task(self, ctx):
        arm = ctx['robot'].arm
        arm.move('handover')
        rospy.loginfo('waiting for user to excert force on object')
        # observe force torque sensor using a treshold and a timeout
        if not self._api_still_targets(ctx, 'hand over'):
            return False
        if not arm.observe_force_torque(5.0, 25.0):
            rospy.logerr('Failed to handover object within 25 seconds')
            if self._api_still_targets(ctx, 'hand over'):
                arm.move('transport')
            return False

        if not self._api_still_targets(ctx, 'hand over'):
            return False
        arm.execute('ReleaseGripper')
        rospy.loginfo('Success at handing over object')
        return True

    def cancel(self):
        """cancel the chosen robot's navigation, pick, place or insert started from this panel; the other robots and
        their commands go on"""
        # TODO: extend the code to handle perception (arm movement) cancellation as well
        ns = self.robot_ns
        cancelled = []
        if self._is_busy(ns, 'navigate'):
            self._nav_cancel_client(ns).cancel_all_goals()
            cancelled.append('navigation')
        robot = self._api_busy_robots.get(ns)
        if robot is not None and robot.arm.cancel_manipulation():
            cancelled.append('the pick, place or insert through the Mobipick API')
        client = self._action_clients.get(ns)
        if client is not None:
            client.cancel_goal()
            cancelled.append(f'the {client.action_client.ns} goal')
        if cancelled:
            rospy.loginfo(f'robot {ns}: sent the request to cancel {" and ".join(cancelled)}')
        else:
            rospy.logerr(f'robot {ns}: nothing to cancel, NOTE: cancel is implemented only for navigation, pick, place '
                         'and insert')
