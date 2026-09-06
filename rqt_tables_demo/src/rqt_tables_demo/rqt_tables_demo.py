import os
import rospy
import rospkg
import tf
import actionlib
import threading

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QApplication
from PyQt5.QtCore import Qt, QMetaObject, Q_ARG, QThread

from grasplan.tools.common import objectToPick
from geometry_msgs.msg import PoseStamped
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from std_srvs.srv import Empty, SetBool, Trigger
from pose_selector.srv import ClassQuery, GetPoses
from grasplan.msg import PickObjectAction, PickObjectGoal, PlaceObjectAction
from grasplan.msg import PlaceObjectGoal, InsertObjectAction, InsertObjectGoal

import robot_api

from functools import wraps


def disable_during_execution(groupbox_attr):
    def decorator(func):
        @wraps(func)
        def wrapper(self, *args, **kwargs):
            # Access the QGroupBox from self using groupbox_attr
            groupbox = getattr(self._widget, groupbox_attr)

            # Check if the function is running in the main thread
            in_main_thread = QThread.currentThread() == QApplication.instance().thread()

            if in_main_thread:
                # Directly disable and enable in the main thread
                groupbox.setEnabled(False)
                # Process events to update UI immediately
                QApplication.processEvents()
                try:
                    # Execute the actual function with just `self`, ignoring other args
                    result = func(self)
                finally:
                    # Re-enable the group box after function execution
                    groupbox.setEnabled(True)
                    # Process events again to update UI
                    QApplication.processEvents()
            else:
                # If in a background thread, use invokeMethod to safely modify UI in the main thread
                QMetaObject.invokeMethod(groupbox, "setEnabled", Qt.QueuedConnection, Q_ARG(bool, False))
                try:
                    result = func(self)
                finally:
                    QMetaObject.invokeMethod(groupbox, "setEnabled", Qt.QueuedConnection, Q_ARG(bool, True))

            return result

        return wrapper

    return decorator


class RqtTablesDemo(Plugin):
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
        # Get a Robot object using the robot's namespace.
        self.mobipick = robot_api.Robot("mobipick")
        # a flag to know if pose selector is available or not
        self.is_pose_selector_available = False
        self.is_gripper_srv_available = False
        self.action_client = None

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

        # manipulation "predefined arm configurations" or "arm poses"
        self._widget.comboArmPoses.addItems(self.mobipick.arm.pose_names)

        # get moveit arm poses from robot api, filter in only the ones containing 'observe'
        default_arm_observation_poses = ['']  # add a first empty element
        for arm_pose in self.mobipick.arm.pose_names:
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

        # pose selector service clients
        pose_selector_activate_srv_name = rospy.get_param(
            '~pose_selector_activate_srv_name', '/pick_pose_selector_node/pose_selector_activate'
        )
        pose_selector_class_query_srv_name = rospy.get_param(
            '~pose_selector_class_query_srv_name', '/pick_pose_selector_node/pose_selector_class_query'
        )
        pose_selector_get_all_poses_srv_name = rospy.get_param(
            '~pose_selector_get_all_poses_srv_name', '/pick_pose_selector_node/pose_selector_get_all'
        )
        pose_selector_clear_srv_name = rospy.get_param(
            '~pose_selector_clear_srv_name', '/pick_pose_selector_node/pose_selector_clear'
        )
        rospy.loginfo(
            f'waiting for pose selector services: {pose_selector_activate_srv_name}, '
            '{pose_selector_class_query_srv_name}, '
            '{pose_selector_get_all_poses_srv_name}, '
            '{pose_selector_clear_srv_name}'
        )
        # if wait_for_service fails, it will throw a
        # rospy.exceptions.ROSException, and the node will exit (as long as
        # this happens before moveit_commander.roscpp_initialize()).

        try:
            rospy.wait_for_service(pose_selector_activate_srv_name, self.wait_for_services)
            rospy.wait_for_service(pose_selector_class_query_srv_name, 0.5)
            rospy.wait_for_service(pose_selector_get_all_poses_srv_name, 0.5)
            rospy.wait_for_service(pose_selector_clear_srv_name, 0.5)
            self.activate_pose_selector_srv = rospy.ServiceProxy(pose_selector_activate_srv_name, SetBool)
            self.pose_selector_class_query_srv = rospy.ServiceProxy(pose_selector_class_query_srv_name, ClassQuery)
            self.pose_selector_get_all_poses_srv = rospy.ServiceProxy(pose_selector_get_all_poses_srv_name, GetPoses)
            self.pose_selector_clear_srv = rospy.ServiceProxy(pose_selector_clear_srv_name, Trigger)
            self.is_pose_selector_available = True
            rospy.loginfo('found pose selector services')
        except Exception:
            self.is_pose_selector_available = False
            self._widget.groupMoveAndActivatePS.setEnabled(False)
            self._widget.PoseSelector_groupBox.setEnabled(False)
            rospy.logwarn('pose selector not available, this functionality will not be available')

        # services to actuate (open/close) gripper
        open_gripper_srv_name = rospy.get_param('~open_gripper_srv_name', '/mobipick/pose_teacher/open_gripper')
        close_gripper_srv_name = rospy.get_param('~close_gripper_srv_name', '/mobipick/pose_teacher/close_gripper')
        try:
            rospy.wait_for_service(open_gripper_srv_name, self.wait_for_services)
            rospy.wait_for_service(close_gripper_srv_name, 0.5)
            self.open_gripper_srv = rospy.ServiceProxy(open_gripper_srv_name, Empty)
            self.close_gripper_srv = rospy.ServiceProxy(close_gripper_srv_name, Empty)
            self.is_gripper_srv_available = True
            rospy.loginfo('found gripper open/close services')
        except Exception:
            self.is_gripper_srv_available = False
            self._widget.cmdOpenGripper.setEnabled(False)
            self._widget.cmdCloseGripper.setEnabled(False)
            rospy.logwarn('Could not found service to open/close gripper, this functionality will not be available')

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
        resp = self.pose_selector_get_all_poses_srv()
        for obj in resp.poses.objects:
            object_to_pick = objectToPick()
            object_to_pick.set_object_class(obj.class_id)
            object_to_pick.set_id(obj.instance_id)
            detected_objects.append(object_to_pick)
            detected_objects_names.append(obj.class_id + '_' + str(obj.instance_id))

        for ignore_chk in self.ignore_from_ps_chks:
            ignore_chk.setText('-')

        if not len(detected_objects_names) > 0:
            rospy.logwarn('Pose selector is empty')
            return

        break_count = len(detected_objects_names)
        for i, ignore_chk in enumerate(self.ignore_from_ps_chks):
            ignore_chk.setText(detected_objects_names[i])
            if i >= break_count - 1:
                break

        self._widget.comboPickObj.clear()
        if self._widget.chkPickEnableId.isChecked():
            self._widget.comboPickObj.addItems(detected_objects_names)
        else:
            self._widget.comboPickObj.addItems(self.objects_of_interest)

        # update insert combo
        self._widget.comboInsertHole.clear()
        for obj in detected_objects:
            if obj.obj_class in self.hole_objects:  # hole_objects: objects where other objects can be inserted
                self._widget.comboInsertHole.addItems([obj.get_object_class_and_id_as_string()])

        rospy.loginfo('update successful!')

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

    @disable_during_execution('Navigation_groupBox')
    def navigation_go(self):
        waypoint_as_text = self._widget.comboNavigationWaypoints.currentText()
        if self._widget.optNavigationNavigate.isChecked():
            rospy.loginfo(f'Navigating robot to waypoint: {waypoint_as_text}')
            # Move the robot's base using move_base.
            angular_q = self.wp_dic[waypoint_as_text][3:7]
            angular_rpy = list(tf.transformations.euler_from_quaternion(angular_q))
            self.mobipick.base.move(self.wp_dic[waypoint_as_text][0], self.wp_dic[waypoint_as_text][1], angular_rpy[2])
        elif self._widget.optNavigationTeletransport.isChecked():
            rospy.loginfo(f'Teletransporting robot to waypoint: {waypoint_as_text}')
            pose_stamped_msg = PoseStamped()
            pose_stamped_msg.header.frame_id = 'map'
            pose_stamped_msg.pose.position.x = self.wp_dic[waypoint_as_text][0]
            pose_stamped_msg.pose.position.y = self.wp_dic[waypoint_as_text][1]
            pose_stamped_msg.pose.position.z = self.wp_dic[waypoint_as_text][2]
            pose_stamped_msg.pose.orientation.x = self.wp_dic[waypoint_as_text][3]
            pose_stamped_msg.pose.orientation.y = self.wp_dic[waypoint_as_text][4]
            pose_stamped_msg.pose.orientation.z = self.wp_dic[waypoint_as_text][5]
            pose_stamped_msg.pose.orientation.w = self.wp_dic[waypoint_as_text][6]
            self.set_model_pose('mobipick', pose_stamped_msg)

    @disable_during_execution('Manipulation_groupBox')
    def manipulation_go(self):
        arm_pose = self._widget.comboArmPoses.currentText()
        rospy.loginfo(f'Moving arm to pose: {arm_pose}')
        self.mobipick.arm.move(arm_pose)

    def perceive_objs(self):
        self.pick_thread = threading.Thread(target=self._perceive_objs_task)
        self.pick_thread.start()

    @disable_during_execution('Perception_groupBox')
    def _perceive_objs_task(self):
        if not self.is_pose_selector_available:
            rospy.logerr("pose selector is not available, can't perceive")
            return
        rospy.loginfo('perceiving objects')
        observation_list = []
        if self._widget.comboPerceptionObs1.currentText() != '':
            observation_list.append(self._widget.comboPerceptionObs1.currentText())
        if self._widget.comboPerceptionObs2.currentText() != '':
            observation_list.append(self._widget.comboPerceptionObs2.currentText())
        if self._widget.comboPerceptionObs3.currentText() != '':
            observation_list.append(self._widget.comboPerceptionObs3.currentText())
        rospy.loginfo(f'observation_list : {observation_list}')
        # iterate over observation poses and go one at a time
        for observation_pose in observation_list:
            rospy.loginfo(f'moving arm to {observation_pose}')
            # move arm to observation pose
            self.mobipick.arm.move(observation_pose)
            # activate pose selector
            rospy.loginfo('activating pose selector')
            resp = self.activate_pose_selector_srv(True)
            rospy.loginfo(f'pose selector response to activation request: {resp}')
            # wait until pose selector gets updates
            rospy.sleep(3.0)
            # deactivate pose selector detections
            rospy.loginfo('deactivating pose selector')
            resp = self.activate_pose_selector_srv(False)

    def perception_ps_activate(self):
        if not self.is_pose_selector_available:
            rospy.logerr("pose selector is not available, can't activate")
            return
        rospy.loginfo('activating pose selector')
        resp = self.activate_pose_selector_srv(True)
        rospy.loginfo(f'response: {resp}')

    def perception_ps_deactivate(self):
        if not self.is_pose_selector_available:
            rospy.logerr("pose selector is not available, can't deactivate")
            return
        rospy.loginfo('deactivating pose selector')
        resp = self.activate_pose_selector_srv(False)
        rospy.loginfo(f'response: {resp}')

    def perception_ps_clear(self):
        if not self.is_pose_selector_available:
            rospy.logerr("pose selector is not available, can't clear")
            return
        rospy.loginfo('clearing pose selector')
        resp = self.pose_selector_clear_srv()
        rospy.loginfo(f'response: {resp}')

    def perception_ps_get_all_objs(self):
        if not self.is_pose_selector_available:
            rospy.logerr("pose selector is not available, can't get all objects")
            return
        rospy.loginfo('getting all objects from pose selector')
        resp = self.pose_selector_get_all_poses_srv()
        if len(resp.poses.objects) > 0:
            for pose_selector_object in resp.poses.objects:
                rospy.loginfo(pose_selector_object)
        else:
            rospy.logwarn('pose selector is empty')

    def perception_ps_class_query(self):
        if not self.is_pose_selector_available:
            rospy.logerr("pose selector is not available, can't query class")
            return
        object_class = self._widget.comboPerceptionPSClassQuery.currentText()
        rospy.loginfo(f'pose selector class query of object: {object_class}')
        resp = self.pose_selector_class_query_srv(object_class)
        if len(resp.poses) == 0:
            rospy.logwarn(f'object of class {object_class} is not in pose selector')
        else:
            rospy.loginfo(f'found {len(resp.poses)} instances of class {object_class}')
            rospy.loginfo(resp.poses)

    def pick_object(self):
        self.pick_thread = threading.Thread(target=self._pick_object_task)
        self.pick_thread.start()

    @disable_during_execution('Manipulation_groupBox')
    def _pick_object_task(self):
        # Retrieve parameters from UI elements
        manually_entered_object = self._widget.txtPickObjectName.text().strip()
        object_to_pick = manually_entered_object or self._widget.comboPickObj.currentText()
        support_surface_name = self._widget.comboPickSurfaces.currentText()
        try:
            timeout = float(self._widget.txtPickTimeout.toPlainText())
        except ValueError:
            rospy.logerr('Invalid timeout value. Please enter a valid number.')
            return

        pick_object_server_name = 'pick_object'
        self.action_client = actionlib.SimpleActionClient(pick_object_server_name, PickObjectAction)

        # Wait for the action server to be available
        rospy.loginfo(f'Waiting for {pick_object_server_name} action server...')
        if self.action_client.wait_for_server(timeout=rospy.Duration.from_sec(self.wait_for_services)):
            rospy.loginfo(f'Connected to {pick_object_server_name} action server')

            # Set up the goal
            goal = PickObjectGoal()
            goal.object_name = object_to_pick
            goal.support_surface_name = support_surface_name
            goal.ignore_object_list = [
                chk.text() for chk in self.ignore_from_ps_chks if chk.isChecked() and chk.text() != '-'
            ]

            rospy.loginfo(
                f'Sending goal: pick {object_to_pick} from {support_surface_name} to {pick_object_server_name}'
            )

            # Log ignored objects if any
            if goal.ignore_object_list:
                rospy.logwarn(f'Ignoring these objects in planning: {goal.ignore_object_list}')
            else:
                rospy.loginfo('All objects will be considered in the planning scene')

            # Send the goal to the action server
            self.action_client.send_goal(goal)

            # Wait for result with specified timeout
            rospy.loginfo(f'Waiting for result from {pick_object_server_name} action server...')
            if self.action_client.wait_for_result(rospy.Duration.from_sec(timeout)):
                result = self.action_client.get_result()
                rospy.loginfo(f'{pick_object_server_name} completed execution with result = "{result}"')
                if result.success:
                    rospy.loginfo(f'Successfully picked {object_to_pick}')
                else:
                    rospy.logerr(f'Failed to pick {object_to_pick}')
            else:
                # Timeout handling with goal cancellation
                self.action_client.cancel_goal()
                rospy.logerr(f'Failed to pick {object_to_pick} within the allocated time. Goal cancellation was sent.')
        else:
            rospy.logerr(f'Action server {pick_object_server_name} not available within the timeout period')
        self.action_client = None

    def open_gripper(self):
        # rosservice call /mobipick/pose_teacher/open_gripper
        if self.is_gripper_srv_available:
            self.open_gripper_srv()
        else:
            rospy.logerr('gripper service was not available when node started and therefore is unavailable')

    def close_gripper(self):
        # rosservice call /mobipick/pose_teacher/close_gripper
        if self.is_gripper_srv_available:
            self.close_gripper_srv()
        else:
            rospy.logerr('gripper service was not available when node started and therefore is unavailable')

    def place_object(self):
        self.pick_thread = threading.Thread(target=self._place_object_task)
        self.pick_thread.start()

    @disable_during_execution('Manipulation_groupBox')
    def _place_object_task(self):
        # Retrieve parameters from UI elements
        support_surface_name = self._widget.comboPlaceSurfaces.currentText()
        try:
            timeout = float(self._widget.txtPlaceTimeout.toPlainText())
        except ValueError:
            rospy.logerr('Invalid timeout value. Please enter a valid number.')
            return

        place_object_server_name = 'place_object'
        self.action_client = actionlib.SimpleActionClient(place_object_server_name, PlaceObjectAction)

        # Wait for the action server to be available
        rospy.loginfo(f'Waiting for {place_object_server_name} action server...')
        if self.action_client.wait_for_server(timeout=rospy.Duration.from_sec(self.wait_for_services)):
            rospy.loginfo(f'Connected to {place_object_server_name} action server')

            # Set up the goal
            goal = PlaceObjectGoal()
            goal.support_surface_name = support_surface_name
            goal.observe_before_place = self._widget.chkPlaceObjObserveBeforePlacing.isChecked()

            rospy.loginfo(f'Sending place goal to {place_object_server_name} action server')

            # Send the goal to the action server
            self.action_client.send_goal(goal)

            # Wait for result with specified timeout
            rospy.loginfo(f'Waiting for result from {place_object_server_name} action server...')
            if self.action_client.wait_for_result(rospy.Duration.from_sec(timeout)):
                result = self.action_client.get_result()
                rospy.loginfo(f'{place_object_server_name} completed execution with result = "{result}"')
                if result.success:
                    rospy.loginfo('Successfully placed object')
                else:
                    rospy.logerr('Failed to place object')
            else:
                # Cancel the goal if timeout occurs
                self.action_client.cancel_goal()
                rospy.logerr('Failed to place object within the allocated time. Goal cancellation was sent.')
        else:
            rospy.logerr(f'Action server {place_object_server_name} not available within the timeout period')

        # Indicate that the place action is no longer running
        self.action_client = None

    def insert_object(self):
        self.pick_thread = threading.Thread(target=self._insert_object_task)
        self.pick_thread.start()

    @disable_during_execution('Manipulation_groupBox')
    def _insert_object_task(self):
        # Retrieve parameters from UI elements
        support_surface_name = self._widget.comboInsertHole.currentText()
        observe_before_insert = self._widget.chkObserveBeforeInsert.isChecked()
        try:
            timeout = float(self._widget.txtInsertTimeout.toPlainText())
        except ValueError:
            rospy.logerr('Invalid timeout value. Please enter a valid number.')
            return

        # Check if support surface is available when observation is not selected
        if not observe_before_insert and not support_surface_name:
            rospy.logerr('Cannot insert, container object has not been perceived.')
            return

        insert_object_server_name = 'insert_object'
        self.action_client = actionlib.SimpleActionClient(insert_object_server_name, InsertObjectAction)

        # Wait for the action server to be available
        rospy.loginfo(f'Waiting for {insert_object_server_name} action server...')
        if self.action_client.wait_for_server(timeout=rospy.Duration.from_sec(self.wait_for_services)):
            rospy.loginfo(f'Connected to {insert_object_server_name} action server')

            # Set up the goal
            goal = InsertObjectGoal()
            goal.support_surface_name = support_surface_name
            goal.observe_before_insert = observe_before_insert
            rospy.loginfo(f'Sending insert goal to {insert_object_server_name} action server')

            # Send the goal to the action server
            self.action_client.send_goal(goal)

            # Wait for result with specified timeout
            rospy.loginfo(f'Waiting for result from {insert_object_server_name} action server...')
            if self.action_client.wait_for_result(rospy.Duration.from_sec(timeout)):
                result = self.action_client.get_result()
                rospy.loginfo(f'{insert_object_server_name} completed execution with result = "{result}"')
                if result.success:
                    rospy.loginfo('Successfully inserted object')
                else:
                    rospy.logerr('Failed to insert object')
            else:
                # Cancel the goal if timeout occurs
                self.action_client.cancel_goal()
                rospy.logerr('Failed to insert object within the allocated time. Goal cancellation was sent.')
        else:
            rospy.logerr(f'Action server {insert_object_server_name} not available within the timeout period')

        # Indicate that the insert action is no longer running
        self.action_client = None

    def hand_over_object(self):
        self.mobipick.arm.move('handover')
        rospy.loginfo('waiting for user to excert force on object')
        # observe force torque sensor using a treshold and a timeout
        if not self.mobipick.arm.observe_force_torque(5.0, 25.0):
            rospy.logerr('Failed to handover object within 25 seconds')
            self.mobipick.arm.move('transport')
            return False

        self.mobipick.arm.execute('ReleaseGripper')
        rospy.loginfo('Success at handing over object')
        return True

    def cancel(self):
        # TODO: extend the code to handle move base and perception (arm movement) cancellation as well
        if not self.action_client:
            rospy.logerr('cannot cancel goal, NOTE: cancel is implemented only for pick, place and insert')
        else:
            action_client_name = self.action_client.action_client.ns
            rospy.loginfo(f'sending request to cancel {action_client_name} goal.')
            self.action_client.cancel_goal()
