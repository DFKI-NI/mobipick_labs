import os
import rospy
import rospkg
import tf
import actionlib

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget

from grasplan.common_grasp_tools import objectToPick
from geometry_msgs.msg import PoseStamped
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from std_srvs.srv import Empty, SetBool, Trigger
from pose_selector.srv import ClassQuery, GetPoses
from grasplan.msg import PickObjectAction, PickObjectGoal, PlaceObjectAction
from grasplan.msg import PlaceObjectGoal, InsertObjectAction, InsertObjectGoal

import robot_api


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

        # navigation and teletransportation in simulation, base waypoints/goals, but also define surfaces
        wp_dic = {
            'table_1': [12.21, 2.10, 0.0, 0.707, 0.0, 0.0, 0.707],
            'table_2': [11.85, 2.45, 0.0, 0.0, 0.0, 0.0, 1.0],
            'table_3': [10.25, 2.45, 0.0, 0.0, 0.0, 0.0, 1.0],
        }
        self.wp_dic = rospy.get_param('navigation_waypoints', wp_dic)
        surfaces = []
        for key in self.wp_dic:
            surfaces.append(key)
        self._widget.comboNavigationWaypoints.addItems(surfaces)
        self._widget.comboPickSurfaces.addItems(surfaces)
        self._widget.comboPlaceSurfaces.addItems(surfaces)

        # manipulation "predefined arm configurations" or "arm poses"
        arm_poses = [
            'home',
            'observe100cm_front',
            'observe100cm_left',
            'observe100cm_right',
            'observe100cm_back',
            'transport',
            'place_1',
            'above',
            'tucked',
            'ready_for_packing',
            'untangle_cable_guide_1_right',
            'untangle_cable_guide_2_right',
        ]
        arm_poses = rospy.get_param('arm_poses', arm_poses)
        self._widget.comboArmPoses.addItems(arm_poses)

        # perception observation poses
        arm_observation_poses = [
            '',
            'observe100cm_right',
            'observe100cm_left',
            'observe100cm_front',
            'observe100cm_back',
        ]
        arm_observation_poses = rospy.get_param('arm_observation_poses', arm_observation_poses)
        self._widget.comboPerceptionObs1.addItems(arm_observation_poses[1:])  # skip first element
        self._widget.comboPerceptionObs2.addItems(arm_observation_poses)
        self._widget.comboPerceptionObs3.addItems(arm_observation_poses)

        # the amount of time to wait for pose selector services to become available
        self.wait_for_services = rospy.get_param('wait_for_services', 2.0)

        # adding demo objects to pose selector class query and others
        objects_of_interest = ['multimeter', 'klt', 'power_drill_with_grip', 'relay', 'screwdriver', 'hot_glue_gun']
        self.objects_of_interest = rospy.get_param('objects_of_interest', objects_of_interest)
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
            self._widget.groupPoseSelector.setEnabled(False)
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

        self._widget.chkPickEnableId.stateChanged.connect(self.chk_pick_enable_id_changed)

        context.add_widget(self._widget)
        rospy.loginfo('rqt_tables_demo initialization finished')
        # end of constructor

    # ::::::::::::::  class methods

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

        rospy.loginfo('update succesful!')

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
            rospy.loginfo(f'Navigating robot to waypoint: {waypoint_as_text}')
            # Move the robot's base using move_base.
            angular_q = [
                self.wp_dic[waypoint_as_text][6],
                self.wp_dic[waypoint_as_text][3],
                self.wp_dic[waypoint_as_text][4],
                self.wp_dic[waypoint_as_text][5],
            ]
            angular_rpy = list(tf.transformations.euler_from_quaternion(angular_q))
            self.mobipick.base.move(self.wp_dic[waypoint_as_text][0], self.wp_dic[waypoint_as_text][1], angular_rpy[2])
        elif self._widget.optNavigationTeletransport.isChecked():
            rospy.loginfo(f'Teletransporting robot to waypoint: {waypoint_as_text}')
            pose_stamped_msg = PoseStamped()
            pose_stamped_msg.header.frame_id = 'map'
            pose_stamped_msg.pose.position.x = self.wp_dic[waypoint_as_text][0]
            pose_stamped_msg.pose.position.y = self.wp_dic[waypoint_as_text][1]
            pose_stamped_msg.pose.position.z = self.wp_dic[waypoint_as_text][2]
            pose_stamped_msg.pose.orientation.w = self.wp_dic[waypoint_as_text][3]  # WARNING: quaternion order wxyz!
            pose_stamped_msg.pose.orientation.x = self.wp_dic[waypoint_as_text][4]
            pose_stamped_msg.pose.orientation.y = self.wp_dic[waypoint_as_text][5]
            pose_stamped_msg.pose.orientation.z = self.wp_dic[waypoint_as_text][6]
            self.set_model_pose('mobipick', pose_stamped_msg)

    def manipulation_go(self):
        arm_pose = self._widget.comboArmPoses.currentText()
        rospy.loginfo(f'Moving arm to pose: {arm_pose}')
        self.mobipick.arm.move(arm_pose)

    def perceive_objs(self):
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
            rospy.sleep(1.0)
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
        object_to_pick = self._widget.comboPickObj.currentText()
        support_surface_name = self._widget.comboPickSurfaces.currentText()
        timeout = float(self._widget.txtPickTimeout.toPlainText())
        pick_object_server_name = 'pick_object'
        action_client = actionlib.SimpleActionClient(pick_object_server_name, PickObjectAction)
        rospy.loginfo(f'waiting for {pick_object_server_name} action server')
        if action_client.wait_for_server(timeout=rospy.Duration.from_sec(self.wait_for_services)):
            rospy.loginfo(f'found {pick_object_server_name} action server')
            goal = PickObjectGoal()
            goal.object_name = object_to_pick
            goal.support_surface_name = support_surface_name
            for chk in self.ignore_from_ps_chks:
                if chk.isChecked():
                    if chk.text() != '-':
                        goal.ignore_object_list.append(chk.text())
            rospy.loginfo(
                f'sending -> pick {object_to_pick} from {support_surface_name} <- '
                'goal to {pick_object_server_name} action server'
            )
            if len(goal.ignore_object_list) > 0:
                rospy.logwarn(
                    f'the following objects: {goal.ignore_object_list} will not be added to the planning scene'
                )
            else:
                rospy.loginfo('all objects are taken into account in planning scene')
            action_client.send_goal(goal)
            rospy.loginfo(f'waiting for result from {pick_object_server_name} action server')
            if action_client.wait_for_result(rospy.Duration.from_sec(timeout)):
                result = action_client.get_result()
                rospy.loginfo(f'{pick_object_server_name} is done with execution, resuĺt was = "{result}"')
                if result.success:
                    rospy.loginfo(f'Succesfully picked {object_to_pick}')
                else:
                    rospy.logerr(f'Failed to pick {object_to_pick}')
            else:
                rospy.logerr(f'Failed to pick {object_to_pick}, timeout?')
        else:
            rospy.logerr(f'action server {pick_object_server_name} not available')

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
        support_surface_name = self._widget.comboPlaceSurfaces.currentText()
        timeout = float(self._widget.txtPlaceTimeout.toPlainText())
        place_object_server_name = 'place_object'
        action_client = actionlib.SimpleActionClient(place_object_server_name, PlaceObjectAction)
        rospy.loginfo(f'waiting for {place_object_server_name} action server')
        if action_client.wait_for_server(timeout=rospy.Duration.from_sec(self.wait_for_services)):
            rospy.loginfo(f'found {place_object_server_name} action server')
            goal = PlaceObjectGoal()
            goal.support_surface_name = support_surface_name
            if self._widget.chkPlaceObjObserveBeforePlacing.isChecked():
                goal.observe_before_place = True
            else:
                goal.observe_before_place = False
            rospy.loginfo(f'sending place goal to {place_object_server_name} action server')
            action_client.send_goal(goal)
            rospy.loginfo(f'waiting for result from {place_object_server_name} action server')
            if action_client.wait_for_result(rospy.Duration.from_sec(timeout)):
                result = action_client.get_result()
                rospy.loginfo(f'{place_object_server_name} is done with execution, resuĺt was = "{result}"')
                if result.success:
                    rospy.loginfo('Succesfully placed object')
                else:
                    rospy.logerr('Failed to place object')
            else:
                rospy.logerr('Failed to place object, timeout?')
        else:
            rospy.logerr(f'action server {place_object_server_name} not available')

    def insert_object(self):
        support_surface_name = self._widget.comboInsertHole.currentText()
        if not self._widget.chkObserveBeforeInsert.isChecked():
            if support_surface_name == '':
                rospy.logerr('cannot insert, container object has not being perceived')
                return
        timeout = float(self._widget.txtInsertTimeout.toPlainText())
        insert_object_server_name = 'insert_object'
        action_client = actionlib.SimpleActionClient(insert_object_server_name, InsertObjectAction)
        rospy.loginfo(f'waiting for {insert_object_server_name} action server')
        if action_client.wait_for_server(timeout=rospy.Duration.from_sec(self.wait_for_services)):
            rospy.loginfo(f'found {insert_object_server_name} action server')
            goal = InsertObjectGoal()
            goal.support_surface_name = support_surface_name
            if self._widget.chkObserveBeforeInsert.isChecked():
                goal.observe_before_insert = True
            else:
                goal.observe_before_insert = False
            rospy.loginfo(f'sending insert goal to {insert_object_server_name} action server')
            action_client.send_goal(goal)
            rospy.loginfo(f'waiting for result from {insert_object_server_name} action server')
            if action_client.wait_for_result(rospy.Duration.from_sec(timeout)):
                result = action_client.get_result()
                rospy.loginfo(f'{insert_object_server_name} is done with execution, resuĺt was = "{result}"')
                if result.success:
                    rospy.loginfo('Succesfully inserted object')
                else:
                    rospy.logerr('Failed to insert object')
            else:
                rospy.logerr('Failed to insert object, timeout?')
        else:
            rospy.logerr(f'action server {insert_object_server_name} not available')
