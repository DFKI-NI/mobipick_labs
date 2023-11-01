# Software License Agreement (BSD License)
#
#  Copyright (c) 2022, DFKI GmbH
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#   * Neither the name of Willow Garage nor the names of its
#     contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.
#
#
# Authors: Alexander Sung, Oscar Lima, Marc Vinci, Sebastian Stock

"""These Mobipick components implement functionalities in the ROS application domain."""


from typing import Dict
import rospy
import actionlib
from std_msgs.msg import String
from std_srvs.srv import Empty, SetBool
from geometry_msgs.msg import Pose
from grasplan.msg import (
    InsertObjectAction,
    InsertObjectGoal,
    PickObjectAction,
    PickObjectGoal,
    PlaceObjectAction,
    PlaceObjectGoal,
)
from unified_planning.model import Object
from symbolic_fact_generation.on_fact_generator import OnGenerator
from symbolic_fact_generation.robot_facts_generator import RobotAtGenerator
from tables_demo_planning.mobipick_components import APIRobot, ArmPose, Item, Location
from tables_demo_planning.subplan_visualization import SubPlanVisualization
from tables_demo_planning.tables_demo import TablesDemoDomain, TablesDemoEnv, TablesDemoRobot
from pose_selector.srv import PoseDelete, PoseDeleteRequest


class TablesDemoAPIRobot(TablesDemoRobot['TablesDemoAPIEnv'], APIRobot):
    def __init__(self, namespace: str, env: 'TablesDemoAPIEnv') -> None:
        APIRobot.__init__(self, namespace)
        TablesDemoRobot.__init__(self, env)

        self.pose_selector_activate = rospy.ServiceProxy("/pick_pose_selector_node/pose_selector_activate", SetBool)
        self.pose_selector_delete = rospy.ServiceProxy("/pick_pose_selector_node/pose_selector_delete", PoseDelete)
        self.open_gripper = rospy.ServiceProxy("/mobipick/pose_teacher/open_gripper", Empty)
        self.pick_object_action_client = actionlib.SimpleActionClient("/mobipick/pick_object", PickObjectAction)
        self.pick_object_goal = PickObjectGoal()
        self.place_object_action_client = actionlib.SimpleActionClient("/mobipick/place_object", PlaceObjectAction)
        self.place_object_goal = PlaceObjectGoal()
        self.insert_object_action_client = actionlib.SimpleActionClient("/mobipick/insert_object", InsertObjectAction)
        self.insert_object_goal = InsertObjectGoal()

        self.on_fact_generator = OnGenerator(
            fact_name='on',
            objects_of_interest=[name for name in TablesDemoAPIDomain.DEMO_ITEMS.keys()],
            container_objects=['klt'],
            query_srv_str="/pick_pose_selector_node/pose_selector_class_query",
            planning_scene_param="/mobipick/pick_object_node/planning_scene_boxes",
        )
        # Clear all demo items' poses already on the pose_selector from previous observations.
        for name in TablesDemoAPIDomain.DEMO_ITEMS.keys():
            class_id, instance_id = name.rsplit("_", 1)
            self.pose_selector_delete(PoseDeleteRequest(class_id=class_id, instance_id=int(instance_id)))

    def pick_item(self, pose: Pose, location: Location, item: Item) -> bool:
        """At pose, look for item at location, pick it up, then move arm to transport pose."""
        rospy.loginfo("Waiting for pick object action server.")
        if not self.pick_object_action_client.wait_for_server(timeout=rospy.Duration(2.0)):
            rospy.logerr("Pick Object action server not available!")
            return False

        rospy.loginfo("Found pick object action server.")
        location = self.env.resolve_search_location(location)
        perceived_item_locations = self.perceive(location)
        if item not in perceived_item_locations.keys():
            rospy.logwarn(f"Cannot find {item.name} at {location.name}. Pick up FAILED!")
            return False

        if perceived_item_locations[item] != location:
            rospy.logwarn(f"Found {item.name} but not on {location.name}. Pick up FAILED!")
            return False

        self.pick_object_goal.object_name = item.name
        self.pick_object_goal.support_surface_name = location.name
        self.pick_object_goal.ignore_object_list = [
            item.name
            for item in TablesDemoDomain.DEMO_ITEMS.values()
            if self.env.believed_item_locations.get(item) == Location.in_box
        ]
        rospy.loginfo(f"Sending pick '{item.name}' goal to pick object action server: {self.pick_object_goal}")
        self.pick_object_action_client.send_goal(self.pick_object_goal)
        rospy.loginfo("Wait for result from pick object action server.")
        if not self.pick_object_action_client.wait_for_result(timeout=rospy.Duration(50.0)):
            rospy.logwarn(f"Pick up {item.name} at {location.name} FAILED due to timeout!")
            return False

        result = self.pick_object_action_client.get_result()
        rospy.loginfo(f"The pick object server is done with execution, resuĺt was: '{result}'")
        if not result or not result.success:
            rospy.logwarn(f"Pick up {item.name} at {location.name} FAILED!")
            return False

        TablesDemoRobot.pick_item(self, pose, location, item)
        self.arm.move("transport")
        return True

    def place_item(self, pose: Pose, location: Location, item: Item) -> bool:
        """At pose, place item at location, then move arm to home pose."""
        rospy.loginfo("Waiting for place object action server.")
        if not self.place_object_action_client.wait_for_server(timeout=rospy.Duration(10.0)):
            rospy.logerr("Place Object action server not available!")
            return False

        rospy.loginfo("Found place object action server.")
        observe_before_place = (
            not item.name.startswith("klt_")
            or self.env.believed_item_locations.get(item) != Location.on_robot
            or all(
                self.env.believed_item_locations.get(check_item) != Location.in_box
                for check_item in TablesDemoDomain.DEMO_ITEMS.values()
            )
        )
        if observe_before_place:
            self.perceive(location)
        self.place_object_goal.support_surface_name = location.name
        self.place_object_goal.observe_before_place = observe_before_place
        rospy.loginfo(f"Sending place '{item.name}' goal to place object action server: {self.place_object_goal}")
        self.place_object_action_client.send_goal(self.place_object_goal)
        rospy.loginfo("Wait for result from place object action server.")
        if not self.place_object_action_client.wait_for_result(timeout=rospy.Duration(50.0)):
            rospy.logwarn(f"Place {item.name} at {location.name} FAILED due to timeout!")
            return False

        result = self.place_object_action_client.get_result()
        rospy.loginfo(f"The place object server is done with execution, result was: '{result}'")
        if not result or not result.success:
            rospy.logwarn(f"Place {item.name} at {location.name} FAILED!")
            return False

        TablesDemoRobot.place_item(self, pose, location, item)
        self.arm.move("home")
        return True

    def store_item(self, pose: Pose, location: Location, item: Item, klt: Item) -> bool:
        """At pose, store item into box at location, then move arm to home pose."""
        rospy.loginfo("Waiting for insert object action server.")
        if not self.insert_object_action_client.wait_for_server(timeout=rospy.Duration(10.0)):
            rospy.logerr("Insert Object action server not available!")
            return False

        rospy.loginfo("Found insert object action server.")
        self.perceive(location)
        assert klt.name.startswith("klt_")
        self.insert_object_goal.support_surface_name = klt.name
        self.insert_object_goal.observe_before_insert = True
        rospy.loginfo(f"Sending insert '{item.name}' goal to insert object action server: {self.insert_object_goal}")
        self.insert_object_action_client.send_goal(self.insert_object_goal)
        rospy.loginfo("Wait for result from insert object action server.")
        if not self.insert_object_action_client.wait_for_result(timeout=rospy.Duration(50.0)):
            rospy.logwarn(f"Insert {item.name} into box at {location.name} FAILED due to timeout!")
            return False

        result = self.insert_object_action_client.get_result()
        rospy.loginfo(f"The insert object server is done with execution, result was: '{result}'")
        if not result or not result.success:
            rospy.logwarn(f"Insert {item.name} into box at {location.name} FAILED!")
            return False

        TablesDemoRobot.store_item(self, pose, location, item, klt)
        self.arm.move("home")
        return True

    def hand_over_item(self, pose: Pose, item: Item) -> bool:
        """
        At pose, hand over item to person, observe force torque feedback
        until item is taken by person, then move arm to home pose.
        """
        self.arm.move("handover")
        self.arm_pose = ArmPose.handover
        # observe force torque sensor using treshold 5.0 and a timeout of 25.0 seconds
        if not self.arm.observe_force_torque(5.0, 25.0):
            return False

        self.arm.execute("ReleaseGripper")
        TablesDemoRobot.hand_over_item(self, pose, item)
        return True

    def perceive(self, location: Location) -> Dict[Item, Location]:
        """Move arm into observation pose and return all perceived items with their locations."""
        self.arm.move("observe100cm_right")
        self.arm_pose = ArmPose.observe
        rospy.loginfo("Wait for pose selector service ...")
        rospy.wait_for_service("/pick_pose_selector_node/pose_selector_activate", timeout=rospy.Duration(2.0))
        rospy.loginfo(f"Clear facts for {location.name}.")
        for believed_item, believed_location in self.env.believed_item_locations.items():
            if believed_location == location:
                class_id, instance_id = believed_item.name.rsplit("_", 1)
                self.pose_selector_delete(PoseDeleteRequest(class_id=class_id, instance_id=int(instance_id)))
        self.pose_selector_activate(True)
        rospy.sleep(5)
        rospy.loginfo("Get facts from fact generator.")
        facts = self.on_fact_generator.generate_facts()
        self.pose_selector_activate(False)
        perceived_item_locations: Dict[Item, Location] = {}
        # Perceive facts for items on table location.
        for fact in facts:
            if fact.name == "on":
                fact_item_name, fact_location_name = fact.values
                rospy.loginfo(f"{fact_item_name} on {fact_location_name} returned by pose_selector and fact_generator.")
                if (
                    fact_item_name in [name for name in TablesDemoDomain.DEMO_ITEMS.keys()]
                    and fact_location_name == location.name
                ):
                    rospy.loginfo(f"{fact_item_name} is perceived as on {fact_location_name}.")
                    perceived_item_locations[Item.get(fact_item_name)] = location
                    # Remove item from offered_items dict to be able to repeat the handover
                    # action with that item, if it is perceived again in the scene.
                    self.env.offered_items.discard(Item.get(fact_item_name))
        # Also perceive facts for items in box if it is perceived on table location.
        for fact in facts:
            if fact.name == "in":
                fact_item_name, fact_location_name = fact.values
                if (
                    fact_item_name in [name for name in TablesDemoDomain.DEMO_ITEMS.keys()]
                    and fact_location_name.startswith("klt_")
                    and perceived_item_locations.get(Item.get(fact_location_name)) == location
                ):
                    rospy.loginfo(f"{fact_item_name} is perceived as on {fact_location_name}.")
                    perceived_item_locations[Item.get(fact_item_name)] = Location.in_box
                    # Remove item from offered_items dict to be able to repeat the handover
                    # action with that item, if it is perceived again in the scene.
                    self.env.offered_items.discard(Item.get(fact_item_name))
                    klt = Item.get(fact_location_name)
                    if klt not in self.env.believe_klt_contains:
                        self.env.believe_klt_contains[klt] = set()
                    self.env.believe_klt_contains[klt].add(Item.get(fact_item_name))
        # Determine newly perceived items and their locations.
        self.env.newly_perceived_item_locations.clear()
        for perceived_item, perceived_location in perceived_item_locations.items():
            if (
                perceived_item not in self.env.believed_item_locations.keys()
                or self.env.believed_item_locations[perceived_item] != location
            ):
                self.env.newly_perceived_item_locations[perceived_item] = perceived_location
        rospy.loginfo(f"Newly perceived items: {[str(i) for i in self.env.newly_perceived_item_locations.keys()]}")
        # Remove all previously perceived items at location.
        for check_item, check_location in list(self.env.believed_item_locations.items()):
            if check_location == location:
                del self.env.believed_item_locations[check_item]
        # Add all currently perceived items at location.
        self.env.believed_item_locations.update(perceived_item_locations)
        self.env.searched_locations.add(location)
        self.env.print_believed_item_locations()
        return perceived_item_locations


class TablesDemoAPIEnv(TablesDemoEnv[TablesDemoAPIRobot]):
    def __init__(self) -> None:
        super().__init__(TablesDemoAPIRobot("mobipick", self))


class TablesDemoAPIDomain(TablesDemoDomain[TablesDemoAPIEnv]):
    def __init__(self) -> None:
        super().__init__(TablesDemoAPIEnv())
        self.env.robot.add_waypoints(self.api_poses)
        self.env.robot.initialize(self.api_poses[self.BASE_HOME_POSE_NAME], *self.env.robot.get())
        self.set_fluent_functions((self.get_robot_at,))
        self.set_api_actions(
            (
                TablesDemoAPIRobot.move_base,
                TablesDemoAPIRobot.move_base_with_item,
                TablesDemoAPIRobot.move_arm,
                TablesDemoAPIRobot.pick_item,
                TablesDemoAPIRobot.place_item,
                TablesDemoAPIRobot.hand_over_item,
                TablesDemoAPIRobot.store_item,
            )
        )

        self.visualization = SubPlanVisualization()
        self.espeak_pub = rospy.Publisher("/espeak_node/speak_line", String, queue_size=1)

        self.robot_at_fact_generator = RobotAtGenerator(
            fact_name='robot_at',
            global_frame='/map',
            robot_frame='/mobipick/base_link',
            undefined_pose_name="unknown_pose",
        )

    def get_robot_at(self, pose: Object) -> bool:
        base_pose_name = None
        robot_at_facts = self.robot_at_fact_generator.generate_facts()
        if robot_at_facts:
            base_pose_name = robot_at_facts[0].values[0]
        base_pose = self.objects.get(base_pose_name, self.unknown_pose)
        return pose == base_pose
