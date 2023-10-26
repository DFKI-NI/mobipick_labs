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
# Authors: Alexander Sung, Sebastian Stock

"""Pick and Place application in the Mobipick domain"""


import rosparam
from unified_planning.model import Object, Problem
from unified_planning.shortcuts import Not
from tables_demo_planning.mobipick_components import APIRobot, ArmPose, EnvironmentRepresentation, Item
from tables_demo_planning.demo_domain import Domain


class PickAndPlaceRobot(APIRobot):
    def __init__(self, namespace: str, env: 'PickAndPlaceEnv') -> None:
        super().__init__(namespace)
        self.env = env

    def get_item(self) -> Item:
        """Return power_drill if robot arm has an object attached, else Item.NOTHING."""
        self.arm.execute("HasAttachedObjects")
        return Item.get("power_drill") if self.arm.get_result().result else Item.NOTHING

    def pick_power_drill(self) -> bool:
        """Pick up power drill."""
        self.arm.execute("CaptureObject")
        self.arm_pose = self.get_arm_pose()
        self.arm.execute("PickUpObject")
        self.arm_pose = self.get_arm_pose()
        if not self.arm.get_result().result:
            return False

        self.item = Item.get("power_drill")
        return True

    def place_power_drill(self) -> bool:
        """Place power drill onto table."""
        self.arm.execute("PlaceObject")
        self.arm_pose = ArmPose.place
        self.item = Item.NOTHING
        return True

    def hand_over(self) -> bool:
        """Hand over item and observe force torque feedback."""
        self.arm.execute("MoveArmToHandover")
        self.arm_pose = self.get_arm_pose()
        self.env.item_offered = True
        if not self.arm.observe_force_torque(5.0, 25.0):
            return False

        self.arm.execute("ReleaseGripper")
        return True


class PickAndPlaceEnv(EnvironmentRepresentation[PickAndPlaceRobot]):
    def __init__(self) -> None:
        super().__init__(PickAndPlaceRobot("mobipick", self))
        self.item_offered = False

    def get_item_offered(self) -> bool:
        """Return fluent value whether item has been offered."""
        return self.item_offered


class PickAndPlaceDomain(Domain[PickAndPlaceEnv]):
    def __init__(self) -> None:
        super().__init__(PickAndPlaceEnv())
        # Read pose parameters from rosparam server.
        rosparam_namespace = "/mobipick/tables_demo_planning"
        params = rosparam.list_params(rosparam_namespace)
        self.base_handover_pose_name = (
            rosparam.get_param(rosparam_namespace + "/base_handover_pose_name")
            if rosparam_namespace + "/base_handover_pose_name" in params
            else self.BASE_HANDOVER_POSE_NAME
        )
        self.base_home_pose_name = (
            rosparam.get_param(rosparam_namespace + "/base_home_pose_name")
            if rosparam_namespace + "/base_home_pose_name" in params
            else self.BASE_HOME_POSE_NAME
        )
        self.base_pick_pose_name = rosparam.get_param(rosparam_namespace + "/base_pick_pose_name")
        self.base_place_pose_name = rosparam.get_param(rosparam_namespace + "/base_place_pose_name")

        # Define scenario poses and waypoints.
        self.scenario_pose_names = (
            self.base_handover_pose_name,
            self.base_home_pose_name,
            self.base_pick_pose_name,
            self.base_place_pose_name,
        )
        self.base_pick_pose = self.objects[self.base_pick_pose_name]
        self.base_place_pose = self.objects[self.base_place_pose_name]
        self.env.robot.add_waypoints(
            {pose_name: pose for pose_name, pose in self.api_poses.items() if pose_name in self.scenario_pose_names}
        )
        self.env.robot.initialize(self.api_poses[self.BASE_HOME_POSE_NAME], *self.env.robot.get())
        self.set_fluent_functions([self.get_robot_at])
        self.item_offered = self.create_fluent_from_function(self.env.get_item_offered)

        # Create additional actions.
        self.set_api_actions(
            (PickAndPlaceRobot.move_base, PickAndPlaceRobot.move_base_with_item, PickAndPlaceRobot.move_arm)
        )
        self.pick, (_,) = self.create_action_from_function(PickAndPlaceRobot.pick_power_drill)
        self.pick.add_precondition(self.robot_at(self.base_pick_pose))
        self.pick.add_precondition(self.robot_has(self.nothing))
        for arm_pose in self.arm_poses:
            self.pick.add_effect(self.robot_arm_at(arm_pose), arm_pose == self.arm_pose_unknown)
        self.pick.add_effect(self.robot_has(self.nothing), False)
        self.pick.add_effect(self.robot_has(self.power_drill), True)
        self.place, (_,) = self.create_action_from_function(PickAndPlaceRobot.place_power_drill)
        self.place.add_precondition(self.robot_at(self.base_place_pose))
        self.place.add_precondition(self.robot_has(self.power_drill))
        for arm_pose in self.arm_poses:
            self.place.add_effect(self.robot_arm_at(arm_pose), arm_pose == self.arm_pose_unknown)
        self.place.add_effect(self.robot_has(self.power_drill), False)
        self.place.add_effect(self.robot_has(self.nothing), True)
        self.hand_over, (_,) = self.create_action_from_function(PickAndPlaceRobot.hand_over)
        self.hand_over.add_precondition(self.robot_at(self.base_handover_pose))
        self.hand_over.add_precondition(Not(self.robot_has(self.nothing)))
        for arm_pose in self.arm_poses:
            self.hand_over.add_effect(self.robot_arm_at(arm_pose), arm_pose == self.arm_pose_handover)
        for item in self.items:
            self.hand_over.add_effect(self.robot_has(item), item == self.nothing)
        self.hand_over.add_effect(self.item_offered, True)

        # Create additional visualization labels.
        self.method_labels.update(
            {
                self.pick: lambda _: "Pick up power drill",
                self.place: lambda _: "Place power drill on table",
                self.hand_over: lambda _: "Hand over power drill to person",
            }
        )
        self.parameter_labels.update(
            {
                self.base_pick_pose: "pick",
                self.base_place_pose: "place",
            }
        )

    def get_robot_at(self, pose: Object) -> bool:
        base_pose_name = self.env.robot.base.get_pose_name()
        base_pose = self.objects.get(base_pose_name, self.unknown_pose)
        return pose == base_pose

    def initialize_problem(self) -> Problem:
        """Initialize current problem."""
        actions = [self.move_base, self.move_base_with_item, self.move_arm, self.pick, self.place]
        if not self.env.item_offered:
            actions.append(self.hand_over)
        return self.define_mobipick_problem(
            actions=actions,
            poses=[self.objects[pose_name] for pose_name in self.scenario_pose_names],
            items=[self.power_drill],
            locations=[],
        )

    def set_goals(self, problem: Problem) -> None:
        """Set the goals of the pick and place demo."""
        problem.add_goal(self.robot_at(self.base_home_pose))
        problem.add_goal(self.robot_has(self.nothing))
        problem.add_goal(self.item_offered)
