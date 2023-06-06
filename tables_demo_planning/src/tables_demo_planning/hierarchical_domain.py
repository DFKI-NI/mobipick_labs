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
# Authors: Marc Vinci, DFKI

"""Addition of hierarchical methods and tasks to the Mobipick domain."""


from typing import Iterable, Optional, Union
from geometry_msgs.msg import Pose
from unified_planning.model import Fluent, InstantaneousAction, Object, Action
from unified_planning.model.htn import HierarchicalProblem, Method, Task, Subtask
from unified_planning.shortcuts import Equals, Not, Or
from unified_planning.model.metrics import MinimizeSequentialPlanLength
from tables_demo_planning.mobipick_components import ArmPose, Item, Location
from tables_demo_planning.tables_demo_api import TablesDemoAPIDomain


class HierarchicalDomain(TablesDemoAPIDomain):
    def __init__(self) -> None:
        super().__init__()

        # TASKS
        self.drive = Task("drive", goal_pose=self.get_type(Pose))
        self.adapt_arm = Task("adapt_arm", to_pose=self.get_type(ArmPose))
        self.perceive = Task("perceive", location=self.get_type(Location))
        self.get_item = Task("get_item", item=self.get_type(Item))
        self.put_item = Task("put_item", item=self.get_type(Item), location=self.get_type(Location))
        self.move_item = Task("move_item", item=self.get_type(Item), location=self.get_type(Location))

        # METHODS

        # DRIVE
        # robot already at goal pose
        self.drive_noop = Method("drive_noop", goal_pose=self.get_type(Pose))
        drive_noop_goal_pose = self.drive_noop.parameter("goal_pose")
        self.drive_noop.set_task(self.drive, drive_noop_goal_pose)
        self.drive_noop.add_precondition(self.robot_at(drive_noop_goal_pose))

        # arm not holding anything, move arm to home pose and move base to goal location
        self.drive_homeposture = Method(
            "drive_homeposture", start_pose=self.get_type(Pose), goal_pose=self.get_type(Pose)
        )
        drive_homeposture_start_pose = self.drive_homeposture.parameter("start_pose")
        drive_homeposture_goal_pose = self.drive_homeposture.parameter("goal_pose")
        self.drive_homeposture.set_task(self.drive, drive_homeposture_goal_pose)
        self.drive_homeposture.add_precondition(self.robot_at(drive_homeposture_start_pose))
        self.drive_homeposture.add_precondition(self.robot_has(self.nothing))
        s1 = self.drive_homeposture.add_subtask(self.adapt_arm, self.arm_pose_home)
        s2 = self.drive_homeposture.add_subtask(
            self.move_base, self.robot, drive_homeposture_start_pose, drive_homeposture_goal_pose
        )
        self.drive_homeposture.set_ordered(s1, s2)

        # arm holding an item, move arm to transport pose and move base to goal location
        self.drive_transport = Method(
            "drive_transport", item=self.get_type(Item), start_pose=self.get_type(Pose), goal_pose=self.get_type(Pose)
        )
        drive_transport_start_pose = self.drive_transport.parameter("start_pose")
        drive_transport_goal_pose = self.drive_transport.parameter("goal_pose")
        drive_transport_item = self.drive_transport.parameter("item")
        self.drive_transport.set_task(self.drive, drive_transport_goal_pose)
        self.drive_transport.add_precondition(self.robot_at(drive_transport_start_pose))
        self.drive_transport.add_precondition(self.robot_has(drive_transport_item))
        s1 = self.drive_transport.add_subtask(self.adapt_arm, self.arm_pose_transport)
        s2 = self.drive_transport.add_subtask(
            self.move_base_with_item,
            self.robot,
            drive_transport_item,
            drive_transport_start_pose,
            drive_transport_goal_pose,
        )
        self.drive_transport.set_ordered(s1, s2)

        # ADAPT ARM
        # arm already in goal arm pose
        self.adapt_arm_noop = Method("adapt_arm_noop", to_pose=self.get_type(ArmPose))
        adapt_arm_noop_to = self.adapt_arm_noop.parameter("to_pose")
        self.adapt_arm_noop.set_task(self.adapt_arm, adapt_arm_noop_to)
        self.adapt_arm_noop.add_precondition(self.robot_arm_at(adapt_arm_noop_to))

        # move arm to goal arm pose
        self.adapt_arm_full = Method("adapt_arm_full", from_pose=self.get_type(ArmPose), to_pose=self.get_type(ArmPose))
        adapt_arm_full_from = self.adapt_arm_full.parameter("from_pose")
        adapt_arm_full_to = self.adapt_arm_full.parameter("to_pose")
        self.adapt_arm_full.set_task(self.adapt_arm, adapt_arm_full_to)
        self.adapt_arm_full.add_precondition(self.robot_arm_at(adapt_arm_full_from))
        self.adapt_arm_full.add_precondition(Not(Equals(adapt_arm_full_from, adapt_arm_full_to)))
        self.adapt_arm_full.add_subtask(self.move_arm, self.robot, adapt_arm_full_from, adapt_arm_full_to)

        # PERCEIVE LOCATION
        # already at location, arm pose unknown
        self.perceive_move_arm = Method("perceive_move_arm", pose=self.get_type(Pose), location=self.get_type(Location))
        perceive_move_arm_pose = self.perceive_move_arm.parameter("pose")
        perceive_move_arm_loc = self.perceive_move_arm.parameter("location")
        self.perceive_move_arm.set_task(self.perceive, perceive_move_arm_loc)
        self.perceive_move_arm.add_precondition(self.robot_at(perceive_move_arm_pose))
        self.perceive_move_arm.add_precondition(
            Or(self.robot_arm_at(arm_pose) for arm_pose in (self.arm_pose_handover, self.arm_pose_unknown))
        )
        s1 = self.perceive_move_arm.add_subtask(self.adapt_arm, self.arm_pose_observe)
        s2 = self.perceive_move_arm.add_subtask(
            self.search_at, self.robot, perceive_move_arm_pose, perceive_move_arm_loc
        )
        self.perceive_move_arm.set_ordered(s1, s2)

        # already at location, perceive location
        self.perceive_location = Method("perceive_location", pose=self.get_type(Pose), location=self.get_type(Location))
        perceive_location_pose = self.perceive_location.parameter("pose")
        perceive_location_loc = self.perceive_location.parameter("location")
        self.perceive_location.set_task(self.perceive, perceive_location_loc)
        self.perceive_location.add_precondition(self.robot_at(perceive_location_pose))
        self.perceive_location.add_subtask(self.search_at, self.robot, perceive_location_pose, perceive_location_loc)

        # drive to location, perceive location
        self.perceive_full = Method("perceive_full", pose=self.get_type(Pose), location=self.get_type(Location))
        perceive_full_pose = self.perceive_full.parameter("pose")
        perceive_full_loc = self.perceive_full.parameter("location")
        self.perceive_full.set_task(self.perceive, perceive_full_loc)
        s1 = self.perceive_full.add_subtask(self.drive, perceive_full_pose)
        s2 = self.perceive_full.add_subtask(self.search_at, self.robot, perceive_full_pose, perceive_full_loc)
        self.perceive_full.set_ordered(s1, s2)

        # GET ITEM
        # item already in robots gripper
        self.get_item_noop = Method("get_item_noop", item=self.get_type(Item))
        get_item_noop_item = self.get_item_noop.parameter("item")
        self.get_item_noop.set_task(self.get_item, get_item_noop_item)
        self.get_item_noop.add_precondition(self.robot_has(get_item_noop_item))

        # not holding an item, robot already at item location, pick up item
        self.get_item_pick = Method(
            "get_item_pick", item=self.get_type(Item), item_loc=self.get_type(Location), pose=self.get_type(Pose)
        )
        get_item_pick_item = self.get_item_pick.parameter("item")
        get_item_pick_loc = self.get_item_pick.parameter("item_loc")
        get_item_pick_pose = self.get_item_pick.parameter("pose")
        self.get_item_pick.set_task(self.get_item, get_item_pick_item)
        self.get_item_pick.add_precondition(self.robot_at(get_item_pick_pose))
        self.get_item_pick.add_precondition(self.robot_has(self.nothing))
        self.get_item_pick.add_precondition(self.believe_item_at(get_item_pick_item, get_item_pick_loc))
        self.get_item_pick.add_precondition(self.pose_at(get_item_pick_pose, get_item_pick_loc))
        self.get_item_pick.add_subtask(
            self.pick_item, self.robot, get_item_pick_pose, get_item_pick_loc, get_item_pick_item
        )

        # not holding an item, go to item location and pick up item
        self.get_item_full = Method(
            "get_item_full", item=self.get_type(Item), item_loc=self.get_type(Location), to_pose=self.get_type(Pose)
        )
        get_item_full_item = self.get_item_full.parameter("item")
        get_item_full_loc = self.get_item_full.parameter("item_loc")
        get_item_full_to_pose = self.get_item_full.parameter("to_pose")
        self.get_item_full.set_task(self.get_item, get_item_full_item)
        self.get_item_full.add_precondition(self.robot_has(self.nothing))
        self.get_item_full.add_precondition(self.believe_item_at(get_item_full_item, get_item_full_loc))
        self.get_item_full.add_precondition(self.pose_at(get_item_full_to_pose, get_item_full_loc))
        s1 = self.get_item_full.add_subtask(self.drive, get_item_full_to_pose)
        s2 = self.get_item_full.add_subtask(
            self.pick_item, self.robot, get_item_full_to_pose, get_item_full_loc, get_item_full_item
        )
        self.get_item_full.set_ordered(s1, s2)

        # PUT ITEM
        # robot already at target pose, place item
        self.put_item_place = Method(
            "put_item_place", item=self.get_type(Item), location=self.get_type(Location), pose=self.get_type(Pose)
        )
        put_item_place_item = self.put_item_place.parameter("item")
        put_item_place_location = self.put_item_place.parameter("location")
        put_item_place_pose = self.put_item_place.parameter("pose")
        self.put_item_place.set_task(self.put_item, put_item_place_item, put_item_place_location)
        self.put_item_place.add_precondition(self.robot_has(put_item_place_item))
        self.put_item_place.add_precondition(self.pose_at(put_item_place_pose, put_item_place_location))
        self.put_item_place.add_precondition(self.robot_at(put_item_place_pose))
        self.put_item_place.add_subtask(
            self.place_item, self.robot, put_item_place_pose, put_item_place_location, put_item_place_item
        )

        # robot drives to target pose and places item
        self.put_item_full = Method(
            "put_item_full", item=self.get_type(Item), location=self.get_type(Location), pose=self.get_type(Pose)
        )
        put_item_full_item = self.put_item_full.parameter("item")
        put_item_full_location = self.put_item_full.parameter("location")
        put_item_full_pose = self.put_item_full.parameter("pose")
        self.put_item_full.set_task(self.put_item, put_item_full_item, put_item_full_location)
        self.put_item_full.add_precondition(self.robot_has(put_item_full_item))
        self.put_item_full.add_precondition(self.pose_at(put_item_full_pose, put_item_full_location))
        s1 = self.put_item_full.add_subtask(self.drive, put_item_full_pose)
        s2 = self.put_item_full.add_subtask(
            self.place_item, self.robot, put_item_full_pose, put_item_full_location, put_item_full_item
        )
        self.put_item_full.set_ordered(s1, s2)

        # MOVE ITEM
        # move item from one location to another location
        self.move_item_full = Method(
            "move_item_full", item=self.get_type(Item), location=self.get_type(Location), pose=self.get_type(Pose)
        )
        move_item_full_item = self.move_item_full.parameter("item")
        move_item_full_loc = self.move_item_full.parameter("location")
        self.move_item_full.set_task(self.move_item, move_item_full_item, move_item_full_loc)
        s1 = self.move_item_full.add_subtask(self.get_item, move_item_full_item)
        s2 = self.move_item_full.add_subtask(self.put_item, move_item_full_item, move_item_full_loc)
        self.move_item_full.set_ordered(s1, s2)

        self.problem = self.define_mobipick_problem(
            fluents=(
                self.robot_at,
                self.robot_arm_at,
                self.robot_has,
                self.believe_item_at,
                self.pose_at,
                self.item_offered,
                self.searched_at,
            ),
            actions=(
                self.move_base,
                self.move_base_with_item,
                self.move_arm,
                self.pick_item,
                self.place_item,
                self.store_item,
                self.handover_item,
                self.search_tool,
                self.search_box,
                self.search_at,
            ),
            methods=(
                self.drive_noop,
                self.drive_homeposture,
                self.drive_transport,
                self.adapt_arm_noop,
                self.adapt_arm_full,
                self.perceive_move_arm,
                self.perceive_location,
                self.perceive_full,
                self.get_item_noop,
                self.get_item_pick,
                self.get_item_full,
                self.put_item_place,
                self.put_item_full,
                self.move_item_full,
            ),
            tasks=(
                self.drive,
                self.adapt_arm,
                self.perceive,
                self.get_item,
                self.put_item,
                self.move_item,
            ),
        )
        self.problem.add_quality_metric(MinimizeSequentialPlanLength())

    def define_mobipick_problem(
        self,
        fluents: Optional[Iterable[Fluent]] = None,
        actions: Optional[Iterable[InstantaneousAction]] = None,
        tasks: Optional[Iterable[Task]] = None,
        methods: Optional[Iterable[Method]] = None,
        poses: Optional[Iterable[Object]] = None,
        items: Optional[Iterable[Object]] = None,
        locations: Optional[Iterable[Object]] = None,
    ) -> HierarchicalProblem:
        """Define hierarchical UP problem by its (potential subsets of) fluents, actions, tasks, methods and objects."""
        problem = HierarchicalProblem()
        problem.add_fluents(self._fluents.values() if fluents is None else fluents)
        problem.add_actions(self._actions.values() if actions is None else actions)
        objects = set(
            [self.robot, self.unknown_pose, self.nothing]
            + (self.poses if poses is None else list(poses))
            + self.arm_poses
            + (self.items if items is None else list(items))
            + (self.locations if locations is None else list(locations))
        )
        problem.add_objects(self._objects.values() if objects is None else objects)
        if tasks is not None:
            for task in tasks:
                problem.add_task(task)
        else:
            for task in self.tasks:
                problem.add_task(task)
        if methods is not None:
            for method in methods:
                problem.add_method(method)
        else:
            for method in self.methods:
                problem.add_method(method)

        return problem

    def set_task(self, task: Union[Task, Subtask, Action]) -> None:
        """Set the task to be executed."""
        self.problem.task_network.add_subtask(task)
