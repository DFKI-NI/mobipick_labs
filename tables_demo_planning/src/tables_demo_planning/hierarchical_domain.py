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

import rospy
from typing import Iterable, Optional, Union, Dict, List, Set
from collections import defaultdict
from geometry_msgs.msg import Pose
from unified_planning.model import Fluent, InstantaneousAction, Object, Action, Problem
from unified_planning.model.htn import HierarchicalProblem, Method, Task, Subtask
from unified_planning.shortcuts import Equals, Not, Or, OneshotPlanner
from unified_planning.plans import PlanKind, ActionInstance
from unified_planning.model.metrics import MinimizeSequentialPlanLength
from unified_planning.engines import OptimalityGuarantee
from tables_demo_planning.components import Robot, ArmPose, Item, Location
from tables_demo_planning.tables_demo_api import TablesDemoAPI


class HierarchicalDomain:
    def __init__(self, api_items: Iterable[Item]) -> None:
        self.tables_demo_api = TablesDemoAPI(api_items)
        # Aliases for domain, env and visualization variable
        self.domain = self.tables_demo_api.domain
        self.env = self.tables_demo_api.env
        self.visualization = self.tables_demo_api.visualization

        # UP types
        type_robot = self.domain.get_type(Robot)
        type_pose = self.domain.get_type(Pose)
        type_arm_pose = self.domain.get_type(ArmPose)
        type_location = self.domain.get_type(Location)
        type_item = self.domain.get_type(Item)

        # UP Objects
        obj_arm_pose_home = self.domain.get(ArmPose, "home")
        obj_arm_pose_transport = self.domain.get(ArmPose, "transport")
        obj_arm_pose_handover = self.domain.get(ArmPose, "handover")
        obj_arm_pose_observe = self.domain.get(ArmPose, "observe100cm_right")
        obj_location_in_klt = self.domain.get(Location, "in_klt")
        obj_pose_handover = self.domain.get(Pose, "base_handover_pose")

        # UP Actions
        self.move_base = self.domain._actions["move_base"]
        self.move_base_with_item = self.domain._actions["move_base_with_item"]
        self.move_arm = self.domain._actions["move_arm"]
        self.pick_item = self.domain._actions["pick_item"]
        self.place_item = self.domain._actions["place_item"]
        self.store_item = self.domain._actions["store_item"]
        self.hand_over_item = self.domain._actions["hand_over_item"]
        self.search_tool = self.domain._actions["search_tool"]
        self.search_klt = self.domain._actions["search_klt"]
        self.search_at = self.domain._actions["search_at"]

        # TASKS
        self.drive = Task("drive", robot=type_robot, goal_pose=type_pose)
        self.adapt_arm = Task("adapt_arm", robot=type_robot, to_pose=type_arm_pose)
        self.perceive = Task("perceive", robot=type_robot, location=type_location)
        self.get_item = Task("get_item", robot=type_robot, item=type_item)
        self.put_item = Task("put_item", robot=type_robot, item=type_item, location=type_location)
        self.move_item = Task("move_item", robot=type_robot, item=type_item, location=type_location)
        self.insert_item = Task("insert_item", robot=type_robot, item=type_item, klt=type_item)
        self.bring_item = Task("bring_item", robot=type_robot, item=type_item)
        self.search_item = Task("search_item", robot=type_robot, item=type_item)
        self.tables_demo = Task("tables_demo", robot=type_robot, tool=type_item, klt=type_item, location=type_location)

        # METHODS

        # DRIVE
        # robot already at goal pose
        self.drive_noop = Method("drive_noop", robot=type_robot, goal_pose=type_pose)
        self.drive_noop.set_task(self.drive, self.drive_noop.robot, self.drive_noop.goal_pose)
        self.drive_noop.add_precondition(self.domain.robot_at(self.drive_noop.robot, self.drive_noop.goal_pose))

        # arm not holding anything, move arm to home pose and move base to goal location
        self.drive_homeposture = Method(
            "drive_homeposture", robot=type_robot, start_pose=type_pose, goal_pose=type_pose
        )
        self.drive_homeposture.set_task(self.drive, self.drive_homeposture.robot, self.drive_homeposture.goal_pose)
        self.drive_homeposture.add_precondition(
            self.domain.robot_at(self.drive_homeposture.robot, self.drive_homeposture.start_pose)
        )
        self.drive_homeposture.add_precondition(
            self.domain.robot_has(self.drive_homeposture.robot, self.domain.nothing)
        )
        s1 = self.drive_homeposture.add_subtask(self.adapt_arm, self.drive_homeposture.robot, obj_arm_pose_home)
        s2 = self.drive_homeposture.add_subtask(
            self.move_base,
            self.drive_homeposture.robot,
            self.drive_homeposture.start_pose,
            self.drive_homeposture.goal_pose,
        )
        self.drive_homeposture.set_ordered(s1, s2)

        # arm holding an item, move arm to transport pose and move base to goal location
        self.drive_transport = Method(
            "drive_transport", robot=type_robot, item=type_item, start_pose=type_pose, goal_pose=type_pose
        )
        self.drive_transport.set_task(self.drive, self.drive_transport.robot, self.drive_transport.goal_pose)
        self.drive_transport.add_precondition(
            self.domain.robot_at(self.drive_transport.robot, self.drive_transport.start_pose)
        )
        self.drive_transport.add_precondition(
            self.domain.robot_has(self.drive_transport.robot, self.drive_transport.item)
        )
        s1 = self.drive_transport.add_subtask(self.adapt_arm, self.drive_transport.robot, obj_arm_pose_transport)
        s2 = self.drive_transport.add_subtask(
            self.move_base_with_item,
            self.drive_transport.robot,
            self.drive_transport.item,
            self.drive_transport.start_pose,
            self.drive_transport.goal_pose,
        )
        self.drive_transport.set_ordered(s1, s2)

        # ADAPT ARM
        # arm already in goal arm pose
        self.adapt_arm_noop = Method("adapt_arm_noop", robot=type_robot, to_pose=type_arm_pose)
        self.adapt_arm_noop.set_task(self.adapt_arm, self.adapt_arm_noop.robot, self.adapt_arm_noop.to_pose)
        self.adapt_arm_noop.add_precondition(
            self.domain.robot_arm_at(self.adapt_arm_noop.robot, self.adapt_arm_noop.to_pose)
        )

        # move arm to goal arm pose
        self.adapt_arm_full = Method("adapt_arm_full", robot=type_robot, from_pose=type_arm_pose, to_pose=type_arm_pose)
        self.adapt_arm_full.set_task(self.adapt_arm, self.adapt_arm_full.robot, self.adapt_arm_full.to_pose)
        self.adapt_arm_full.add_precondition(
            self.domain.robot_arm_at(self.adapt_arm_full.robot, self.adapt_arm_full.from_pose)
        )
        self.adapt_arm_full.add_precondition(Not(Equals(self.adapt_arm_full.from_pose, self.adapt_arm_full.to_pose)))
        self.adapt_arm_full.add_subtask(
            self.move_arm,
            self.adapt_arm_full.robot,
            self.adapt_arm_full.from_pose,
            self.adapt_arm_full.to_pose,
        )

        # PERCEIVE LOCATION
        # already perceived location
        self.perceive_noop = Method("perceive_noop", robot=type_robot, pose=type_pose, location=type_location)
        self.perceive_noop.set_task(self.perceive, self.perceive_noop.robot, self.perceive_noop.location)
        self.perceive_noop.add_precondition(self.domain.searched_at(self.perceive_noop.location))

        # already at location, arm pose unknown
        self.perceive_move_arm = Method("perceive_move_arm", robot=type_robot, pose=type_pose, location=type_location)
        self.perceive_move_arm.set_task(self.perceive, self.perceive_move_arm.robot, self.perceive_move_arm.location)
        self.perceive_move_arm.add_precondition(
            self.domain.robot_at(self.perceive_move_arm.robot, self.perceive_move_arm.pose)
        )
        self.perceive_move_arm.add_precondition(
            Or(
                self.domain.robot_arm_at(self.perceive_move_arm.robot, arm_pose)
                for arm_pose in (obj_arm_pose_handover, self.domain.arm_pose_unknown)
            )
        )
        s1 = self.perceive_move_arm.add_subtask(self.adapt_arm, self.perceive_move_arm.robot, obj_arm_pose_observe)
        s2 = self.perceive_move_arm.add_subtask(
            self.search_at,
            self.perceive_move_arm.robot,
            self.perceive_move_arm.pose,
            self.perceive_move_arm.location,
        )
        self.perceive_move_arm.set_ordered(s1, s2)

        # already at location, perceive location
        self.perceive_location = Method("perceive_location", robot=type_robot, pose=type_pose, location=type_location)
        self.perceive_location.set_task(self.perceive, self.perceive_location.robot, self.perceive_location.location)
        self.perceive_location.add_precondition(
            self.domain.robot_at(self.perceive_location.robot, self.perceive_location.pose)
        )
        self.perceive_location.add_subtask(
            self.search_at,
            self.perceive_location.robot,
            self.perceive_location.pose,
            self.perceive_location.location,
        )

        # drive to location, perceive location
        self.perceive_full = Method("perceive_full", robot=type_robot, pose=type_pose, location=type_location)
        self.perceive_full.set_task(self.perceive, self.perceive_full.robot, self.perceive_full.location)
        s1 = self.perceive_full.add_subtask(self.drive, self.perceive_full.robot, self.perceive_full.pose)
        s2 = self.perceive_full.add_subtask(
            self.search_at,
            self.perceive_full.robot,
            self.perceive_full.pose,
            self.perceive_full.location,
        )
        self.perceive_full.set_ordered(s1, s2)

        # GET ITEM
        # item already in robots gripper
        self.get_item_noop = Method("get_item_noop", robot=type_robot, item=type_item)
        self.get_item_noop.set_task(self.get_item, self.get_item_noop.robot, self.get_item_noop.item)
        self.get_item_noop.add_precondition(self.domain.robot_has(self.get_item_noop.robot, self.get_item_noop.item))

        # not holding an item, robot already at item location, pick up item
        self.get_item_pick = Method(
            "get_item_pick", robot=type_robot, item=type_item, location=type_location, pose=type_pose
        )
        self.get_item_pick.set_task(self.get_item, self.get_item_pick.robot, self.get_item_pick.item)
        self.get_item_pick.add_precondition(self.domain.robot_at(self.get_item_pick.robot, self.get_item_pick.pose))
        self.get_item_pick.add_precondition(self.domain.robot_has(self.get_item_pick.robot, self.domain.nothing))
        self.get_item_pick.add_precondition(
            self.domain.believe_item_at(self.get_item_pick.item, self.get_item_pick.location)
        )
        self.get_item_pick.add_precondition(self.domain.pose_at(self.get_item_pick.pose, self.get_item_pick.location))
        self.get_item_pick.add_subtask(
            self.pick_item,
            self.get_item_pick.robot,
            self.get_item_pick.pose,
            self.get_item_pick.location,
            self.get_item_pick.item,
        )

        # not holding an item, go to item location and pick up item
        self.get_item_full = Method(
            "get_item_full", robot=type_robot, item=type_item, location=type_location, pose=type_pose
        )
        self.get_item_full.set_task(self.get_item, self.get_item_full.robot, self.get_item_full.item)
        self.get_item_full.add_precondition(self.domain.robot_has(self.get_item_full.robot, self.domain.nothing))
        self.get_item_full.add_precondition(
            self.domain.believe_item_at(self.get_item_full.item, self.get_item_full.location)
        )
        self.get_item_full.add_precondition(self.domain.pose_at(self.get_item_full.pose, self.get_item_full.location))
        s1 = self.get_item_full.add_subtask(self.drive, self.get_item_full.robot, self.get_item_full.pose)
        s2 = self.get_item_full.add_subtask(
            self.pick_item,
            self.get_item_full.robot,
            self.get_item_full.pose,
            self.get_item_full.location,
            self.get_item_full.item,
        )
        self.get_item_full.set_ordered(s1, s2)

        # PUT ITEM
        # item already at target location, robot is holding nothing
        self.put_item_noop = Method(
            "put_item_noop", robot=type_robot, item=type_item, location=type_location, pose=type_pose
        )
        self.put_item_noop.set_task(
            self.put_item, self.put_item_noop.robot, self.put_item_noop.item, self.put_item_noop.location
        )
        self.put_item_noop.add_precondition(self.domain.robot_has(self.put_item_noop.robot, self.domain.nothing))
        self.put_item_noop.add_precondition(
            self.domain.believe_item_at(self.put_item_noop.item, self.put_item_noop.location)
        )

        # robot already at target pose, place item
        self.put_item_place = Method(
            "put_item_place", robot=type_robot, item=type_item, location=type_location, pose=type_pose
        )
        self.put_item_place.set_task(
            self.put_item, self.put_item_place.robot, self.put_item_place.item, self.put_item_place.location
        )
        self.put_item_place.add_precondition(self.domain.robot_has(self.put_item_place.robot, self.put_item_place.item))
        self.put_item_place.add_precondition(
            self.domain.pose_at(self.put_item_place.pose, self.put_item_place.location)
        )
        self.put_item_place.add_precondition(self.domain.robot_at(self.put_item_place.robot, self.put_item_place.pose))
        self.put_item_place.add_subtask(
            self.place_item,
            self.put_item_place.robot,
            self.put_item_place.pose,
            self.put_item_place.location,
            self.put_item_place.item,
        )

        # robot drives to target pose and places item
        self.put_item_full = Method(
            "put_item_full", robot=type_robot, item=type_item, location=type_location, pose=type_pose
        )
        self.put_item_full.set_task(
            self.put_item, self.put_item_full.robot, self.put_item_full.item, self.put_item_full.location
        )
        self.put_item_full.add_precondition(self.domain.robot_has(self.put_item_full.robot, self.put_item_full.item))
        self.put_item_full.add_precondition(self.domain.pose_at(self.put_item_full.pose, self.put_item_full.location))
        s1 = self.put_item_full.add_subtask(self.drive, self.put_item_full.robot, self.put_item_full.pose)
        s2 = self.put_item_full.add_subtask(
            self.place_item,
            self.put_item_full.robot,
            self.put_item_full.pose,
            self.put_item_full.location,
            self.put_item_full.item,
        )
        self.put_item_full.set_ordered(s1, s2)

        # MOVE ITEM
        # item already at location, nothing to do
        self.move_item_noop = Method("move_item_noop", robot=type_robot, item=type_item, location=type_location)
        self.move_item_noop.set_task(
            self.move_item, self.move_item_noop.robot, self.move_item_noop.item, self.move_item_noop.location
        )
        self.move_item_noop.add_precondition(
            self.domain.believe_item_at(self.move_item_noop.item, self.move_item_noop.location)
        )

        # move item from one location to another location
        self.move_item_full = Method("move_item_full", robot=type_robot, item=type_item, location=type_location)
        self.move_item_full.set_task(
            self.move_item, self.move_item_full.robot, self.move_item_full.item, self.move_item_full.location
        )
        s1 = self.move_item_full.add_subtask(self.get_item, self.move_item_full.robot, self.move_item_full.item)
        s2 = self.move_item_full.add_subtask(
            self.put_item, self.move_item_full.robot, self.move_item_full.item, self.move_item_full.location
        )
        self.move_item_full.set_ordered(s1, s2)

        # INSERT ITEM
        # item already in klt
        self.insert_item_noop = Method(
            "insert_item_noop",
            robot=type_robot,
            klt_pose=type_pose,
            klt_loc=type_location,
            item=type_item,
            klt=type_item,
        )
        self.insert_item_noop.set_task(
            self.insert_item, self.insert_item_noop.robot, self.insert_item_noop.item, self.insert_item_noop.klt
        )
        self.insert_item_noop.add_precondition(
            self.domain.believe_item_at(self.insert_item_noop.item, obj_location_in_klt)
        )

        # already at klt location, store item
        self.insert_item_store = Method(
            "insert_item_store",
            robot=type_robot,
            klt_pose=type_pose,
            klt_loc=type_location,
            item=type_item,
            klt=type_item,
        )
        self.insert_item_store.set_task(
            self.insert_item, self.insert_item_store.robot, self.insert_item_store.item, self.insert_item_store.klt
        )
        self.insert_item_store.add_precondition(
            Not(self.domain.believe_item_at(self.insert_item_store.klt, self.domain.anywhere))
        )
        self.insert_item_store.add_precondition(
            self.domain.pose_at(self.insert_item_store.klt_pose, self.insert_item_store.klt_loc)
        )
        self.insert_item_store.add_precondition(
            self.domain.robot_has(self.insert_item_store.robot, self.insert_item_store.item)
        )
        self.insert_item_store.add_precondition(
            self.domain.robot_at(self.insert_item_store.robot, self.insert_item_store.klt_pose)
        )
        s1 = self.insert_item_store.add_subtask(
            self.drive, self.insert_item_store.robot, self.insert_item_store.klt_pose
        )
        s2 = self.insert_item_store.add_subtask(
            self.store_item,
            self.insert_item_store.robot,
            self.insert_item_store.klt_pose,
            self.insert_item_store.klt_loc,
            self.insert_item_store.item,
            self.insert_item_store.klt,
        )
        self.insert_item_store.set_ordered(s1, s2)

        # already holding item, move to klt and insert
        self.insert_item_drive = Method(
            "insert_item_drive",
            robot=type_robot,
            klt_pose=type_pose,
            klt_loc=type_location,
            item=type_item,
            klt=type_item,
        )
        self.insert_item_drive.set_task(
            self.insert_item, self.insert_item_drive.robot, self.insert_item_drive.item, self.insert_item_drive.klt
        )
        self.insert_item_drive.add_precondition(
            Not(self.domain.believe_item_at(self.insert_item_drive.klt, self.domain.anywhere))
        )
        self.insert_item_drive.add_precondition(
            self.domain.pose_at(self.insert_item_drive.klt_pose, self.insert_item_drive.klt_loc)
        )
        self.insert_item_drive.add_precondition(
            self.domain.robot_has(self.insert_item_drive.robot, self.insert_item_drive.item)
        )
        s1 = self.insert_item_drive.add_subtask(
            self.drive, self.insert_item_drive.robot, self.insert_item_drive.klt_pose
        )
        s2 = self.insert_item_drive.add_subtask(
            self.store_item,
            self.insert_item_drive.robot,
            self.insert_item_drive.klt_pose,
            self.insert_item_drive.klt_loc,
            self.insert_item_drive.item,
            self.insert_item_drive.klt,
        )
        self.insert_item_drive.set_ordered(s1, s2)

        # go to item location, pickup item, go to klt location, store item in klt
        self.insert_item_full = Method(
            "insert_item_full",
            robot=type_robot,
            klt_pose=type_pose,
            klt_loc=type_location,
            item=type_item,
            klt=type_item,
        )
        self.insert_item_full.set_task(
            self.insert_item, self.insert_item_full.robot, self.insert_item_full.item, self.insert_item_full.klt
        )
        self.insert_item_full.add_precondition(
            Not(self.domain.believe_item_at(self.insert_item_full.klt, self.domain.anywhere))
        )
        self.insert_item_full.add_precondition(
            self.domain.pose_at(self.insert_item_full.klt_pose, self.insert_item_full.klt_loc)
        )
        s1 = self.insert_item_full.add_subtask(self.get_item, self.insert_item_full.robot, self.insert_item_full.item)
        s2 = self.insert_item_full.add_subtask(self.drive, self.insert_item_full.robot, self.insert_item_full.klt_pose)
        s3 = self.insert_item_full.add_subtask(
            self.store_item,
            self.insert_item_full.robot,
            self.insert_item_full.klt_pose,
            self.insert_item_full.klt_loc,
            self.insert_item_full.item,
            self.insert_item_full.klt,
        )
        self.insert_item_full.set_ordered(s1, s2, s3)

        # BRING ITEM
        # robot already has item and is at handover pose, hand item over, move arm to home pose
        self.bring_item_handover = Method("bring_item_handover", robot=type_robot, item=type_item)
        self.bring_item_handover.set_task(
            self.bring_item, self.bring_item_handover.robot, self.bring_item_handover.item
        )
        self.bring_item_handover.add_precondition(
            self.domain.robot_has(self.bring_item_handover.robot, self.bring_item_handover.item)
        )
        self.bring_item_handover.add_precondition(
            self.domain.robot_at(self.bring_item_handover.robot, obj_pose_handover)
        )
        s1 = self.bring_item_handover.add_subtask(
            self.hand_over_item, self.bring_item_handover.robot, self.bring_item_handover.item
        )
        s2 = self.bring_item_handover.add_subtask(self.adapt_arm, self.bring_item_handover.robot, obj_arm_pose_home)
        self.bring_item_handover.set_ordered(s1, s2)

        # robot already has item, move to handover pose, hand item over, move arm to home pose
        self.bring_item_drive = Method("bring_item_drive", robot=type_robot, item=type_item)
        self.bring_item_drive.set_task(self.bring_item, self.bring_item_drive.robot, self.bring_item_drive.item)
        self.bring_item_drive.add_precondition(
            self.domain.robot_has(self.bring_item_drive.robot, self.bring_item_drive.item)
        )
        s1 = self.bring_item_drive.add_subtask(self.drive, self.bring_item_drive.robot, obj_pose_handover)
        s2 = self.bring_item_drive.add_subtask(
            self.hand_over_item, self.bring_item_drive.robot, self.bring_item_drive.item
        )
        s3 = self.bring_item_drive.add_subtask(self.adapt_arm, self.bring_item_drive.robot, obj_arm_pose_home)
        self.bring_item_drive.set_ordered(s1, s2, s3)

        # go to item location, pick up item, go to handover pose, hand item over, move arm to home pose
        self.bring_item_full = Method("bring_item_full", robot=type_robot, item=type_item)
        self.bring_item_full.set_task(self.bring_item, self.bring_item_full.robot, self.bring_item_full.item)
        s1 = self.bring_item_full.add_subtask(self.get_item, self.bring_item_full.robot, self.bring_item_full.item)
        s2 = self.bring_item_full.add_subtask(self.drive, self.bring_item_full.robot, obj_pose_handover)
        s3 = self.bring_item_full.add_subtask(
            self.hand_over_item, self.bring_item_full.robot, self.bring_item_full.item
        )
        s4 = self.bring_item_full.add_subtask(self.adapt_arm, self.bring_item_full.robot, obj_arm_pose_home)
        self.bring_item_full.set_ordered(s1, s2, s3, s4)

        # SEARCH ITEM
        # item location already known, nothing to do
        self.search_item_noop = Method("search_item_noop", robot=type_robot, item=type_item)
        self.search_item_noop.set_task(self.search_item, self.search_item_noop.robot, self.search_item_noop.item)
        self.search_item_noop.add_precondition(
            Not(self.domain.believe_item_at(self.search_item_noop.item, self.domain.anywhere))
        )

        # full search
        self.search_item_full = Method("search_item_full", robot=type_robot, item=type_item)
        self.search_item_full.set_task(self.search_item, self.search_item_full.robot, self.search_item_full.item)
        self.search_item_full.add_precondition(
            self.domain.believe_item_at(self.search_item_full.item, self.domain.anywhere)
        )
        for location in self.domain.get_table_objects():
            self.search_item_full.add_subtask(self.perceive, self.search_item_full.robot, location)

        # TABLES DEMO
        # tool in klt on target table, nothing to do
        self.tables_demo_noop = Method(
            "tables_demo_noop", robot=type_robot, tool=type_item, klt=type_item, location=type_location
        )
        self.tables_demo_noop.set_task(
            self.tables_demo,
            self.tables_demo_noop.robot,
            self.tables_demo_noop.tool,
            self.tables_demo_noop.klt,
            self.tables_demo_noop.location,
        )
        self.tables_demo_noop.add_precondition(
            self.domain.believe_item_at(self.tables_demo_noop.tool, obj_location_in_klt)
        )
        self.tables_demo_noop.add_precondition(self.domain.robot_has(self.tables_demo_noop.robot, self.domain.nothing))
        self.tables_demo_noop.add_precondition(
            self.domain.believe_item_at(self.tables_demo_noop.klt, self.tables_demo_noop.location)
        )

        # klt location already known
        self.tables_demo_search_tool = Method(
            "tables_demo_search_tool",
            robot=type_robot,
            tool=type_item,
            klt=type_item,
            location=type_location,
        )
        self.tables_demo_search_tool.set_task(
            self.tables_demo,
            self.tables_demo_search_tool.robot,
            self.tables_demo_search_tool.tool,
            self.tables_demo_search_tool.klt,
            self.tables_demo_search_tool.location,
        )
        self.tables_demo_search_tool.add_precondition(
            Not(self.domain.believe_item_at(self.tables_demo_search_tool.klt, self.domain.anywhere))
        )
        s1 = self.tables_demo_search_tool.add_subtask(
            self.search_tool, self.tables_demo_search_tool.robot, self.tables_demo_search_tool.tool
        )
        s2 = self.tables_demo_search_tool.add_subtask(
            self.get_item, self.tables_demo_search_tool.robot, self.tables_demo_search_tool.tool
        )
        s3 = self.tables_demo_search_tool.add_subtask(
            self.insert_item,
            self.tables_demo_search_tool.robot,
            self.tables_demo_search_tool.tool,
            self.tables_demo_search_tool.klt,
        )
        s4 = self.tables_demo_search_tool.add_subtask(
            self.move_item,
            self.tables_demo_search_tool.robot,
            self.tables_demo_search_tool.klt,
            self.tables_demo_search_tool.location,
        )
        self.tables_demo_search_tool.set_ordered(s1, s2, s3, s4)

        # tool location already known
        self.tables_demo_search_klt = Method(
            "tables_demo_search_klt", robot=type_robot, tool=type_item, klt=type_item, location=type_location
        )
        self.tables_demo_search_klt.set_task(
            self.tables_demo,
            self.tables_demo_search_klt.robot,
            self.tables_demo_search_klt.tool,
            self.tables_demo_search_klt.klt,
            self.tables_demo_search_klt.location,
        )
        self.tables_demo_search_klt.add_precondition(
            Not(self.domain.believe_item_at(self.tables_demo_search_klt.tool, self.domain.anywhere))
        )
        s1 = self.tables_demo_search_klt.add_subtask(
            self.get_item, self.tables_demo_search_klt.robot, self.tables_demo_search_klt.tool
        )
        s2 = self.tables_demo_search_klt.add_subtask(
            self.search_klt, self.tables_demo_search_klt.robot, self.tables_demo_search_klt.klt
        )
        s3 = self.tables_demo_search_klt.add_subtask(
            self.insert_item,
            self.tables_demo_search_klt.robot,
            self.tables_demo_search_klt.tool,
            self.tables_demo_search_klt.klt,
        )
        s4 = self.tables_demo_search_klt.add_subtask(
            self.move_item,
            self.tables_demo_search_klt.robot,
            self.tables_demo_search_klt.klt,
            self.tables_demo_search_klt.location,
        )
        self.tables_demo_search_klt.set_ordered(s1, s2, s3, s4)

        # tool and klt location already known, get item, insert into klt and move to target table
        self.tables_demo_move_item = Method(
            "tables_demo_move_item", robot=type_robot, tool=type_item, klt=type_item, location=type_location
        )
        self.tables_demo_move_item.set_task(
            self.tables_demo,
            self.tables_demo_move_item.robot,
            self.tables_demo_move_item.tool,
            self.tables_demo_move_item.klt,
            self.tables_demo_move_item.location,
        )
        self.tables_demo_move_item.add_precondition(
            Not(self.domain.believe_item_at(self.tables_demo_move_item.tool, self.domain.anywhere))
        )
        self.tables_demo_move_item.add_precondition(
            Not(self.domain.believe_item_at(self.tables_demo_move_item.klt, self.domain.anywhere))
        )
        s1 = self.tables_demo_move_item.add_subtask(
            self.get_item, self.tables_demo_move_item.robot, self.tables_demo_move_item.tool
        )
        s2 = self.tables_demo_move_item.add_subtask(
            self.insert_item,
            self.tables_demo_move_item.robot,
            self.tables_demo_move_item.tool,
            self.tables_demo_move_item.klt,
        )
        s3 = self.tables_demo_move_item.add_subtask(
            self.move_item,
            self.tables_demo_move_item.robot,
            self.tables_demo_move_item.klt,
            self.tables_demo_move_item.location,
        )
        self.tables_demo_move_item.set_ordered(s1, s2, s3)

        # full tables demo
        self.tables_demo_full = Method(
            "tables_demo_full", robot=type_robot, tool=type_item, klt=type_item, location=type_location
        )
        self.tables_demo_full.set_task(
            self.tables_demo,
            self.tables_demo_full.robot,
            self.tables_demo_full.tool,
            self.tables_demo_full.klt,
            self.tables_demo_full.location,
        )
        s1 = self.tables_demo_full.add_subtask(
            self.search_tool, self.tables_demo_full.robot, self.tables_demo_full.tool
        )
        s2 = self.tables_demo_full.add_subtask(self.get_item, self.tables_demo_full.robot, self.tables_demo_full.tool)
        s3 = self.tables_demo_full.add_subtask(self.search_klt, self.tables_demo_full.robot, self.tables_demo_full.klt)
        s4 = self.tables_demo_full.add_subtask(
            self.insert_item, self.tables_demo_full.robot, self.tables_demo_full.tool, self.tables_demo_full.klt
        )
        s5 = self.tables_demo_full.add_subtask(
            self.move_item, self.tables_demo_full.robot, self.tables_demo_full.klt, self.tables_demo_full.location
        )
        self.tables_demo_full.set_ordered(s1, s2, s3, s4, s5)

        self.problem = self.define_mobipick_problem(
            fluents=(
                self.domain.robot_at,
                self.domain.robot_arm_at,
                self.domain.robot_has,
                self.domain.believe_item_at,
                self.domain.believe_item_in,
                self.domain.pose_at,
                self.domain.item_offered,
                self.domain.searched_at,
            ),
            actions=(
                self.search_at,
                self.move_base,
                self.move_base_with_item,
                self.move_arm,
                self.pick_item,
                self.place_item,
                self.store_item,
                self.hand_over_item,
                self.search_tool,
                self.search_klt,
            ),
            methods=(
                self.drive_noop,
                self.drive_homeposture,
                self.drive_transport,
                self.adapt_arm_noop,
                self.adapt_arm_full,
                self.perceive_noop,
                self.perceive_move_arm,
                self.perceive_location,
                self.perceive_full,
                self.get_item_noop,
                self.get_item_pick,
                self.get_item_full,
                self.put_item_noop,
                self.put_item_place,
                self.put_item_full,
                self.move_item_noop,
                self.move_item_full,
                self.bring_item_handover,
                self.bring_item_drive,
                self.bring_item_full,
                self.search_item_noop,
                self.search_item_full,
                self.insert_item_noop,
                self.insert_item_drive,
                self.insert_item_store,
                self.insert_item_full,
                self.tables_demo_noop,
                self.tables_demo_search_klt,
                self.tables_demo_search_tool,
                self.tables_demo_move_item,
                self.tables_demo_full,
            ),
            tasks=(
                self.drive,
                self.adapt_arm,
                self.perceive,
                self.get_item,
                self.put_item,
                self.move_item,
                self.bring_item,
                self.search_item,
                self.insert_item,
                self.tables_demo,
            ),
        )
        self.problem.add_quality_metric(MinimizeSequentialPlanLength())

        self.subproblem = self.define_mobipick_problem(
            fluents=(
                self.domain.robot_at,
                self.domain.robot_arm_at,
                self.domain.robot_has,
                self.domain.believe_item_at,
                self.domain.believe_item_in,
                self.domain.searched_at,
                self.domain.pose_at,
            ),
            actions=(self.move_base, self.move_base_with_item, self.move_arm, self.search_at),
            methods=(
                self.drive_noop,
                self.drive_homeposture,
                self.drive_transport,
                self.adapt_arm_noop,
                self.adapt_arm_full,
                self.perceive_noop,
                self.perceive_move_arm,
                self.perceive_location,
                self.perceive_full,
                self.search_item_noop,
                self.search_item_full,
            ),
            tasks=(
                self.drive,
                self.adapt_arm,
                self.perceive,
                self.search_item,
                self.tables_demo,
            ),
        )
        self.subproblem.add_quality_metric(MinimizeSequentialPlanLength())

    def define_mobipick_problem(
        self,
        fluents: Optional[Iterable[Fluent]] = None,
        actions: Optional[Iterable[InstantaneousAction]] = None,
        objects: Optional[Iterable[Object]] = None,
        tasks: Optional[Iterable[Task]] = None,
        methods: Optional[Iterable[Method]] = None,
    ) -> HierarchicalProblem:
        """Define hierarchical UP problem by its (potential subsets of) fluents, actions, tasks, methods and objects."""
        problem = HierarchicalProblem()
        problem.add_fluents(self.domain._fluents.values() if fluents is None else fluents)
        problem.add_actions(self.domain._actions.values() if actions is None else actions)
        problem.add_objects(self.domain._objects.values() if objects is None else objects)
        if tasks is not None:
            for task in tasks:
                problem.add_task(task)
        if methods is not None:
            for method in methods:
                problem.add_method(method)

        return problem

    def solve_problem(self, problem: Problem) -> Optional[List[ActionInstance]]:
        """Solve planning problem and return plan."""
        result = OneshotPlanner(
            name="aries", problem_kind=problem.kind, optimality_guarantee=OptimalityGuarantee.SOLVED_OPTIMALLY
        ).solve(problem)
        if result.status and result.status.value > 2:
            if result.log_messages:
                rospy.logerr(f"Error during plan generation: {result.log_messages[0].message}")
                return None
            else:
                rospy.logerr(f"Error during plan generation: {result.status}")
                return None
        if result.plan is not None:
            plan = result.plan
            if plan.kind == PlanKind.HIERARCHICAL_PLAN:
                # First check if contained action plan is time-triggered plan
                # (if aries returns an empty plan it is a time-triggered, which
                # cant be converted to sequential and produces an error)
                if plan.action_plan and plan.action_plan.kind == PlanKind.TIME_TRIGGERED_PLAN:
                    return None
                # Convert hierarchical plan to sequential plan for execution
                plan = plan.convert_to(PlanKind.SEQUENTIAL_PLAN, self.problem)
            return plan.actions if plan.actions else None
        return None

    def replan(self) -> Optional[List[ActionInstance]]:
        """Print believed item locations, initialize UP problem, and solve it."""
        self.env.print_believed_item_locations()
        self.domain.set_initial_values(self.problem)
        return self.solve_problem(self.problem)

    def set_task(self, problem: HierarchicalProblem, task: Union[Task, Subtask, Action]) -> None:
        """Set the task to be executed."""
        problem.task_network.add_subtask(task)

    def clear_tasks(self, problem: HierarchicalProblem) -> None:
        problem.task_network._subtasks.clear()

    def run(self, target_item: Item, target_klt: Item, target_location: Location) -> None:
        """Run the mobipick tables demo."""
        retries_before_abortion = self.tables_demo_api.RETRIES_BEFORE_ABORTION
        error_counts: Dict[str, int] = defaultdict(int)
        executed_action_names: Set[str] = set()  # Note: For visualization purposes only.
        # Solve overall problem.
        self.clear_tasks(self.problem)
        self.set_task(
            self.problem,
            self.tables_demo(
                self.domain.robot,
                self.domain.objects[target_item.name],
                self.domain.objects[target_klt.name],
                self.domain.objects[target_location.name],
            ),
        )
        actions = self.replan()
        if actions is None:
            print("Execution ended because no plan could be found.")
            return

        # Loop action execution as long as there are actions.
        while actions:
            print("> Plan:")
            print('\n'.join(map(str, actions)))
            self.visualization.set_actions(
                [
                    f"{number + len(executed_action_names)} {self.tables_demo_api.label(action)}"
                    for number, action in enumerate(actions, start=1)
                ],
                preserve_actions=executed_action_names,
            )
            print("> Execution:")
            for action in actions:
                executable_action, parameters = self.domain.get_executable_action(action)
                action_name = f"{len(executed_action_names) + 1} {self.tables_demo_api.label(action)}"
                print(action)
                # Explicitly do not pick up KLT from target_table since planning does not handle it yet.
                if target_location and action.action.name == "pick_item" and parameters[-1].name.startswith("klt_"):
                    assert isinstance(parameters[-2], Location)
                    location = self.env.resolve_search_location(parameters[-2])
                    if location == target_location:
                        print("Picking up KLT OBSOLETE.")
                        self.visualization.cancel(action_name)
                        print("Replanning")
                        actions = self.replan()
                        break

                self.visualization.execute(action_name)

                # Execute action.
                result = executable_action(
                    *(parameters[1:] if hasattr(self.tables_demo_api.mobipick, action.action.name) else parameters)
                )
                executed_action_names.add(action_name)

                # Handle item search as an inner execution loop.
                # Rationale: It has additional stop criteria, and might continue the outer loop.
                if self.env.item_search:
                    try:
                        # Check whether an obsolete item search invalidates the previous plan.
                        if self.env.believed_item_locations[self.env.item_search] != Location.get("anywhere"):
                            print(f"Search for {self.env.item_search.name} OBSOLETE.")
                            self.visualization.cancel(action_name)
                            actions = self.replan()
                            break

                        # Search for item by creating and executing a subplan.
                        self.clear_tasks(self.subproblem)
                        self.domain.set_initial_values(self.subproblem)
                        self.set_task(
                            self.subproblem,
                            self.search_item(self.domain.robot, self.domain.objects[self.env.item_search.name]),
                        )
                        subactions = self.solve_problem(self.subproblem)
                        assert subactions, f"No solution for: {self.subproblem}"
                        print("- Search plan:")
                        print('\n'.join(map(str, subactions)))
                        self.visualization.set_actions(
                            [
                                f"{len(executed_action_names)}{chr(number)} {self.tables_demo_api.label(subaction)}"
                                for number, subaction in enumerate(subactions, start=97)
                            ],
                            preserve_actions=executed_action_names,
                            predecessor=action_name,
                        )
                        print("- Search execution:")
                        subaction_execution_count = 0
                        for subaction in subactions:
                            executable_subaction, subparameters = self.domain.get_executable_action(subaction)
                            subaction_name = (
                                f"{len(executed_action_names)}{chr(subaction_execution_count + 97)}"
                                f" {self.tables_demo_api.label(subaction)}"
                            )
                            print(subaction)
                            self.visualization.execute(subaction_name)
                            # Execute search action.
                            result = executable_subaction(
                                *(
                                    subparameters[1:]
                                    if hasattr(self.tables_demo_api.mobipick, subaction.action.name)
                                    else subparameters
                                )
                            )
                            subaction_execution_count += 1

                            if result is not None:
                                if result:
                                    # Note: True result only means any subaction succeeded.
                                    # Check if the search actually succeeded.
                                    if (
                                        self.env.item_search is None
                                        and len(self.env.newly_perceived_item_locations) <= 1
                                    ):
                                        print("- Continue with plan.")
                                        self.visualization.succeed(subaction_name)
                                        break
                                    # Check if the search found another item.
                                    elif self.env.newly_perceived_item_locations:
                                        self.env.newly_perceived_item_locations.clear()
                                        print("- Found another item, search ABORTED.")
                                        self.visualization.cancel(subaction_name)
                                        # Set result to None to trigger replanning.
                                        result = None
                                        break
                                else:
                                    self.visualization.fail(subaction_name)
                                    break
                        # Note: The conclude action at the end of any search always fails.
                    finally:
                        # Always end the search at this point.
                        self.env.item_search = None

                if result is not None:
                    if result:
                        retries_before_abortion = self.tables_demo_api.RETRIES_BEFORE_ABORTION
                        self.visualization.succeed(action_name)
                    else:
                        self.visualization.fail(action_name)
                        error_counts[self.domain.label(action)] += 1
                        # Note: This will also fail if two different failures occur successively.
                        if retries_before_abortion <= 0 or any(count >= 3 for count in error_counts.values()):
                            print("Task could not be completed even after retrying.")
                            self.visualization.add_node("Mission impossible", "red")
                            return

                        retries_before_abortion -= 1
                        actions = self.replan()
                        break
                else:
                    self.visualization.cancel(action_name)
                    retries_before_abortion = self.tables_demo_api.RETRIES_BEFORE_ABORTION
                    actions = self.replan()
                    break
            else:
                break
            if actions is None:
                print("Execution ended because no plan could be found.")
                break

        print("Demo complete.")
        self.visualization.add_node("Demo complete", "green")
