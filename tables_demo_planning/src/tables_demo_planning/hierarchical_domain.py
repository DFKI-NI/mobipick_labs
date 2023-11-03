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
# Authors:
#   Marc Vinci, DFKI
#   Sebastian Stock, DFKI

"""Addition of hierarchical methods and tasks to the Mobipick domain."""

import rospy
from typing import Iterable, Optional, Union, Set, Dict, List
from collections import defaultdict
import datetime
from geometry_msgs.msg import Pose
from unified_planning.model import Fluent, InstantaneousAction, Object, Action, Problem
from unified_planning.model.htn import HierarchicalProblem, Method, Task, Subtask
from unified_planning.shortcuts import Equals, Not, Or, OneshotPlanner
from unified_planning.plans import ActionInstance
from unified_planning.model.metrics import MinimizeSequentialPlanLength
from unified_planning.engines import OptimalityGuarantee
from tables_demo_planning.mobipick_components import ArmPose, Item, Location, ItemClass
from tables_demo_planning.tables_demo_api import TablesDemoAPIDomain


class HierarchicalDomain(TablesDemoAPIDomain):
    def __init__(self) -> None:
        super().__init__()

        # UP types
        type_pose = self.get_type(Pose)
        type_arm_pose = self.get_type(ArmPose)
        type_location = self.get_type(Location)
        type_item = self.get_type(Item)
        type_item_class = self.get_type(ItemClass)

        # TASKS
        self.drive = Task("drive", goal_pose=type_pose)
        self.adapt_arm = Task("adapt_arm", to_pose=type_arm_pose)
        self.perceive = Task("perceive", location=type_location)
        self.get_item = Task("get_item", item=type_item)
        self.put_item = Task("put_item", item=type_item, location=type_location)
        self.move_item = Task("move_item", item=type_item, location=type_location)
        self.insert_item = Task("insert_item", item=type_item, box=type_item)
        self.bring_item = Task("bring_item", item=type_item)
        self.search_item = Task("search_item", item=type_item)
        self.tables_demo = Task("tables_demo", tool=type_item, box=type_item, location=type_location)
        self.bring_item_of_class = Task("bring_item_of_class", item_type=type_item_class, location=type_location)

        # METHODS

        # DRIVE
        # robot already at goal pose
        self.drive_noop = Method("drive_noop", goal_pose=type_pose)
        self.drive_noop.set_task(self.drive, self.drive_noop.goal_pose)
        self.drive_noop.add_precondition(self.robot_at(self.drive_noop.goal_pose))

        # arm not holding anything, move arm to home pose and move base to goal location
        self.drive_homeposture = Method("drive_homeposture", start_pose=type_pose, goal_pose=type_pose)
        self.drive_homeposture.set_task(self.drive, self.drive_homeposture.goal_pose)
        self.drive_homeposture.add_precondition(self.robot_at(self.drive_homeposture.start_pose))
        self.drive_homeposture.add_precondition(self.robot_has(self.nothing))
        s1 = self.drive_homeposture.add_subtask(self.adapt_arm, self.arm_pose_home)
        s2 = self.drive_homeposture.add_subtask(
            self.move_base, self.robot, self.drive_homeposture.start_pose, self.drive_homeposture.goal_pose
        )
        self.drive_homeposture.set_ordered(s1, s2)

        # arm holding an item, move arm to transport pose and move base to goal location
        self.drive_transport = Method("drive_transport", item=type_item, start_pose=type_pose, goal_pose=type_pose)
        self.drive_transport.set_task(self.drive, self.drive_transport.goal_pose)
        self.drive_transport.add_precondition(self.robot_at(self.drive_transport.start_pose))
        self.drive_transport.add_precondition(self.robot_has(self.drive_transport.item))
        s1 = self.drive_transport.add_subtask(self.adapt_arm, self.arm_pose_transport)
        s2 = self.drive_transport.add_subtask(
            self.move_base_with_item,
            self.robot,
            self.drive_transport.item,
            self.drive_transport.start_pose,
            self.drive_transport.goal_pose,
        )
        self.drive_transport.set_ordered(s1, s2)

        # ADAPT ARM
        # arm already in goal arm pose
        self.adapt_arm_noop = Method("adapt_arm_noop", to_pose=type_arm_pose)
        self.adapt_arm_noop.set_task(self.adapt_arm, self.adapt_arm_noop.to_pose)
        self.adapt_arm_noop.add_precondition(self.robot_arm_at(self.adapt_arm_noop.to_pose))

        # move arm to goal arm pose
        self.adapt_arm_full = Method("adapt_arm_full", from_pose=type_arm_pose, to_pose=type_arm_pose)
        self.adapt_arm_full.set_task(self.adapt_arm, self.adapt_arm_full.to_pose)
        self.adapt_arm_full.add_precondition(self.robot_arm_at(self.adapt_arm_full.from_pose))
        self.adapt_arm_full.add_precondition(Not(Equals(self.adapt_arm_full.from_pose, self.adapt_arm_full.to_pose)))
        self.adapt_arm_full.add_subtask(
            self.move_arm, self.robot, self.adapt_arm_full.from_pose, self.adapt_arm_full.to_pose
        )

        # PERCEIVE LOCATION
        # already perceived location
        self.perceive_noop = Method("perceive_noop", pose=type_pose, location=type_location)
        self.perceive_noop.set_task(self.perceive, self.perceive_noop.location)
        self.perceive_noop.add_precondition(self.searched_at(self.perceive_noop.location))

        # already at location, arm pose unknown
        self.perceive_move_arm = Method("perceive_move_arm", pose=type_pose, location=type_location)
        self.perceive_move_arm.set_task(self.perceive, self.perceive_move_arm.location)
        self.perceive_move_arm.add_precondition(self.robot_at(self.perceive_move_arm.pose))
        self.perceive_move_arm.add_precondition(
            Or(self.robot_arm_at(arm_pose) for arm_pose in (self.arm_pose_handover, self.arm_pose_unknown))
        )
        s1 = self.perceive_move_arm.add_subtask(self.adapt_arm, self.arm_pose_observe)
        s2 = self.perceive_move_arm.add_subtask(
            self.search_at, self.robot, self.perceive_move_arm.pose, self.perceive_move_arm.location
        )
        self.perceive_move_arm.set_ordered(s1, s2)

        # already at location, perceive location
        self.perceive_location = Method("perceive_location", pose=type_pose, location=type_location)
        self.perceive_location.set_task(self.perceive, self.perceive_location.location)
        self.perceive_location.add_precondition(self.robot_at(self.perceive_location.pose))
        self.perceive_location.add_subtask(
            self.search_at, self.robot, self.perceive_location.pose, self.perceive_location.location
        )

        # drive to location, perceive location
        self.perceive_full = Method("perceive_full", pose=type_pose, location=type_location)
        self.perceive_full.set_task(self.perceive, self.perceive_full.location)
        s1 = self.perceive_full.add_subtask(self.drive, self.perceive_full.pose)
        s2 = self.perceive_full.add_subtask(
            self.search_at, self.robot, self.perceive_full.pose, self.perceive_full.location
        )
        self.perceive_full.set_ordered(s1, s2)

        # GET ITEM
        # item already in robots gripper
        self.get_item_noop = Method("get_item_noop", item=type_item)
        self.get_item_noop.set_task(self.get_item, self.get_item_noop.item)
        self.get_item_noop.add_precondition(self.robot_has(self.get_item_noop.item))

        # not holding an item, robot already at item location, pick up item
        self.get_item_pick = Method("get_item_pick", item=type_item, location=type_location, pose=type_pose)
        self.get_item_pick.set_task(self.get_item, self.get_item_pick.item)
        self.get_item_pick.add_precondition(self.robot_at(self.get_item_pick.pose))
        self.get_item_pick.add_precondition(self.robot_has(self.nothing))
        self.get_item_pick.add_precondition(self.believe_item_at(self.get_item_pick.item, self.get_item_pick.location))
        self.get_item_pick.add_precondition(self.pose_at(self.get_item_pick.pose, self.get_item_pick.location))
        self.get_item_pick.add_subtask(
            self.pick_item, self.robot, self.get_item_pick.pose, self.get_item_pick.location, self.get_item_pick.item
        )

        # not holding an item, go to item location and pick up item
        self.get_item_full = Method("get_item_full", item=type_item, location=type_location, pose=type_pose)
        self.get_item_full.set_task(self.get_item, self.get_item_full.item)
        self.get_item_full.add_precondition(self.robot_has(self.nothing))
        self.get_item_full.add_precondition(self.believe_item_at(self.get_item_full.item, self.get_item_full.location))
        self.get_item_full.add_precondition(self.pose_at(self.get_item_full.pose, self.get_item_full.location))
        s1 = self.get_item_full.add_subtask(self.drive, self.get_item_full.pose)
        s2 = self.get_item_full.add_subtask(
            self.pick_item, self.robot, self.get_item_full.pose, self.get_item_full.location, self.get_item_full.item
        )
        self.get_item_full.set_ordered(s1, s2)

        # PUT ITEM
        # item already at target location, robot is holding nothing
        self.put_item_noop = Method("put_item_noop", item=type_item, location=type_location, pose=type_pose)
        self.put_item_noop.set_task(self.put_item, self.put_item_noop.item, self.put_item_noop.location)
        self.put_item_noop.add_precondition(self.robot_has(self.nothing))
        self.put_item_noop.add_precondition(self.believe_item_at(self.put_item_noop.item, self.put_item_noop.location))

        # robot already at target pose, place item
        self.put_item_place = Method("put_item_place", item=type_item, location=type_location, pose=type_pose)
        self.put_item_place.set_task(self.put_item, self.put_item_place.item, self.put_item_place.location)
        self.put_item_place.add_precondition(self.robot_has(self.put_item_place.item))
        self.put_item_place.add_precondition(self.pose_at(self.put_item_place.pose, self.put_item_place.location))
        self.put_item_place.add_precondition(self.robot_at(self.put_item_place.pose))
        self.put_item_place.add_subtask(
            self.place_item,
            self.robot,
            self.put_item_place.pose,
            self.put_item_place.location,
            self.put_item_place.item,
        )

        # robot drives to target pose and places item
        self.put_item_full = Method("put_item_full", item=type_item, location=type_location, pose=type_pose)
        self.put_item_full.set_task(self.put_item, self.put_item_full.item, self.put_item_full.location)
        self.put_item_full.add_precondition(self.robot_has(self.put_item_full.item))
        self.put_item_full.add_precondition(self.pose_at(self.put_item_full.pose, self.put_item_full.location))
        s1 = self.put_item_full.add_subtask(self.drive, self.put_item_full.pose)
        s2 = self.put_item_full.add_subtask(
            self.place_item, self.robot, self.put_item_full.pose, self.put_item_full.location, self.put_item_full.item
        )
        self.put_item_full.set_ordered(s1, s2)

        # MOVE ITEM
        # item already at location, nothing to do
        self.move_item_noop = Method("move_item_noop", item=type_item, location=type_location)
        self.move_item_noop.set_task(self.move_item, self.move_item_noop.item, self.move_item_noop.location)
        self.move_item_noop.add_precondition(
            self.believe_item_at(self.move_item_noop.item, self.move_item_noop.location)
        )

        # move item from one location to another location
        self.move_item_full = Method("move_item_full", item=type_item, location=type_location)
        self.move_item_full.set_task(self.move_item, self.move_item_full.item, self.move_item_full.location)
        s1 = self.move_item_full.add_subtask(self.get_item, self.move_item_full.item)
        s2 = self.move_item_full.add_subtask(self.put_item, self.move_item_full.item, self.move_item_full.location)
        self.move_item_full.set_ordered(s1, s2)

        # INSERT ITEM
        # item already in box
        self.insert_item_noop = Method(
            "insert_item_noop",
            item=type_item,
            box=type_item,
            box_pose=type_pose,
            box_loc=type_location,
        )
        self.insert_item_noop.set_task(self.insert_item, self.insert_item_noop.item, self.insert_item_noop.box)
        self.insert_item_noop.add_precondition(self.believe_item_at(self.insert_item_noop.item, self.in_box))

        # already at box location, store item
        self.insert_item_store = Method(
            "insert_item_store",
            item=type_item,
            box=type_item,
            box_pose=type_pose,
            box_loc=type_location,
        )
        self.insert_item_store.set_task(self.insert_item, self.insert_item_store.item, self.insert_item_store.box)
        self.insert_item_store.add_precondition(Not(self.believe_item_at(self.insert_item_store.box, self.anywhere)))
        self.insert_item_store.add_precondition(
            self.pose_at(self.insert_item_store.box_pose, self.insert_item_store.box_loc)
        )
        self.insert_item_store.add_precondition(self.robot_has(self.insert_item_store.item))
        self.insert_item_store.add_precondition(self.robot_at(self.insert_item_store.box_pose))
        s1 = self.insert_item_store.add_subtask(self.drive, self.insert_item_store.box_pose)
        s2 = self.insert_item_store.add_subtask(
            self.store_item,
            self.robot,
            self.insert_item_store.box_pose,
            self.insert_item_store.box_loc,
            self.insert_item_store.item,
            self.insert_item_store.box,
        )
        self.insert_item_store.set_ordered(s1, s2)

        # already holding item, move to box and insert
        self.insert_item_drive = Method(
            "insert_item_drive",
            item=type_item,
            box=type_item,
            box_pose=type_pose,
            box_loc=type_location,
        )
        self.insert_item_drive.set_task(self.insert_item, self.insert_item_drive.item, self.insert_item_drive.box)
        self.insert_item_drive.add_precondition(Not(self.believe_item_at(self.insert_item_drive.box, self.anywhere)))
        self.insert_item_drive.add_precondition(
            self.pose_at(self.insert_item_drive.box_pose, self.insert_item_drive.box_loc)
        )
        self.insert_item_drive.add_precondition(self.robot_has(self.insert_item_drive.item))
        s1 = self.insert_item_drive.add_subtask(self.drive, self.insert_item_drive.box_pose)
        s2 = self.insert_item_drive.add_subtask(
            self.store_item,
            self.robot,
            self.insert_item_drive.box_pose,
            self.insert_item_drive.box_loc,
            self.insert_item_drive.item,
            self.insert_item_drive.box,
        )
        self.insert_item_drive.set_ordered(s1, s2)

        # go to item location, pickup item, go to box location, store item in box
        self.insert_item_full = Method(
            "insert_item_full",
            item=type_item,
            box=type_item,
            box_pose=type_pose,
            box_loc=type_location,
        )
        self.insert_item_full.set_task(self.insert_item, self.insert_item_full.item, self.insert_item_full.box)
        self.insert_item_full.add_precondition(Not(self.believe_item_at(self.insert_item_full.box, self.anywhere)))
        self.insert_item_full.add_precondition(
            self.pose_at(self.insert_item_full.box_pose, self.insert_item_full.box_loc)
        )
        s1 = self.insert_item_full.add_subtask(self.get_item, self.insert_item_full.item)
        s2 = self.insert_item_full.add_subtask(self.drive, self.insert_item_full.box_pose)
        s3 = self.insert_item_full.add_subtask(
            self.store_item,
            self.robot,
            self.insert_item_full.box_pose,
            self.insert_item_full.box_loc,
            self.insert_item_full.item,
            self.insert_item_full.box,
        )
        self.insert_item_full.set_ordered(s1, s2, s3)

        # BRING ITEM
        # robot already has item and is at handover pose, hand item over, move arm to home pose
        self.bring_item_handover = Method("bring_item_handover", item=type_item)
        self.bring_item_handover.set_task(self.bring_item, self.bring_item_handover.item)
        self.bring_item_handover.add_precondition(self.robot_has(self.bring_item_handover.item))
        self.bring_item_handover.add_precondition(self.robot_at(self.base_handover_pose))
        s1 = self.bring_item_handover.add_subtask(
            self.handover_item, self.robot, self.base_handover_pose, self.bring_item_handover.item
        )
        s2 = self.bring_item_handover.add_subtask(self.adapt_arm, self.arm_pose_home)
        self.bring_item_handover.set_ordered(s1, s2)

        # robot already has item, move to handover pose, hand item over, move arm to home pose
        self.bring_item_drive = Method("bring_item_drive", item=type_item)
        self.bring_item_drive.set_task(self.bring_item, self.bring_item_drive.item)
        self.bring_item_drive.add_precondition(self.robot_has(self.bring_item_drive.item))
        s1 = self.bring_item_drive.add_subtask(self.drive, self.base_handover_pose)
        s2 = self.bring_item_drive.add_subtask(
            self.handover_item, self.robot, self.base_handover_pose, self.bring_item_drive.item
        )
        s3 = self.bring_item_drive.add_subtask(self.adapt_arm, self.arm_pose_home)
        self.bring_item_drive.set_ordered(s1, s2, s3)

        # go to item location, pick up item, go to handover pose, hand item over, move arm to home pose
        self.bring_item_full = Method("bring_item_full", item=type_item)
        self.bring_item_full.set_task(self.bring_item, self.bring_item_full.item)
        s1 = self.bring_item_full.add_subtask(self.get_item, self.bring_item_full.item)
        s2 = self.bring_item_full.add_subtask(self.drive, self.base_handover_pose)
        s3 = self.bring_item_full.add_subtask(
            self.handover_item, self.robot, self.base_handover_pose, self.bring_item_full.item
        )
        s4 = self.bring_item_full.add_subtask(self.adapt_arm, self.arm_pose_home)
        self.bring_item_full.set_ordered(s1, s2, s3, s4)

        # SEARCH ITEM
        # item location already known, nothing to do
        self.search_item_noop = Method("search_item_noop", item=type_item)
        self.search_item_noop.set_task(self.search_item, self.search_item_noop.item)
        self.search_item_noop.add_precondition(Not(self.believe_item_at(self.search_item_noop.item, self.anywhere)))

        # full search
        self.search_item_full = Method("search_item_full", item=type_item)
        self.search_item_full.set_task(self.search_item, self.search_item_full.item)
        self.search_item_full.add_precondition(self.believe_item_at(self.search_item_full.item, self.anywhere))
        for location in self.TABLE_LOCATIONS:
            self.search_item_full.add_subtask(self.perceive, self.get_object(location))

        # TABLES DEMO
        # tool in box on target table, nothing to do
        self.tables_demo_noop = Method("tables_demo_noop", tool=type_item, box=type_item, location=type_location)
        self.tables_demo_noop.set_task(
            self.tables_demo, self.tables_demo_noop.tool, self.tables_demo_noop.box, self.tables_demo_noop.location
        )
        self.tables_demo_noop.add_precondition(self.believe_item_at(self.tables_demo_noop.tool, self.in_box))
        self.tables_demo_noop.add_precondition(self.robot_has(self.nothing))
        self.tables_demo_noop.add_precondition(
            self.believe_item_at(self.tables_demo_noop.box, self.tables_demo_noop.location)
        )

        # box location already known
        self.tables_demo_search_tool = Method(
            "tables_demo_search_tool",
            tool=type_item,
            box=type_item,
            location=type_location,
        )
        self.tables_demo_search_tool.set_task(
            self.tables_demo,
            self.tables_demo_search_tool.tool,
            self.tables_demo_search_tool.box,
            self.tables_demo_search_tool.location,
        )
        self.tables_demo_search_tool.add_precondition(
            Not(self.believe_item_at(self.tables_demo_search_tool.box, self.anywhere))
        )
        self.tables_demo_search_tool.add_precondition(
            Not(self.believe_item_at(self.tables_demo_search_tool.box, self.tables_demo_search_tool.location))
        )
        s1 = self.tables_demo_search_tool.add_subtask(self.search_tool, self.robot, self.tables_demo_search_tool.tool)
        s2 = self.tables_demo_search_tool.add_subtask(self.get_item, self.tables_demo_search_tool.tool)
        s3 = self.tables_demo_search_tool.add_subtask(
            self.insert_item, self.tables_demo_search_tool.tool, self.tables_demo_search_tool.box
        )
        s4 = self.tables_demo_search_tool.add_subtask(
            self.move_item, self.tables_demo_search_tool.box, self.tables_demo_search_tool.location
        )
        self.tables_demo_search_tool.set_ordered(s1, s2, s3, s4)

        # tool location already known
        self.tables_demo_search_box = Method(
            "tables_demo_search_box", tool=type_item, box=type_item, location=type_location
        )
        self.tables_demo_search_box.set_task(
            self.tables_demo,
            self.tables_demo_search_box.tool,
            self.tables_demo_search_box.box,
            self.tables_demo_search_box.location,
        )
        self.tables_demo_search_box.add_precondition(
            Not(self.believe_item_at(self.tables_demo_search_box.tool, self.anywhere))
        )
        s1 = self.tables_demo_search_box.add_subtask(self.get_item, self.tables_demo_search_box.tool)
        s2 = self.tables_demo_search_box.add_subtask(self.search_box, self.robot)
        s3 = self.tables_demo_search_box.add_subtask(
            self.insert_item, self.tables_demo_search_box.tool, self.tables_demo_search_box.box
        )
        s4 = self.tables_demo_search_box.add_subtask(
            self.move_item, self.tables_demo_search_box.box, self.tables_demo_search_box.location
        )
        self.tables_demo_search_box.set_ordered(s1, s2, s3, s4)

        # tool and box location already known, get item, insert into box and move to target table
        self.tables_demo_move_item = Method(
            "tables_demo_move_item", tool=type_item, box=type_item, location=type_location
        )
        self.tables_demo_move_item.set_task(
            self.tables_demo,
            self.tables_demo_move_item.tool,
            self.tables_demo_move_item.box,
            self.tables_demo_move_item.location,
        )
        self.tables_demo_move_item.add_precondition(
            Not(self.believe_item_at(self.tables_demo_move_item.tool, self.anywhere))
        )
        self.tables_demo_move_item.add_precondition(
            Not(self.believe_item_at(self.tables_demo_move_item.box, self.anywhere))
        )
        s1 = self.tables_demo_move_item.add_subtask(self.get_item, self.tables_demo_move_item.tool)
        s2 = self.tables_demo_move_item.add_subtask(
            self.insert_item, self.tables_demo_move_item.tool, self.tables_demo_move_item.box
        )
        s3 = self.tables_demo_move_item.add_subtask(
            self.move_item, self.tables_demo_move_item.box, self.tables_demo_move_item.location
        )
        self.tables_demo_move_item.set_ordered(s1, s2, s3)

        # full tables demo
        self.tables_demo_full = Method("tables_demo_full", tool=type_item, box=type_item, location=type_location)
        self.tables_demo_full.set_task(
            self.tables_demo, self.tables_demo_full.tool, self.tables_demo_full.box, self.tables_demo_full.location
        )
        s1 = self.tables_demo_full.add_subtask(self.search_tool, self.robot, self.tables_demo_full.tool)
        s2 = self.tables_demo_full.add_subtask(self.get_item, self.tables_demo_full.tool)
        s3 = self.tables_demo_full.add_subtask(self.search_box, self.robot)
        s4 = self.tables_demo_full.add_subtask(self.insert_item, self.tables_demo_full.tool, self.tables_demo_full.box)
        s5 = self.tables_demo_full.add_subtask(
            self.move_item, self.tables_demo_full.box, self.tables_demo_full.location
        )
        self.tables_demo_full.set_ordered(s1, s2, s3, s4, s5)

        # BRING_ITEM_OF_CLASS
        # no object of type is known - search and replan
        self.m_bring_item_of_class_search = Method(
            "m_bring_item_of_class_search",
            item_class=type_item_class,
            location=type_location,
            search_table=type_location,
        )
        self.m_bring_item_of_class_search.set_task(
            self.bring_item_of_class,
            self.m_bring_item_of_class_search.item_class,
            self.m_bring_item_of_class_search.location,
        )
        s1 = self.m_bring_item_of_class_search.add_subtask(
            self.perceive, self.m_bring_item_of_class_search.search_table
        )
        self.m_bring_item_of_class_search.add_precondition(
            Not(self.searched_at(self.m_bring_item_of_class_search.search_table))
        )
        s1 = self.m_bring_item_of_class_search.add_subtask(
            self.perceive, self.m_bring_item_of_class_search.search_table
        )
        s2 = self.m_bring_item_of_class_search.add_subtask(self.trigger_replanning, self.robot)
        self.m_bring_item_of_class_search.set_ordered(s1, s2)

        # object is known - move it to the target
        self.m_bring_item_of_class_move = Method(
            "m_bring_item_of_class_move",
            item_class=type_item_class,
            location=type_location,
            search_table=type_location,
            item=type_item,
            item_loc=type_location,
        )
        self.m_bring_item_of_class_move.set_task(
            self.bring_item_of_class,
            self.m_bring_item_of_class_move.item_class,
            self.m_bring_item_of_class_move.location,
        )
        self.m_bring_item_of_class_move.add_precondition(
            self.has_class(self.m_bring_item_of_class_move.item, self.m_bring_item_of_class_move.item_class)
        )
        self.m_bring_item_of_class_move.add_precondition(
            self.believe_item_at(self.m_bring_item_of_class_move.item, self.m_bring_item_of_class_move.item_loc)
        )
        # TODO do we need to make sure that it is not 'anywhere'?
        self.m_bring_item_of_class_move.add_subtask(
            self.move_item, self.m_bring_item_of_class_move.item, self.m_bring_item_of_class_move.location
        )

        # object is in a klt - move klt to the target
        self.m_bring_item_of_class_move_in_klt = Method(
            "m_bring_item_of_class_move_in_klt",
            item_class=type_item_class,
            location=type_location,
            search_table=type_location,
            item=type_item,
            klt=type_item,
        )
        self.m_bring_item_of_class_move_in_klt.set_task(
            self.bring_item_of_class,
            self.m_bring_item_of_class_move_in_klt.item_class,
            self.m_bring_item_of_class_move_in_klt.location,
        )
        self.m_bring_item_of_class_move_in_klt.add_precondition(
            self.has_class(
                self.m_bring_item_of_class_move_in_klt.item, self.m_bring_item_of_class_move_in_klt.item_class
            )
        )
        self.m_bring_item_of_class_move_in_klt.add_precondition(
            self.believe_item_in(
                self.m_bring_item_of_class_move_in_klt.item, self.m_bring_item_of_class_move_in_klt.klt
            )
        )
        self.m_bring_item_of_class_move_in_klt.add_subtask(
            self.move_item, self.m_bring_item_of_class_move_in_klt.klt, self.m_bring_item_of_class_move_in_klt.location
        )

        self.problem = self.define_mobipick_problem(
            fluents=(
                self.robot_at,
                self.robot_arm_at,
                self.robot_has,
                self.believe_item_at,
                self.pose_at,
                self.item_offered,
                self.searched_at,
                self.believe_item_in,
                self.has_class,
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
                self.trigger_replanning,
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
                self.tables_demo_search_box,
                self.tables_demo_search_tool,
                self.tables_demo_move_item,
                self.tables_demo_full,
                self.m_bring_item_of_class_search,
                self.m_bring_item_of_class_move,
                self.m_bring_item_of_class_move_in_klt,
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
                self.bring_item_of_class,
            ),
        )
        self.problem.add_quality_metric(MinimizeSequentialPlanLength())

        self.subproblem = self.define_mobipick_problem(
            fluents=(
                self.robot_at,
                self.robot_arm_at,
                self.robot_has,
                self.believe_item_at,
                self.searched_at,
                self.pose_at,
            ),
            actions=(
                self.move_base,
                self.move_base_with_item,
                self.move_arm,
                self.search_at,
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
        tasks: Optional[Iterable[Task]] = None,
        methods: Optional[Iterable[Method]] = None,
        poses: Optional[Iterable[Object]] = None,
        items: Optional[Iterable[Object]] = None,
        locations: Optional[Iterable[Object]] = None,
        item_classes: Optional[Iterable[Object]] = None,
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
            + (self.item_classes if item_classes is None else list(item_classes))
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

    def solve_problem(self, problem: Problem) -> Optional[List[ActionInstance]]:
        """Solve planning problem and return plan."""
        result = OneshotPlanner(
            name="aries", problem_kind=problem.kind, optimality_guarantee=OptimalityGuarantee.SOLVED_OPTIMALLY
        ).solve(problem)
        if result.status and result.status.value > 2:
            if result.log_messages:
                rospy.logerr(f"Error during plan generation: {result.log_messages[0].message}")
            else:
                rospy.logerr(f"Error during plan generation: {result.status}")
        print(result.plan.actions)
        return result.plan.action_plan.actions if result.plan and result.plan.action_plan else None

    def write_problem(self) -> None:
        """Write problem into file with current timestamp"""
        # Write self.problem to file with current timestamp as filename
        timestamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        filename = f"/tmp/problem_{timestamp}.txt"
        with open(filename, "w") as f:
            f.write(str(self.problem))

    def replan(self) -> Optional[List[ActionInstance]]:
        """Print believed item locations, initialize UP problem, and solve it."""
        self.env.print_believed_item_locations()
        self.set_initial_values(self.problem)
        self.write_problem()
        return self.solve_problem(self.problem)

    def set_task(self, problem: HierarchicalProblem, task: Union[Task, Subtask, Action]) -> None:
        """Set the task to be executed."""
        problem.task_network.add_subtask(task)

    def clear_tasks(self, problem: HierarchicalProblem) -> None:
        problem.task_network._subtasks.clear()

    def run(self, target_item: Object, target_box: Object, target_location: Object) -> None:
        """Run the mobipick tables demo."""
        executed_action_names: Set[str] = set()  # Note: For visualization purposes only.
        retries_before_abortion = self.RETRIES_BEFORE_ABORTION
        error_counts: Dict[str, int] = defaultdict(int)
        # Solve overall problem.
        self.clear_tasks(self.problem)
        # self.set_task(self.problem, self.tables_demo(target_item, target_box, target_location))
        self.set_task(self.problem, self.bring_item_of_class(self.get_object(ItemClass.multimeter), target_location))
        self.env.robot.update_facts()
        actions = self.replan()
        if actions is None:
            print("Execution ended because no plan could be found.")
            return

        # Loop action execution as long as there are actions.
        while actions:
            print("> Plan:")
            print('\n'.join(map(str, actions)))
            if self.visualization:
                self.visualization.set_actions(
                    [
                        f"{number + len(executed_action_names)} {self.label(action)}"
                        for number, action in enumerate(actions, start=1)
                    ],
                    preserve_actions=executed_action_names,
                )
            print("> Execution:")
            for action in actions:
                if action.action.name == "trigger_replanning":
                    actions = self.replan()
                    break
                executable_action, parameters = self.get_executable_action(action)
                action_name = f"{len(executed_action_names) + 1} {self.label(action)}"
                print(action)
                # Explicitly do not pick up box from target_table since planning does not handle it yet.
                if action.action.name == "pick_item" and parameters[-1].name.startswith("klt_"):
                    assert isinstance(parameters[-2], Location)
                    location = self.env.resolve_search_location(parameters[-2])
                    if location == Location(target_location.name):
                        print("Picking up box OBSOLETE.")
                        if self.visualization:
                            self.visualization.cancel(action_name)
                        actions = self.replan()
                        break

                if self.visualization:
                    self.visualization.execute(action_name)
                if self.espeak_pub:
                    self.espeak_pub.publish(self.label(action))

                # Execute action.
                result = executable_action(*parameters)
                executed_action_names.add(action_name)
                if rospy.is_shutdown():
                    return

                # Handle item search as an inner execution loop.
                # Rationale: It has additional stop criteria, and might continue the outer loop.
                if self.env.item_search:
                    try:
                        # Check whether an obsolete item search invalidates the previous plan.
                        if self.env.believed_item_locations.get(self.env.item_search, self.anywhere) != self.anywhere:
                            print(f"Search for {self.env.item_search.name} OBSOLETE.")
                            if self.visualization:
                                self.visualization.cancel(action_name)
                            actions = self.replan()
                            break

                        # Search for item by creating and executing a subplan.
                        self.clear_tasks(self.subproblem)
                        self.set_initial_values(self.subproblem)
                        self.set_task(self.subproblem, self.search_item(self.get_object(self.env.item_search)))
                        subactions = self.solve_problem(self.subproblem)
                        assert subactions, f"No solution for: {self.subproblem}"
                        print("- Search plan:")
                        print('\n'.join(map(str, subactions)))
                        if self.visualization:
                            self.visualization.set_actions(
                                [
                                    f"{len(executed_action_names)}{chr(number)} {self.label(subaction)}"
                                    for number, subaction in enumerate(subactions, start=97)
                                ],
                                preserve_actions=executed_action_names,
                                predecessor=action_name,
                            )
                        print("- Search execution:")
                        subaction_execution_count = 0
                        for subaction in subactions:
                            executable_subaction, subparameters = self.get_executable_action(subaction)
                            subaction_name = (
                                f"{len(executed_action_names)}{chr(subaction_execution_count + 97)}"
                                f" {self.label(subaction)}"
                            )
                            print(subaction)
                            if self.visualization:
                                self.visualization.execute(subaction_name)
                            if self.espeak_pub:
                                self.espeak_pub.publish(self.label(subaction))
                            # Execute search action.
                            result = executable_subaction(*subparameters)
                            subaction_execution_count += 1
                            if rospy.is_shutdown():
                                return

                            if result is not None:
                                if result:
                                    # Note: True result only means any subaction succeeded.
                                    # Check if the search actually succeeded.
                                    if (
                                        self.env.item_search is None
                                        and len(self.env.newly_perceived_item_locations) <= 1
                                    ):
                                        print("- Continue with plan.")
                                        if self.visualization:
                                            self.visualization.succeed(subaction_name)
                                        break
                                    # Check if the search found another item.
                                    elif self.env.newly_perceived_item_locations:
                                        self.env.newly_perceived_item_locations.clear()
                                        print("- Found another item, search ABORTED.")
                                        if self.visualization:
                                            self.visualization.cancel(subaction_name)
                                        if self.espeak_pub:
                                            self.espeak_pub.publish("Found another item. Make a new plan.")
                                        # Set result to None to trigger replanning.
                                        result = None
                                        break
                                else:
                                    if self.visualization:
                                        self.visualization.fail(subaction_name)
                                    break
                        # Note: The conclude action at the end of any search always fails.
                    finally:
                        # Always end the search at this point.
                        self.env.item_search = None

                if result is not None:
                    if result:
                        if self.visualization:
                            self.visualization.succeed(action_name)
                        retries_before_abortion = self.RETRIES_BEFORE_ABORTION
                    else:
                        if self.visualization:
                            self.visualization.fail(action_name)
                        if self.espeak_pub:
                            self.espeak_pub.publish("Action failed.")
                        error_counts[self.label(action)] += 1
                        # Note: This will also fail if two different failures occur successively.
                        if retries_before_abortion <= 0 or any(count >= 3 for count in error_counts.values()):
                            print("Task could not be completed even after retrying.")
                            if self.visualization:
                                self.visualization.add_node("Mission impossible", "red")
                            if self.espeak_pub:
                                self.espeak_pub.publish("Mission impossible!")
                            return

                        retries_before_abortion -= 1
                        actions = self.replan()
                        break
                else:
                    if self.visualization:
                        self.visualization.cancel(action_name)
                    retries_before_abortion = self.RETRIES_BEFORE_ABORTION
                    actions = self.replan()
                    break
            else:
                break
            if actions is None:
                print("Execution ended because no plan could be found.")
                break

        print("Demo complete.")
        if self.visualization:
            self.visualization.add_node("Demo complete", "green")
        if self.espeak_pub:
            self.espeak_pub.publish("Demo complete.")
