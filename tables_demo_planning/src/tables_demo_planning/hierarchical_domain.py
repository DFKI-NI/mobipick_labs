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
from unified_planning.shortcuts import Equals, Not
from unified_planning.model.metrics import MinimizeSequentialPlanLength
from tables_demo_planning.mobipick_components import ArmPose, Item
from tables_demo_planning.tables_demo_api import TablesDemoAPIDomain


class HierarchicalDomain(TablesDemoAPIDomain):
    def __init__(self) -> None:
        super().__init__()

        # TASKS
        self.drive = Task("drive", goal_pose=self.get_type(Pose))
        self.adapt_arm = Task("adapt_arm", to_pose=self.get_type(ArmPose))

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
        self.adapt_arm_op = Method("adapt_arm_op", from_pose=self.get_type(ArmPose), to_pose=self.get_type(ArmPose))
        adapt_arm_op_from = self.adapt_arm_op.parameter("from_pose")
        adapt_arm_op_to = self.adapt_arm_op.parameter("to_pose")
        self.adapt_arm_op.set_task(self.adapt_arm, adapt_arm_op_to)
        self.adapt_arm_op.add_precondition(self.robot_arm_at(adapt_arm_op_from))
        self.adapt_arm_op.add_precondition(Not(Equals(adapt_arm_op_from, adapt_arm_op_to)))
        self.adapt_arm_op.add_subtask(self.move_arm, self.robot, adapt_arm_op_from, adapt_arm_op_to)

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
                self.adapt_arm_op,
            ),
            tasks=(
                self.drive,
                self.adapt_arm,
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
