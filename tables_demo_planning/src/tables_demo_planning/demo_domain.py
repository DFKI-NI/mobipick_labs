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
# Authors: Alexander Sung, DFKI

"""A concrete Mobipick domain, connecting its application to its planning representations"""


from typing import Callable, Dict, Generic, Iterable, List, Optional, Sequence, TypeVar, Union
import time
import yaml
import rospy
import rosparam
from geometry_msgs.msg import Pose
from unified_planning.engines import OptimalityGuarantee
from unified_planning.model import Fluent, InstantaneousAction, Object, Problem
from unified_planning.plans import ActionInstance
from unified_planning.shortcuts import Equals, Not, OneshotPlanner
from up_esb.bridge import Bridge
from tables_demo_planning.mobipick_components import (
    ArmPose,
    EnvironmentRepresentation,
    Item,
    Location,
    Robot,
    ItemClass,
)
from robot_api import TuplePose, is_instance


# Note: A Generic cannot be a TypeVar of another Generic.
#  See https://github.com/python/mypy/issues/2756.
E = TypeVar('E', bound=EnvironmentRepresentation)


class Domain(Bridge, Generic[E]):
    BASE_HANDOVER_POSE_NAME = "base_handover_pose"
    BASE_HOME_POSE_NAME = "base_home_pose"
    BASE_TABLE_1_POSE_NAME = "base_table_1_pose"
    BASE_TABLE_2_POSE_NAME = "base_table_2_pose"
    BASE_TABLE_3_POSE_NAME = "base_table_3_pose"
    TOOL_SEARCH_POSE_NAME = "tool_search_pose"
    BOX_SEARCH_POSE_NAME = "box_search_pose"
    UNKNOWN_POSE_NAME = "unknown_pose"

    def __init__(self, env: E) -> None:
        Bridge.__init__(self)
        self.env = env
        # Create types for planning based on class types.
        self.create_types([Robot, Pose, ArmPose, Item, Location, ItemClass])

        # Create fluents for planning.
        self.robot_at = self.create_fluent("get_robot_at", pose=Pose)
        self.robot_arm_at = self.create_fluent_from_function(env.get_robot_arm_at)
        self.robot_has = self.create_fluent_from_function(env.get_robot_has)

        # Create objects for both planning and execution.
        self.robot = self.create_object("mobipick", env.robot)
        rosparam_namespace = "/mobipick/tables_demo_planning"
        self.api_poses: Dict[str, Pose] = {}
        for param_path in rosparam.list_params(rosparam_namespace):
            assert isinstance(param_path, str)
            param = rosparam.get_param(param_path)
            param_name = param_path.rsplit('/', maxsplit=1)[-1]
            if is_instance(param, Sequence[Sequence[Union[float, int]]]) and tuple(map(len, param)) == (3, 4):
                self.api_poses[param_name] = TuplePose.to_pose(param)
        if len({TuplePose.from_pose(pose) for pose in self.api_poses.values()}) < len(self.api_poses):
            rospy.logwarn(
                f"Duplicate poses in rosparam namespace '{rosparam_namespace}'"
                " might let checks fail whether something is at a specific symbolic pose."
            )
        self.poses = self.create_objects(self.api_poses)
        self.base_home_pose = self.objects[self.BASE_HOME_POSE_NAME]
        self.base_handover_pose = self.objects[self.BASE_HANDOVER_POSE_NAME]
        self.base_table_1_pose = self.objects[self.BASE_TABLE_1_POSE_NAME]
        self.base_table_2_pose = self.objects[self.BASE_TABLE_2_POSE_NAME]
        self.base_table_3_pose = self.objects[self.BASE_TABLE_3_POSE_NAME]
        self.tool_search_pose = self.create_object(self.TOOL_SEARCH_POSE_NAME, Pose())
        self.box_search_pose = self.create_object(self.BOX_SEARCH_POSE_NAME, Pose())
        self.unknown_pose = self.create_object(self.UNKNOWN_POSE_NAME, Pose())
        self.poses.extend([self.tool_search_pose, self.box_search_pose, self.unknown_pose])
        self.arm_poses = self.create_enum_objects(ArmPose)
        self.arm_pose_handover = self.objects[ArmPose.handover.name]
        self.arm_pose_home = self.objects[ArmPose.home.name]
        self.arm_pose_observe = self.objects[ArmPose.observe.name]
        self.arm_pose_transport = self.objects[ArmPose.transport.name]
        self.arm_pose_unknown = self.objects[ArmPose.unknown.name]
        items = [
            Item.NOTHING,
            Item.get("power_drill_with_grip_1"),
            Item.get("klt_1"),
            Item.get("klt_2"),
            Item.get("multimeter_1"),
            Item.get("relay_1"),
            Item.get("screwdriver_1"),
        ]
        self.items = self.create_objects({item.name: item for item in items})
        self.nothing = self.objects["nothing"]
        self.power_drill = self.objects["power_drill_with_grip_1"]
        self.box = self.objects["klt_1"]
        self.multimeter = self.objects["multimeter_1"]
        self.relay = self.objects["relay_1"]
        self.screwdriver = self.objects["screwdriver_1"]
        self.locations = self.create_enum_objects(Location)
        self.anywhere = self.objects[Location.anywhere.name]
        self.table_1 = self.objects[Location.table_1.name]
        self.table_2 = self.objects[Location.table_2.name]
        self.table_3 = self.objects[Location.table_3.name]
        self.tables = (self.table_1, self.table_2, self.table_3)
        self.in_box = self.objects[Location.in_box.name]
        self.on_robot = self.objects[Location.on_robot.name]
        self.tool_search_location = self.objects[Location.tool_search_location.name]
        self.box_search_location = self.objects[Location.box_search_location.name]
        self.item_classes = self.create_enum_objects(ItemClass)

        # Create actions for planning based on class definitions.
        self.move_base, (_, x, y) = self.create_action_from_function(Robot.move_base, set_callable=False)
        self.move_base.add_precondition(self.robot_at(x))
        self.move_base.add_precondition(self.robot_has(self.nothing))
        self.move_base.add_precondition(self.robot_arm_at(self.arm_pose_home))
        self.move_base.add_effect(self.robot_at(x), False)
        self.move_base.add_effect(self.robot_at(y), True)
        self.move_base_with_item, (_, item, x, y) = self.create_action_from_function(
            Robot.move_base_with_item, set_callable=False
        )
        self.move_base_with_item.add_precondition(self.robot_at(x))
        self.move_base_with_item.add_precondition(self.robot_has(item))
        self.move_base_with_item.add_precondition(Not(Equals(item, self.nothing)))
        self.move_base_with_item.add_precondition(self.robot_arm_at(self.arm_pose_transport))
        self.move_base_with_item.add_effect(self.robot_at(x), False)
        self.move_base_with_item.add_effect(self.robot_at(y), True)
        self.move_arm, (_, x, y) = self.create_action_from_function(Robot.move_arm, set_callable=False)
        self.move_arm.add_precondition(self.robot_arm_at(x))
        self.move_arm.add_effect(self.robot_arm_at(x), False)
        self.move_arm.add_effect(self.robot_arm_at(y), True)

        # Create visualization labels for actions as functions of their parameters.
        self.method_labels: Dict[InstantaneousAction, Callable[[Sequence[str]], str]] = {
            self.move_base: lambda parameters: f"Move to {parameters[-1]}",
            self.move_base_with_item: lambda parameters: f"Transport {parameters[1]} to {parameters[-1]}",
            self.move_arm: lambda parameters: f"Move arm to its {parameters[-1]} pose",
        }
        self.parameter_labels: Dict[Object, str] = {
            self.base_home_pose: "home",
            self.base_handover_pose: "handover",
            self.base_table_1_pose: "table_1",
            self.base_table_2_pose: "table_2",
            self.base_table_3_pose: "table_3",
            self.tool_search_pose: "where tool has been found",
            self.box_search_pose: "where box has been found",
        }

    @staticmethod
    def load_waypoints(filepath: str) -> Dict[str, Pose]:
        """Load poses from config file."""
        poses: Dict[str, Pose] = {}
        with open(filepath, 'r') as yaml_file:
            yaml_contents: Dict[str, List[float]] = yaml.safe_load(yaml_file)["poses"]
            for pose_name in sorted(yaml_contents.keys()):
                if pose_name.startswith("base_") and pose_name.endswith("_pose"):
                    poses[pose_name] = TuplePose.to_pose(yaml_contents[pose_name])
        return poses

    def define_mobipick_problem(
        self,
        fluents: Optional[Iterable[Fluent]] = None,
        actions: Optional[Iterable[InstantaneousAction]] = None,
        poses: Optional[Iterable[Object]] = None,
        items: Optional[Iterable[Object]] = None,
        locations: Optional[Iterable[Object]] = None,
        item_classes: Optional[Iterable[Object]] = None,
    ) -> Problem:
        """Define UP problem by its (potential subsets of) fluents, actions, and objects."""
        return self.define_problem(
            fluents,
            actions,
            objects=set(
                [self.robot, self.unknown_pose, self.nothing]
                + (self.poses if poses is None else list(poses))
                + self.arm_poses
                + (self.items if items is None else list(items))
                + (self.locations if locations is None else list(locations))
                + (self.item_classes if item_classes is None else list(item_classes))
            ),
        )

    def solve(self, problem: Problem) -> Optional[List[ActionInstance]]:
        """Solve planning problem and return list of UP actions."""
        print("Calculating plan ...")
        start_time = time.time()
        result = OneshotPlanner(
            problem_kind=problem.kind, optimality_guarantee=OptimalityGuarantee.SOLVED_OPTIMALLY
        ).solve(problem)
        rospy.loginfo(f"Result received from '{result.engine_name}' planner after {time.time() - start_time} seconds.")
        return result.plan.actions if result.plan else None

    def label(self, action: ActionInstance) -> str:
        """Return a user-friendly label for visualizing action."""
        parameters = [parameter.object() for parameter in action.actual_parameters]
        parameter_labels = [self.parameter_labels.get(parameter, str(parameter)) for parameter in parameters]
        return self.method_labels[action.action](parameter_labels)
