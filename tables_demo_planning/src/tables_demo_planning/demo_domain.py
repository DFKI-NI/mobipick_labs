from typing import Callable, Dict, List, Optional, Sequence, Tuple, Type
from enum import Enum
import math
import os
import time
import yaml
import rospy
import rospkg
from geometry_msgs.msg import Pose
from unified_planning.model.action import InstantaneousAction
from unified_planning.model.fluent import Fluent
from unified_planning.model.object import Object
from unified_planning.model.problem import Problem
from unified_planning.plans.plan import ActionInstance
from unified_planning.shortcuts import Equals, Not, OneshotPlanner
from tables_demo_planning.planning_bridge import Bridge
from robot_api import TuplePose
import robot_api

"""
Define types, objects, and actions for the mobipick_tables_demo domain,
both for planning and for execution.
"""


# Define state representation classes.


class ArmPose(Enum):
    unknown = "unknown"
    home = "home"
    observe = "observe100cm_right"
    transport = "transport"
    place = "place_1"
    tucked = "tucked"
    interaction = "interaction"  # not a pose in rosparams but used to simplify planning


class Item(Enum):
    nothing = "nothing"
    power_drill = "power_drill_with_grip_1"
    box = "klt_1"
    multimeter = "multimeter_1"
    relay = "relay_1"
    screwdriver = "screwdriver_1"
    something = "something"


class Location(Enum):
    anywhere = "anywhere"
    table_1 = "table_1"
    table_2 = "table_2"
    table_3 = "table_3"
    in_box = "in_box"
    on_robot = "on_robot"
    tool_search_location = "tool_search_location"
    box_search_location = "box_search_location"


class Robot(robot_api.Robot):
    """Robot representation which maintains the current state and enables action execution via Robot API."""

    def __init__(self, namespace: str) -> None:
        super().__init__(namespace, True, True)
        self.pose: Pose = TuplePose.to_pose(self.base.get_pose())
        self.arm_pose = self.get_arm_pose()
        self.item = self.get_initial_item()
        self.item_offered = False

    @staticmethod
    def enum_values(enum: Type[Enum]) -> Tuple[str, ...]:
        return tuple(member.value for member in enum)

    def get_arm_pose(self) -> ArmPose:
        arm_pose_name = self.arm.get_pose_name()
        return ArmPose(arm_pose_name) if arm_pose_name in self.enum_values(ArmPose) else ArmPose.unknown

    def get_initial_item(self) -> Item:
        self.arm.execute("HasAttachedObjects")
        return Item.something if self.arm.get_result().result else Item.nothing

    def move_base(self, _: Pose, pose: Pose) -> bool:
        if self.base.move(pose) != 3:
            rospy.logerr(f"Move base to {pose} FAILED!")
            # Move to home pose whenever movement fails. Note: This is a drastic workaround.
            self.base.move(Domain.BASE_HOME_POSE)
            return False

        self.pose = pose
        return True

    def move_base_with_item(self, item: Item, _: Pose, pose: Pose) -> bool:
        # Note: Same action as move_base(), just with item in transport pose.
        return self.move_base(_, pose)

    def move_arm(self, _: ArmPose, arm_pose: ArmPose) -> bool:
        if not self.arm.move(arm_pose.name):
            rospy.logerr(f"Move arm to '{arm_pose.name} FAILED!'")
            return False

        self.arm_pose = arm_pose
        return True


# Define domain for both planning and execution.


class Domain(Bridge):
    BASE_HANDOVER_POSE_NAME = "base_handover_pose"
    BASE_HOME_POSE_NAME = "base_home_pose"
    BASE_PICK_POSE_NAME = "base_pick_pose"
    BASE_PLACE_POSE_NAME = "base_place_pose"
    BASE_TABLE_1_POSE = "base_table_1_pose"
    BASE_TABLE_2_POSE = "base_table_2_pose"
    BASE_TABLE_3_POSE = "base_table_3_pose"
    BASE_HOME_POSE: Pose

    def __init__(self, robot: Robot) -> None:
        super().__init__()
        # Create types for planning based on class types.
        self.create_types([Robot, Pose, ArmPose, Item, Location])

        # Create fluents for planning.
        self.robot_at = self.create_fluent("At", [Robot, Pose])
        self.robot_arm_at = self.create_fluent("ArmAt", [Robot, ArmPose])
        self.robot_has = self.create_fluent("Has", [Robot, Item])
        self.robot_offered = self.create_fluent("Offered", [Robot])
        self.believe_item_at = self.create_fluent("BelieveItemAt", [Item, Location])
        self.searched_at = self.create_fluent("SearchedAt", [Location])
        self.pose_at = self.create_fluent("PoseAt", [Pose, Location])

        # Create objects for both planning and execution.
        self.api_robot = robot
        self.robot = self.create_object("mobipick", self.api_robot)
        config_path = f"{rospkg.RosPack().get_path('mobipick_pick_n_place')}/config/"
        filename = "moelk_tables_demo.yaml"
        self.api_poses = self.load_waypoints(os.path.join(config_path, filename))
        Domain.BASE_HOME_POSE = self.api_poses[self.BASE_HOME_POSE_NAME]
        self.poses = self.create_objects(self.api_poses)
        self.base_home_pose = self.objects[self.BASE_HOME_POSE_NAME]
        self.base_handover_pose = self.objects[self.BASE_HANDOVER_POSE_NAME]
        self.base_pick_pose = self.objects[self.BASE_PICK_POSE_NAME]
        self.base_place_pose = self.objects[self.BASE_PLACE_POSE_NAME]
        self.base_table_1_pose = self.objects[self.BASE_TABLE_1_POSE]
        self.base_table_2_pose = self.objects[self.BASE_TABLE_2_POSE]
        self.base_table_3_pose = self.objects[self.BASE_TABLE_3_POSE]
        self.tool_search_pose = self.create_object("tool_search_pose", Pose())
        self.box_search_pose = self.create_object("box_search_pose", Pose())
        self.unknown_pose = self.create_object("unknown_pose", Pose())
        self.poses.extend([self.tool_search_pose, self.box_search_pose, self.unknown_pose])
        self.arm_poses = self.create_objects({pose.name: pose for pose in ArmPose})
        self.arm_pose_home = self.objects[ArmPose.home.name]
        self.arm_pose_observe = self.objects[ArmPose.observe.name]
        self.arm_pose_transport = self.objects[ArmPose.transport.name]
        self.arm_pose_interaction = self.objects[ArmPose.interaction.name]
        self.items = self.create_objects({item.name: item for item in Item})
        self.nothing = self.objects[Item.nothing.name]
        self.power_drill = self.objects[Item.power_drill.name]
        self.box = self.objects[Item.box.name]
        self.multimeter = self.objects[Item.multimeter.name]
        self.relay = self.objects[Item.relay.name]
        self.screwdriver = self.objects[Item.screwdriver.name]
        self.locations = self.create_objects({location.name: location for location in Location})
        self.anywhere = self.objects[Location.anywhere.name]
        self.table_1 = self.objects[Location.table_1.name]
        self.table_2 = self.objects[Location.table_2.name]
        self.table_3 = self.objects[Location.table_3.name]
        self.tables = (self.table_1, self.table_2, self.table_3)
        self.in_box = self.objects[Location.in_box.name]
        self.on_robot = self.objects[Location.on_robot.name]
        self.tool_search_location = self.objects[Location.tool_search_location.name]
        self.box_search_location = self.objects[Location.box_search_location.name]

        # Create actions for planning based on class definitions.
        self.move_base, (robot, x, y) = self.create_action(Robot.move_base)
        self.move_base.add_precondition(self.robot_at(robot, x))
        self.move_base.add_precondition(self.robot_has(robot, self.nothing))
        self.move_base.add_precondition(self.robot_arm_at(robot, self.arm_pose_home))
        self.move_base.add_effect(self.robot_at(robot, x), False)
        self.move_base.add_effect(self.robot_at(robot, y), True)
        self.move_base_with_item, (robot, item, x, y) = self.create_action(Robot.move_base_with_item)
        self.move_base_with_item.add_precondition(self.robot_at(robot, x))
        self.move_base_with_item.add_precondition(self.robot_has(robot, item))
        self.move_base_with_item.add_precondition(Not(Equals(item, self.nothing)))
        self.move_base_with_item.add_precondition(self.robot_arm_at(robot, self.arm_pose_transport))
        self.move_base_with_item.add_effect(self.robot_at(robot, x), False)
        self.move_base_with_item.add_effect(self.robot_at(robot, y), True)
        self.move_arm, (robot, x, y) = self.create_action(Robot.move_arm)
        self.move_arm.add_precondition(self.robot_arm_at(robot, x))
        self.move_arm.add_effect(self.robot_arm_at(robot, x), False)
        self.move_arm.add_effect(self.robot_arm_at(robot, y), True)

        # Create visualization labels for actions as functions of their parameters.
        self.method_labels: Dict[InstantaneousAction, Callable[[Sequence[str]], str]] = {
            self.move_base: lambda parameters: f"Move to {parameters[-1]}",
            self.move_base_with_item: lambda parameters: f"Transport {parameters[1]} to {parameters[-1]}",
            self.move_arm: lambda parameters: f"Move arm to its {parameters[-1]} pose",
        }
        self.parameter_labels: Dict[Object, str] = {
            self.base_home_pose: "home",
            self.base_handover_pose: "handover",
            self.base_pick_pose: "pick",
            self.base_place_pose: "place",
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
                    pose = yaml_contents[pose_name]
                    position, orientation = pose[:3], pose[4:] + [pose[3]]
                    poses[pose_name] = TuplePose.to_pose((position, orientation))
                    robot_api.add_waypoint(pose_name, (position, orientation))
        return poses

    def define_problem(
        self,
        fluents: Optional[Sequence[Fluent]] = None,
        poses: Optional[Sequence[Object]] = None,
        items: Optional[Sequence[Object]] = None,
        locations: Optional[Sequence[Object]] = None,
        actions: Optional[Sequence[InstantaneousAction]] = None,
    ) -> Problem:
        """Define a UP problem by its (potential subsets of) fluents, objects, and actions."""
        problem = Problem()
        for fluent in self._fluents.values() if fluents is None else fluents:
            problem.add_fluent(fluent, default_initial_value=False)
        problem.add_object(self.robot)
        problem.add_objects(self.poses if poses is None else poses)
        problem.add_objects(self.arm_poses)
        problem.add_objects(self.items if items is None else items)
        problem.add_objects(self.locations if locations is None else locations)
        for action in self._actions.values() if actions is None else actions:
            problem.add_action(action)
        return problem

    def solve(
        self, problem: Problem
    ) -> Optional[Sequence[Tuple[ActionInstance, Tuple[Callable[..., object], List[object]]]]]:
        """Solve planning problem, then return list of UP and Robot API actions."""
        print("Calculating plan ...")
        start_time = time.time()
        result = OneshotPlanner(problem_kind=problem.kind, optimality_guarantee='SOLVED_OPTIMALLY').solve(problem)
        rospy.loginfo(f"Result received from '{result.engine_name}' planner after {time.time() - start_time} seconds.")
        return [(action, self.get_executable_action(action)) for action in result.plan.actions] if result.plan else None

    def set_initial_values(self, problem: Problem) -> None:
        base_pose_name = self.api_robot.base.get_pose_name()
        base_pose = self.objects[base_pose_name] if base_pose_name else self.unknown_pose
        if self.robot_at in problem.fluents:
            for pose in self.poses:
                problem.set_initial_value(self.robot_at(self.robot, pose), pose == base_pose)
        if self.robot_arm_at in problem.fluents:
            for arm_pose in self.arm_poses:
                problem.set_initial_value(
                    self.robot_arm_at(self.robot, arm_pose), arm_pose == self.objects[self.api_robot.arm_pose.name]
                )
        if self.robot_has in problem.fluents:
            for item in self.items:
                problem.set_initial_value(
                    self.robot_has(self.robot, item), item == self.objects[self.api_robot.item.name]
                )
        if self.robot_offered in problem.fluents:
            problem.set_initial_value(self.robot_offered(self.robot), self.api_robot.item_offered)
        if self.believe_item_at in problem.fluents:
            for item in self.items:
                for location in self.locations:
                    problem.set_initial_value(self.believe_item_at(item, location), location == self.anywhere)
        if self.searched_at in problem.fluents:
            for location in self.locations:
                problem.set_initial_value(self.searched_at(location), False)
        pose_locations = {
            self.base_table_1_pose: self.table_1,
            self.base_table_2_pose: self.table_2,
            self.base_table_3_pose: self.table_3,
            self.tool_search_pose: self.tool_search_location,
            self.box_search_pose: self.box_search_location,
        }
        if self.pose_at in problem.fluents:
            for pose in self.poses:
                problem.set_initial_value(
                    self.pose_at(pose, pose_locations[pose] if pose in pose_locations.keys() else self.anywhere), True
                )

    def label(self, action: ActionInstance) -> str:
        parameters = [parameter.object() for parameter in action.actual_parameters]
        parameter_labels = [self.parameter_labels.get(parameter, str(parameter)) for parameter in parameters]
        return self.method_labels[action.action](parameter_labels)
