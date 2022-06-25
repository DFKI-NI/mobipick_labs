from typing import Callable, Dict, List, Optional, Sequence, Tuple, Type
from abc import ABC, abstractmethod
from enum import Enum, IntEnum
import math
import os
import yaml
import rospkg
from geometry_msgs.msg import Pose
from unified_planning.model.action import InstantaneousAction
from unified_planning.model.fluent import Fluent
from unified_planning.model.object import Object
from unified_planning.model.problem import Problem
from unified_planning.plans.plan import ActionInstance
from unified_planning.shortcuts import Equals, OneshotPlanner
from tables_demo_planning.planning_bridge import Bridge
from robot_api import TuplePose
import robot_api

"""
Define types, objects, and actions for the mobipick_tables_demo domain,
both for planning and for execution.
"""


# Define state representation classes.


class ArmPose(IntEnum):
    unknown = 0
    home = 1
    observe100cm_right = 2
    transport = 3
    place_1 = 4
    tucked = 5
    interaction = 6  # not a pose in rosparams but used to simplify planning


class Item(Enum):
    nothing = "nothing_1"
    power_drill = "powerdrill_with_grip_1"
    klt = "klt_1"
    multimeter = "multimeter_1"
    relay = "relay_1"
    screwdriver = "screwdriver_1"
    something = "something"


class Location(Enum):
    anywhere = "anywhere"
    table_1 = "table_1"
    table_2 = "table_2"
    table_3 = "table_3"
    in_klt = "klt_1"
    on_robot = "on_robot"
    item_search = "item_search"
    klt_search = "klt_search"


class Robot(robot_api.Robot):
    """Robot representation which maintains the current state and enables action execution via Robot API."""

    def __init__(self, namespace: str) -> None:
        super().__init__(namespace, True, True)
        self.pose: Pose = TuplePose.to_pose(self.base.get_pose())
        self.arm_pose = self.get_arm_pose()
        self.item = self.get_initial_item()
        self.item_offered = False

    def get_arm_pose(self) -> ArmPose:
        arm_pose_name = self.arm.get_pose_name()
        return ArmPose[arm_pose_name] if arm_pose_name in ArmPose.__members__ else ArmPose.unknown

    def get_initial_item(self) -> Item:
        self.arm.execute("HasAttachedObjects")
        return Item.something if self.arm.get_result().result else Item.nothing

    def move_base(self, _: Pose, pose: Pose) -> bool:
        self.base.move(pose)
        self.pose = pose
        return True

    def move_base_with_item(self, _: Pose, pose: Pose) -> bool:
        self.base.move(pose)
        self.pose = pose
        return True

    def move_arm(self, _: ArmPose, arm_pose: ArmPose) -> bool:
        self.arm.move(arm_pose.name)
        self.arm_pose = arm_pose
        return True

    def hand_over(self) -> bool:
        self.arm.execute("MoveArmToHandover")
        self.arm_pose = self.get_arm_pose()
        self.item_offered = True
        if not self.arm.observe_force_torque(5.0, 25.0):
            return False

        self.arm.execute("ReleaseGripper")
        return True


# Define domain for both planning and execution.


class Domain(Bridge, ABC):
    BASE_HANDOVER_POSE_NAME = "base_handover_pose"
    BASE_HOME_POSE_NAME = "base_home_pose"
    BASE_PICK_POSE_NAME = "base_pick_pose"
    BASE_PLACE_POSE_NAME = "base_place_pose"
    BASE_TABLE_1_POSE = "base_table_1_pose"
    BASE_TABLE_2_POSE = "base_table_2_pose"
    BASE_TABLE_3_POSE = "base_table_3_pose"

    def __init__(self, robot_class: Type[Robot] = Robot) -> None:
        super().__init__()
        # Create types for planning based on class types.
        self.create_types([Robot, Pose, ArmPose, Item, Location])

        # Create fluents for planning.
        self.robot_at = self.create_fluent("At", [Robot], Pose)
        self.robot_arm_at = self.create_fluent("ArmAt", [Robot], ArmPose)
        self.robot_has = self.create_fluent("Has", [Robot], Item)
        self.robot_offered = self.create_fluent("Offered", [Robot])
        self.believe_item_at = self.create_fluent("BelieveItemAt", [Item], Location)
        self.searched_at = self.create_fluent("SearchedAt", [Location])

        # Create objects for both planning and execution.
        self.api_robot = robot_class("mobipick")
        self.robot = self.create_object("mobipick", self.api_robot)
        config_path = f"{rospkg.RosPack().get_path('mobipick_pick_n_place')}/config/"
        filename = "moelk_tables_demo.yaml"
        self.api_poses = self.load_waypoints(os.path.join(config_path, filename))
        self.poses = self.create_objects(self.api_poses)
        self.base_home_pose = self.objects[self.BASE_HOME_POSE_NAME]
        self.base_handover_pose = self.objects[self.BASE_HANDOVER_POSE_NAME]
        self.base_pick_pose = self.objects[self.BASE_PICK_POSE_NAME]
        self.base_place_pose = self.objects[self.BASE_PLACE_POSE_NAME]
        self.arm_poses = self.create_objects({pose.name: pose for pose in ArmPose})
        self.arm_pose_home = self.objects[ArmPose.home.name]
        self.arm_pose_transport = self.objects[ArmPose.transport.name]
        self.arm_pose_interaction = self.objects[ArmPose.interaction.name]
        self.items = self.create_objects({item.name: item for item in Item})
        self.nothing = self.objects[Item.nothing.name]
        self.power_drill = self.objects[Item.power_drill.name]
        self.locations = self.create_objects({location.name: location for location in Location})
        self.in_klt = self.objects[Location.in_klt.name]

        # Create actions for planning based on class definitions.
        self.move_base, (robot, x, y) = self.create_action(Robot, Robot.move_base)
        self.move_base.add_precondition(Equals(self.robot_at(robot), x))
        self.move_base.add_precondition(Equals(self.robot_has(robot), self.nothing))
        self.move_base.add_precondition(Equals(self.robot_arm_at(robot), self.arm_pose_home))
        self.move_base.add_effect(self.robot_at(robot), y)
        self.move_base_with_item, (robot, x, y) = self.create_action(Robot, Robot.move_base_with_item)
        self.move_base_with_item.add_precondition(Equals(self.robot_at(robot), x))
        self.move_base_with_item.add_precondition(Equals(self.robot_has(robot), self.power_drill))
        self.move_base_with_item.add_precondition(Equals(self.robot_arm_at(robot), self.arm_pose_transport))
        self.move_base_with_item.add_effect(self.robot_at(robot), y)
        self.move_arm, (robot, x, y) = self.create_action(Robot, Robot.move_arm)
        self.move_arm.add_precondition(Equals(self.robot_arm_at(robot), x))
        self.move_arm.add_effect(self.robot_arm_at(robot), y)
        self.hand_over, (robot,) = self.create_action(Robot, Robot.hand_over)
        self.hand_over.add_precondition(Equals(self.robot_has(robot), self.power_drill))
        self.hand_over.add_precondition(Equals(self.robot_at(robot), self.base_handover_pose))
        self.hand_over.add_precondition(Equals(self.robot_arm_at(robot), self.arm_pose_transport))
        self.hand_over.add_effect(self.robot_arm_at(robot), self.arm_pose_interaction)
        self.hand_over.add_effect(self.robot_has(robot), self.nothing)
        self.hand_over.add_effect(self.robot_offered(robot), True)

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
        for fluent in self.fluents.values() if fluents is None else fluents:
            problem.add_fluent(fluent)
        problem.add_object(self.robot)
        problem.add_objects(self.poses if poses is None else poses)
        problem.add_objects(self.arm_poses)
        problem.add_objects(self.items if items is None else items)
        problem.add_objects(self.locations if locations is None else locations)
        for action in self.actions.values() if actions is None else actions:
            problem.add_action(action)
        return problem

    def solve(
        self, problem: Problem
    ) -> Optional[Sequence[Tuple[ActionInstance, Tuple[Callable[..., bool], List[object]]]]]:
        """Solve planning problem, then return list of UP and Robot API actions."""
        result = OneshotPlanner(problem_kind=problem.kind).solve(problem)
        return [(action, self.get_action(action)) for action in result.plan.actions] if result.plan else None

    def set_values(self, problem: Problem) -> None:
        base_pose_name = self.api_robot.base.get_pose_name(xy_tolerance=math.inf, yaw_tolerance=math.pi)
        fluents = problem.fluents
        if self.robot_at in fluents:
            problem.set_initial_value(self.robot_at(self.robot), self.objects[base_pose_name])
        if self.robot_arm_at in fluents:
            problem.set_initial_value(self.robot_arm_at(self.robot), self.objects[self.api_robot.arm_pose.name])
        if self.robot_has in fluents:
            problem.set_initial_value(self.robot_has(self.robot), self.objects[self.api_robot.item.name])
        if self.robot_offered in fluents:
            problem.set_initial_value(self.robot_offered(self.robot), self.api_robot.item_offered)

    @abstractmethod
    def initialize_problem(self) -> Problem:
        """Initialize the UP problem based on current state."""

    @abstractmethod
    def set_goals(self, problem: Problem) -> None:
        """Define the current goals of the UP problem."""
