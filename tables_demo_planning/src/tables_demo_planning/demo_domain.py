from typing import Callable, Dict, Iterable, List, Optional, Sequence, TypeVar
from abc import ABC, abstractmethod
import itertools
import os
import time
import yaml
import rospy
import rospkg
from geometry_msgs.msg import Pose
from unified_planning.engines import OptimalityGuarantee
from unified_planning.model.action import InstantaneousAction
from unified_planning.model.fluent import Fluent
from unified_planning.model.object import Object
from unified_planning.model.problem import Problem
from unified_planning.plans.plan import ActionInstance
from unified_planning.shortcuts import Equals, Not, OneshotPlanner
from tables_demo_planning.planning_bridge import Bridge
from tables_demo_planning.mobipick_components import ArmPose, EnvironmentRepresentation, Item, Location, Robot
from robot_api import TuplePose
import robot_api

"""Concrete Mobipick domain, bridged from its application to its planning representations"""


R = TypeVar('R', bound=Robot)


class Domain(Bridge, ABC):
    BASE_HANDOVER_POSE_NAME = "base_handover_pose"
    BASE_HOME_POSE_NAME = "base_home_pose"
    BASE_PICK_POSE_NAME = "base_pick_pose"
    BASE_PLACE_POSE_NAME = "base_place_pose"
    BASE_TABLE_1_POSE_NAME = "base_table_1_pose"
    BASE_TABLE_2_POSE_NAME = "base_table_2_pose"
    BASE_TABLE_3_POSE_NAME = "base_table_3_pose"
    TOOL_SEARCH_POSE_NAME = "tool_search_pose"
    BOX_SEARCH_POSE_NAME = "box_search_pose"
    UNKNOWN_POSE_NAME = "unknown_pose"

    def __init__(self, env: EnvironmentRepresentation[R]) -> None:
        super().__init__()
        # Create types for planning based on class types.
        self.create_types([Robot, Pose, ArmPose, Item, Location])

        # Create fluents for planning.
        self.robot_at = self.create_fluent_from_signature("robot_at", [Pose])
        self.robot_arm_at = self.create_fluent(env.get_robot_arm_at)
        self.robot_has = self.create_fluent(env.get_robot_has)

        # Create objects for both planning and execution.
        self.api_robot = env.robot
        self.robot = self.create_object("mobipick", env.robot)
        config_path = f"{rospkg.RosPack().get_path('mobipick_pick_n_place')}/config/"
        filename = "moelk_tables_demo.yaml"
        self.api_poses = self.load_waypoints(os.path.join(config_path, filename))
        env.robot.set_home_pose(self.api_poses[self.BASE_HOME_POSE_NAME])
        self.poses = self.create_objects(self.api_poses)
        self.base_home_pose = self.objects[self.BASE_HOME_POSE_NAME]
        self.base_handover_pose = self.objects[self.BASE_HANDOVER_POSE_NAME]
        self.base_pick_pose = self.objects[self.BASE_PICK_POSE_NAME]
        self.base_place_pose = self.objects[self.BASE_PLACE_POSE_NAME]
        self.base_table_1_pose = self.objects[self.BASE_TABLE_1_POSE_NAME]
        self.base_table_2_pose = self.objects[self.BASE_TABLE_2_POSE_NAME]
        self.base_table_3_pose = self.objects[self.BASE_TABLE_3_POSE_NAME]
        self.tool_search_pose = self.create_object(self.TOOL_SEARCH_POSE_NAME, Pose())
        self.box_search_pose = self.create_object(self.BOX_SEARCH_POSE_NAME, Pose())
        self.unknown_pose = self.create_object(self.UNKNOWN_POSE_NAME, Pose())
        self.poses.extend([self.tool_search_pose, self.box_search_pose, self.unknown_pose])
        self.arm_poses = self.create_enum_objects(ArmPose)
        self.arm_pose_home = self.objects[ArmPose.home.name]
        self.arm_pose_observe = self.objects[ArmPose.observe.name]
        self.arm_pose_transport = self.objects[ArmPose.transport.name]
        self.arm_pose_interaction = self.objects[ArmPose.interaction.name]
        self.items = self.create_enum_objects(Item)
        self.nothing = self.objects[Item.nothing.name]
        self.power_drill = self.objects[Item.power_drill.name]
        self.box = self.objects[Item.box.name]
        self.multimeter = self.objects[Item.multimeter.name]
        self.relay = self.objects[Item.relay.name]
        self.screwdriver = self.objects[Item.screwdriver.name]
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

        # Create actions for planning based on class definitions.
        self.move_base, (_, x, y) = self.create_action(Robot.move_base)
        self.move_base.add_precondition(self.robot_at(x))
        self.move_base.add_precondition(self.robot_has(self.nothing))
        self.move_base.add_precondition(self.robot_arm_at(self.arm_pose_home))
        self.move_base.add_effect(self.robot_at(x), False)
        self.move_base.add_effect(self.robot_at(y), True)
        self.move_base_with_item, (_, item, x, y) = self.create_action(Robot.move_base_with_item)
        self.move_base_with_item.add_precondition(self.robot_at(x))
        self.move_base_with_item.add_precondition(self.robot_has(item))
        self.move_base_with_item.add_precondition(Not(Equals(item, self.nothing)))
        self.move_base_with_item.add_precondition(self.robot_arm_at(self.arm_pose_transport))
        self.move_base_with_item.add_effect(self.robot_at(x), False)
        self.move_base_with_item.add_effect(self.robot_at(y), True)
        self.move_arm, (_, x, y) = self.create_action(Robot.move_arm)
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

    def define_mobipick_problem(
        self,
        fluents: Optional[Iterable[Fluent]] = None,
        actions: Optional[Iterable[InstantaneousAction]] = None,
        poses: Optional[Iterable[Object]] = None,
        items: Optional[Iterable[Object]] = None,
        locations: Optional[Iterable[Object]] = None,
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

    @abstractmethod
    def initialize_problem(self) -> Problem:
        """Initialize the current UP problem to solve."""
        pass

    @abstractmethod
    def set_goals(self, problem: Problem) -> None:
        """Set goals of UP problem."""
        pass

    def set_initial_values(self, problem: Problem) -> None:
        """Set all initial values using the functions corresponding to this problem's fluents."""
        type_objects: Dict[type, List[Object]] = {}
        # Collect objects in problem for all parameters of all fluents.
        for fluent in problem.fluents:
            for parameter in fluent.signature:
                # Avoid redundancy.
                if parameter.type not in type_objects.keys():
                    type_objects[parameter.type] = list(problem.objects(parameter.type))
        for fluent in problem.fluents:
            if fluent.name in self._fluent_functions.keys():
                # Loop through all parameter value combinations.
                for parameters in itertools.product(*[type_objects[parameter.type] for parameter in fluent.signature]):
                    # Use the fluent function to calculate the initial values.
                    api_parameters = [self._api_objects[parameter.name] for parameter in parameters]
                    value = self.get_object(self._fluent_functions[fluent.name](*api_parameters))
                    problem.set_initial_value(fluent(*parameters), value)
        # Explicitly handle fluents without fluent function.
        if self.robot_at in problem.fluents:
            base_pose_name = self.api_robot.base.get_pose_name()
            base_pose = self.objects[base_pose_name] if base_pose_name else self.unknown_pose
            for pose in self.poses:
                problem.set_initial_value(self.robot_at(pose), pose == base_pose)

    def label(self, action: ActionInstance) -> str:
        parameters = [parameter.object() for parameter in action.actual_parameters]
        parameter_labels = [self.parameter_labels.get(parameter, str(parameter)) for parameter in parameters]
        return self.method_labels[action.action](parameter_labels)
