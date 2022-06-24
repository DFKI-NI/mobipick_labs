from typing import Any, Dict, List, Optional, Sequence, Tuple, Type
from abc import ABC, abstractmethod
from enum import IntEnum
import os
import yaml
import rospkg
import rospy
from geometry_msgs.msg import Pose
from std_srvs.srv import SetBool
from unified_planning.model.action import InstantaneousAction
from unified_planning.model.fluent import Fluent
from unified_planning.model.object import Object
from unified_planning.model.problem import Problem
from unified_planning.plans.plan import ActionInstance
from unified_planning.shortcuts import OneshotPlanner
from symbolic_fact_generation import on_fact_generator
from tables_demo_planning.planning_bridge import Action, Bridge
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


class Item(IntEnum):
    nothing = 0
    power_drill = 1


class Robot(robot_api.Robot):
    """Robot representation which maintains the current state and enables action execution via Robot API."""

    def __init__(self, namespace: str) -> None:
        super().__init__(namespace, True, True)
        self.pose: Pose = TuplePose.to_pose(self.base.get_pose())
        self.arm_pose = self.get_arm_pose()
        self.item = self.get_item()
        self.item_offered = False

    def get_arm_pose(self) -> ArmPose:
        arm_pose_name = self.arm.get_pose_name()
        return ArmPose[arm_pose_name] if arm_pose_name in ArmPose.__members__ else ArmPose.unknown

    def get_item(self) -> Item:
        # Note: In this demo, robot can only have the power drill.
        self.arm.execute("HasAttachedObjects")
        return Item(int(self.arm.get_result().result))


# Define executable actions.


class RobotAction(Action):
    SIGNATURE: Tuple[Type, ...] = (Robot,)

    def __init__(self, *args: Any) -> None:
        super().__init__(*args)
        self.robot: Robot = args[0]


class MoveBaseAction(RobotAction):
    SIGNATURE = (Robot, Pose, Pose)

    def __init__(self, *args: Any) -> None:
        super().__init__(*args)
        self.pose: Pose = args[2]

    def __call__(self) -> bool:
        self.robot.base.move(self.pose)
        self.robot.pose = self.pose
        return True


class TransportAction(MoveBaseAction):
    pass


class MoveArmAction(RobotAction):
    SIGNATURE = (Robot, ArmPose, ArmPose)

    def __init__(self, *args: Any) -> None:
        super().__init__(*args)
        self.arm_pose: ArmPose = args[2]

    def __call__(self) -> bool:
        self.robot.arm.move(self.arm_pose.name)
        self.robot.arm_pose = self.arm_pose
        return True


class PickAction(RobotAction):
    def __call__(self) -> bool:
        PerceiveAction(self.robot)()
        self.robot.arm.execute("CaptureObject")
        self.robot.arm_pose = self.robot.get_arm_pose()
        self.robot.arm.execute("PickUpObject")
        self.robot.arm_pose = self.robot.get_arm_pose()
        if not self.robot.arm.get_result().result:
            return False

        self.robot.item = Item.power_drill
        return True


class PlaceAction(RobotAction):
    def __call__(self) -> bool:
        self.robot.arm.execute("PlaceObject")
        self.robot.arm_pose = ArmPose.place_1
        self.robot.item = Item.nothing
        return True


class HandoverAction(RobotAction):
    def __call__(self) -> bool:
        self.robot.arm.execute("MoveArmToHandover")
        self.robot.arm_pose = self.robot.get_arm_pose()
        self.robot.item_offered = True
        if not self.robot.arm.observe_force_torque(5.0, 25.0):
            return False

        self.robot.arm.execute("ReleaseGripper")
        return True


class PerceiveAction(RobotAction):
    activate_pose_selector = rospy.ServiceProxy('/pose_selector_activate', SetBool)

    def __call__(self) -> bool:
        self.robot.arm.move("observe100cm_right")
        rospy.wait_for_service('/pose_selector_activate')
        activation_result = self.activate_pose_selector(True)
        rospy.sleep(5)
        facts = on_fact_generator.get_current_facts()
        print(facts)
        deactivation_result = self.activate_pose_selector(False)
        return activation_result and deactivation_result


# Define domain for both planning and execution.


class Domain(Bridge, ABC):
    BASE_HANDOVER_POSE_NAME = "base_handover_pose"
    BASE_HOME_POSE_NAME = "base_home_pose"
    BASE_PICK_POSE_NAME = "base_pick_pose"
    BASE_PLACE_POSE_NAME = "base_place_pose"
    BASE_START_POSE_NAME = "base_start_pose"
    BASE_TABLE_1_POSE = "base_table_1_pose"
    BASE_TABLE_2_POSE = "base_table_2_pose"
    BASE_TABLE_3_POSE = "base_table_3_pose"

    def __init__(self) -> None:
        super().__init__()
        # Create types for planning based on class types.
        self.create_types([Robot, Pose, ArmPose, Item])

        # Create fluents for planning.
        self.robot_at = self.create_fluent("At", [Robot, Pose])
        self.robot_arm_at = self.create_fluent("ArmAt", [Robot, ArmPose])
        self.robot_has = self.create_fluent("Has", [Robot, Item])
        self.robot_offered = self.create_fluent("Offered", [Robot])

        # Create objects for both planning and execution.
        self.api_robot = Robot("mobipick")
        self.robot = self.create_object("mobipick", self.api_robot)
        config_path = f"{rospkg.RosPack().get_path('mobipick_pick_n_place')}/config/"
        filename = "moelk_tables_demo.yaml"
        self.api_poses = self.load_waypoints(os.path.join(config_path, filename))
        self.api_poses[self.BASE_START_POSE_NAME] = self.api_robot.pose
        robot_api.add_waypoint(self.BASE_START_POSE_NAME, TuplePose.from_pose(self.api_robot.pose))
        self.poses = self.create_objects(self.api_poses)
        self.base_home_pose = self.objects[self.BASE_HOME_POSE_NAME]
        self.base_handover_pose = self.objects[self.BASE_HANDOVER_POSE_NAME]
        self.base_pick_pose = self.objects[self.BASE_PICK_POSE_NAME]
        self.base_place_pose = self.objects[self.BASE_PLACE_POSE_NAME]
        self.arm_poses = self.create_objects({pose.name: pose for pose in ArmPose})
        arm_pose_home = self.objects[ArmPose.home.name]
        arm_pose_transport = self.objects[ArmPose.transport.name]
        arm_pose_interaction = self.objects[ArmPose.interaction.name]
        self.items = self.create_objects({item.name: item for item in Item})
        (self.nothing, self.power_drill) = self.items

        # Create actions for planning based on class definitions.
        self.move_base, (robot, x, y) = self.create_action(MoveBaseAction)
        self.move_base.add_precondition(self.robot_at(robot, x))
        self.move_base.add_precondition(self.robot_has(robot, self.nothing))
        self.move_base.add_precondition(self.robot_arm_at(robot, arm_pose_home))
        self.move_base.add_effect(self.robot_at(robot, x), False)
        self.move_base.add_effect(self.robot_at(robot, y), True)
        self.transport, (robot, x, y) = self.create_action(TransportAction)
        self.transport.add_precondition(self.robot_at(robot, x))
        self.transport.add_precondition(self.robot_has(robot, self.power_drill))
        self.transport.add_precondition(self.robot_arm_at(robot, arm_pose_transport))
        self.transport.add_effect(self.robot_at(robot, x), False)
        self.transport.add_effect(self.robot_at(robot, y), True)
        self.move_arm, (robot, x, y) = self.create_action(MoveArmAction)
        self.move_arm.add_precondition(self.robot_arm_at(robot, x))
        self.move_arm.add_effect(self.robot_arm_at(robot, x), False)
        self.move_arm.add_effect(self.robot_arm_at(robot, y), True)
        self.pick, (robot,) = self.create_action(PickAction)
        self.pick.add_precondition(self.robot_has(robot, self.nothing))
        self.pick.add_precondition(self.robot_at(robot, self.base_pick_pose))
        self.pick.add_precondition(self.robot_arm_at(robot, arm_pose_home))
        self.pick.add_effect(self.robot_has(robot, self.nothing), False)
        self.pick.add_effect(self.robot_has(robot, self.power_drill), True)
        self.pick.add_effect(self.robot_arm_at(robot, arm_pose_home), False)
        self.pick.add_effect(self.robot_arm_at(robot, arm_pose_interaction), True)
        self.place, (robot,) = self.create_action(PlaceAction)
        self.place.add_precondition(self.robot_has(robot, self.power_drill))
        self.place.add_precondition(self.robot_at(robot, self.base_place_pose))
        self.place.add_precondition(self.robot_arm_at(robot, arm_pose_transport))
        self.place.add_effect(self.robot_has(robot, self.power_drill), False)
        self.place.add_effect(self.robot_has(robot, self.nothing), True)
        self.place.add_effect(self.robot_arm_at(robot, arm_pose_transport), False)
        self.place.add_effect(self.robot_arm_at(robot, arm_pose_interaction), True)
        self.hand_over, (robot,) = self.create_action(HandoverAction)
        self.hand_over.add_precondition(self.robot_has(robot, self.power_drill))
        self.hand_over.add_precondition(self.robot_at(robot, self.base_handover_pose))
        self.hand_over.add_precondition(self.robot_arm_at(robot, arm_pose_transport))
        self.hand_over.add_effect(self.robot_arm_at(robot, arm_pose_transport), False)
        self.hand_over.add_effect(self.robot_arm_at(robot, arm_pose_interaction), True)
        self.hand_over.add_effect(self.robot_has(robot, self.power_drill), False)
        self.hand_over.add_effect(self.robot_has(robot, self.nothing), True)
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
        actions: Optional[Sequence[InstantaneousAction]] = None,
    ) -> Problem:
        """Define a UP problem by its (potential subsets of) fluents, objects, and actions."""
        problem = Problem()
        for fluent in fluents if fluents else self.fluents.values():
            problem.add_fluent(fluent, default_initial_value=False)
        problem.add_object(self.robot)
        problem.add_objects(poses if poses else self.poses)
        problem.add_objects(self.arm_poses)
        problem.add_objects(items if items else self.items)
        for action in actions if actions else self.actions.values():
            problem.add_action(action)
        return problem

    def solve(self, problem: Problem) -> Optional[Sequence[Tuple[ActionInstance, Action]]]:
        """Solve planning problem, then return list of UP and Robot API actions."""
        result = OneshotPlanner(problem_kind=problem.kind).solve(problem)
        return [(action, self.get_action(action)) for action in result.plan.actions] if result.plan else None

    @abstractmethod
    def initialize_problem(self) -> Problem:
        """Initialize the UP problem based on current state."""

    @abstractmethod
    def set_goals(self, problem: Problem) -> None:
        """Define the current goals of the UP problem."""
