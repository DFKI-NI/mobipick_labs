"""General component definitions for Mobipick applications"""

from typing import Dict, Generic, Tuple, TypeVar
from enum import Enum, IntEnum
from geometry_msgs.msg import Pose
from symbolic_fact_generation.robot_facts_generator import HasArmPostureGenerator
import rospy
from robot_api import TuplePose
import robot_api


class ActionResult(IntEnum):
    FAILURE = 0
    SUCCESS = 1


class ArmPose(Enum):
    unknown = "unknown"
    home = "home"
    observe = "observe100cm_right"
    transport = "transport"
    place = "place_1"
    tucked = "tucked"
    handover = "handover"

    @classmethod
    def get_by_value(cls, value: str) -> 'ArmPose':
        """Return ArmPose by its Enum value if it exists, else ArmPose.unknown."""
        for member in ArmPose:
            if value == member.value:
                return member
        return ArmPose.unknown


class Item:
    items: Dict[str, 'Item'] = {}
    NOTHING: 'Item'

    def __init__(self, name: str) -> None:
        assert name not in Item.items.keys(), f"Item with name '{name}' already exists!"
        self.name = name
        Item.items[name] = self

    @staticmethod
    def get(name: str) -> 'Item':
        return Item.items[name] if name in Item.items.keys() else Item(name)

    def __str__(self) -> str:
        return self.name


Item.NOTHING = Item("nothing")


class Location(Enum):
    anywhere = "anywhere"
    table_1 = "table_1"
    table_2 = "table_2"
    table_3 = "table_3"
    in_box = "in_box"
    on_robot = "on_robot"
    tool_search_location = "tool_search_location"
    box_search_location = "box_search_location"


class ItemClass(Enum):
    klt = "klt"
    multimeter = "multimeter"
    screwdriver = "screwdriver"
    power_drill = "powerdrill"
    yellow_part = "yellow_part"
    bright_green_part = "bright_green_part"
    dark_green_part = "dark_green_part"
    purple_part = "purple_part"
    magenta_part = "magenta_part"
    red_part = "red_part"
    blue_part = "blue_part"


class Robot:
    """Robot representation which maintains the current state."""

    def __init__(self) -> None:
        self.home_pose: Pose
        self.pose: Pose
        self.arm_pose: ArmPose
        self.item: Item

    def initialize(self, home_pose: Pose, pose: Pose, arm_pose: ArmPose, item: Item) -> None:
        """Initialize all attributes of this robot."""
        # Note: The sources for initialization are not guaranteed to exist during instantiation.
        self.home_pose = home_pose
        self.pose = pose
        self.arm_pose = arm_pose
        self.item = item

    def move_base(self, _: Pose, pose: Pose) -> bool:
        """Move base to pose."""
        self.pose = pose
        return True

    def move_base_with_item(self, item: Item, _: Pose, pose: Pose) -> bool:
        """Move base with item to pose."""
        # Note: Same action as move_base(), just with item in transport pose.
        return self.move_base(_, pose)

    def move_arm(self, _: ArmPose, arm_pose: ArmPose) -> bool:
        """Move arm to arm_pose."""
        self.arm_pose = arm_pose
        return True


class APIRobot(Robot, robot_api.Robot):
    """Robot which enables action execution via Robot API."""

    def __init__(self, namespace: str) -> None:
        robot_api.Robot.__init__(self, namespace, True, True)
        Robot.__init__(self)

    @staticmethod
    def add_waypoints(poses: Dict[str, Pose]) -> None:
        for pose_name, pose in poses.items():
            position, orientation = TuplePose.from_pose(pose)
            robot_api.add_waypoint(pose_name, (position, orientation))

    def get_pose(self) -> Pose:
        """Return current base pose."""
        return TuplePose.to_pose(self.base.get_pose())

    def get_arm_pose(self) -> ArmPose:
        """Return current arm pose."""
        return ArmPose.get_by_value(self.arm.get_pose_name())

    def get_item(self) -> Item:
        """Return Item 'something' if robot arm has an object attached, else Item.NOTHING."""
        self.arm.execute("HasAttachedObjects")
        return Item.get("something") if self.arm.get_result().result else Item.NOTHING

    def get(self) -> Tuple[Pose, ArmPose, Item]:
        """Return current state of robot."""
        return self.get_pose(), self.get_arm_pose(), self.get_item()

    def move_base(self, _: Pose, pose: Pose) -> bool:
        if self.base.move(pose) != 3:
            rospy.logerr(f"Move base to {pose} FAILED!")
            # Move to home pose whenever movement fails. Note: This is a drastic workaround.
            self.base.move(self.home_pose)
            return False

        return Robot.move_base(self, _, pose)

    def move_arm(self, _: ArmPose, arm_pose: ArmPose) -> bool:
        if not self.arm.move(arm_pose.name):
            rospy.logerr(f"Move arm to '{arm_pose.name} FAILED!'")
            return False

        return Robot.move_arm(self, _, arm_pose)


R = TypeVar('R', bound=Robot)


class EnvironmentRepresentation(Generic[R]):
    def __init__(self, robot: R) -> None:
        # Note: Instantiate a subclass of Robot in a subclass of this class
        #  to enable mutual references during their initializations.
        self.robot = robot

        self.arm_pose_fact_generator = HasArmPostureGenerator(
            fact_name='robot_arm_pose',
            joint_states_topic='/mobipick/joint_states',
            arm_posture_param='',
            arm_tolerance=0.01,
            undefined_pose_name="unknown",
        )

    def __repr__(self) -> str:
        return (
            f"{self.robot.__class__.__name__} at {self.robot.pose},"
            f" arm at {self.robot.arm_pose}"
            f" with {self.robot.item}"
        )

    def get_robot_arm_at(self, arm_pose: ArmPose) -> bool:
        """Return fluent value whether robot arm is at arm_pose."""
        arm_pose_facts = self.arm_pose_fact_generator.generate_facts()
        arm_pose_name = ArmPose.unknown.value
        if arm_pose_facts:
            arm_pose_name = arm_pose_facts[0].values[0]
        return ArmPose.get_by_value(arm_pose_name) == arm_pose

    def get_robot_has(self, item: Item) -> bool:
        """Return fluent value whether robot has item."""
        return self.robot.item == item
