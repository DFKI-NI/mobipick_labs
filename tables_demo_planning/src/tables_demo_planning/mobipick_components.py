from abc import ABC, abstractmethod
from enum import Enum, IntEnum
from geometry_msgs.msg import Pose
import rospy
from robot_api import TuplePose
import robot_api

"""General component definitions for Mobipick applications"""


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


class Robot(ABC):
    """Robot representation which maintains the current state."""

    def __init__(self) -> None:
        self.pose = self.get_pose()
        self.home_pose = self.pose
        self.arm_pose = self.get_arm_pose()
        self.item = self.get_item()

    @abstractmethod
    def get_pose(self) -> Pose:
        """Return current base pose."""

    @abstractmethod
    def get_arm_pose(self) -> ArmPose:
        """Return current arm pose."""

    @abstractmethod
    def get_item(self) -> Item:
        """Return Item.something if robot arm has an object attached, else Item.nothing."""

    def set_home_pose(self, pose: Pose) -> None:
        """Set home pose of robot base."""
        self.home_pose = pose

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
        super().__init__()

    def get_pose(self) -> Pose:
        return TuplePose.to_pose(self.base.get_pose())

    def get_arm_pose(self) -> ArmPose:
        arm_pose_name = self.arm.get_pose_name()
        return ArmPose(arm_pose_name) if arm_pose_name in [member.value for member in ArmPose] else ArmPose.unknown

    def get_item(self) -> Item:
        self.arm.execute("HasAttachedObjects")
        return Item.something if self.arm.get_result().result else Item.nothing

    def move_base(self, _: Pose, pose: Pose) -> bool:
        if self.base.move(pose) != 3:
            rospy.logerr(f"Move base to {pose} FAILED!")
            # Move to home pose whenever movement fails. Note: This is a drastic workaround.
            self.base.move(self.home_pose)
            return False

        return super().move_base(_, pose)

    def move_arm(self, _: ArmPose, arm_pose: ArmPose) -> bool:
        if not self.arm.move(arm_pose.name):
            rospy.logerr(f"Move arm to '{arm_pose.name} FAILED!'")
            return False

        return super().move_arm(_, arm_pose)


class EnvironmentRepresentation:
    def __init__(self, robot: Robot) -> None:
        # Note: Instantiate a subclass of Robot in a subclass of this class
        #  to enable mutual references during their initializations.
        self.robot = robot

    def __repr__(self) -> str:
        return (
            f"{self.robot.__class__.__name__} at {self.robot.pose},"
            f" arm at {self.robot.arm_pose}"
            f" with {self.robot.item}"
        )

    def get_robot_arm_at(self, arm_pose: ArmPose) -> bool:
        """Return fluent value whether robot arm is at arm_pose."""
        return self.robot.arm_pose == arm_pose

    def get_robot_has(self, item: Item) -> bool:
        """Return fluent value whether robot has item."""
        return self.robot.item == item
