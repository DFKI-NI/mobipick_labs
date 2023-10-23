from typing import Dict
from enum import Enum
from geometry_msgs.msg import Pose
from symbolic_fact_generation.robot_facts_generator import HasArmPostureGenerator
import rospy
import robot_api


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


# Generic item class
class Item:
    def __init__(self, name: str) -> None:
        self.name = name


class Location:
    anywhere = "anywhere"
    table_1 = "table_1"
    table_2 = "table_2"
    table_3 = "table_3"
    table_4 = "table_4"
    in_box = "in_box"
    on_robot = "on_robot"


# Simple environment representation
# TODO Replace with fact generator.
class Env:
    robot_items: Dict[str, Item] = {}


class Mobipick:
    home_pose: Pose
    arm_pose: ArmPose
    item: Item

    api = robot_api.Robot("mobipick", True, True)
    arm_pose_fact_generator = HasArmPostureGenerator(
        fact_name='robot_arm_pose',
        joint_states_topic='/mobipick/joint_states',
        arm_posture_param='',
        arm_tolerance=0.01,
        undefined_pose_name="unknown",
    )

    @classmethod
    def initialize(cls, home_pose: Pose, arm_pose: ArmPose, item: Item) -> None:
        """Initialize all attributes of this robot."""
        cls.home_pose = home_pose
        cls.arm_pose = arm_pose
        cls.item = item

    # Robot action to get item
    # TODO Call actual robot command within.
    @staticmethod
    def get(item: Item) -> None:
        Env.robot_items[item.name] = item
        print(f"Get {item.name}.")

    @staticmethod
    def get_robot_arm_at(arm_pose: ArmPose) -> bool:
        """Return whether this robot's arm is at arm_pose."""
        arm_pose_facts = Mobipick.arm_pose_fact_generator.generate_facts()
        arm_pose_name = ArmPose.unknown.value
        if arm_pose_facts:
            arm_pose_name = arm_pose_facts[0].values[0]
        return ArmPose.get_by_value(arm_pose_name) == arm_pose

    @staticmethod
    def get_robot_has(item: Item) -> bool:
        """Return whether this robot has item."""
        return item in Env.robot_items.values()

    @staticmethod
    def move_base(_: Pose, pose: Pose) -> bool:
        if Mobipick.api.base.move(pose) != 3:
            rospy.logerr(f"Move base to {pose} FAILED!")
            # Move to home pose whenever movement fails. Note: This is a drastic workaround.
            Mobipick.api.base.move(Mobipick.home_pose)
            return False

        return True

    @staticmethod
    def move_arm(_: ArmPose, arm_pose: ArmPose) -> bool:
        if not Mobipick.api.arm.move(arm_pose.name):
            rospy.logerr(f"Move arm to '{arm_pose.name} FAILED!'")
            return False

        return True
