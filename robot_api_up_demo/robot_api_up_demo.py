#!/usr/bin/env python3
from typing import Dict, List
from enum import IntEnum
import math
import yaml
import rospkg
from geometry_msgs.msg import Pose
from up_planning import Planning
from robot_api import Action, TuplePose
import robot_api


# Define state representation classes and specifiers.

class ArmPose(IntEnum):
    unknown = 0
    home = 1
    transport = 2
    interaction = 3


class GripperObject(IntEnum):
    nothing = 0
    power_drill = 1


class Robot(robot_api.Robot):
    """Robot representation which maintains the current state and enables action execution via Robot API."""
    def __init__(self, namespace: str) -> None:
        super().__init__(namespace, True, True)
        self.pose = TuplePose.to_pose(self.base.get_pose())
        arm_pose_name = self.arm.get_pose_name()
        self.arm_pose = ArmPose[arm_pose_name] if arm_pose_name in ArmPose.__members__ else ArmPose.unknown
        self.arm.execute("HasAttachedObjects")
        self.gripper_object = GripperObject(int(self.arm.get_result().result))
        self.offered_object = False


# Define executable actions.

class MoveBaseAction(Action):
    @staticmethod
    def execute(robot: Robot, _: Pose, pose: Pose) -> None:
        robot.base.move(pose=pose)
        robot.pose = pose


class TransportAction(Action):
    @staticmethod
    def execute(robot: Robot, _: Pose, pose: Pose) -> None:
        robot.base.move(pose=pose)
        robot.pose = pose


class MoveArmAction(Action):
    @staticmethod
    def execute(robot: Robot, _: ArmPose, pose: ArmPose) -> None:
        robot.arm.move(pose.name)
        robot.arm_pose = pose


class PickAction(Action):
    @staticmethod
    def execute(robot: Robot) -> bool:
        robot.arm.execute("CaptureObject")
        robot.arm_pose = ArmPose.interaction
        robot.arm.execute("PickUpObject")
        if not robot.arm.get_result().result:
            return False

        robot.gripper_object = GripperObject.power_drill
        return True


class PlaceAction(Action):
    @staticmethod
    def execute(robot: Robot) -> None:
        robot.arm.execute("PlaceObject")
        robot.arm_pose = ArmPose.interaction
        robot.gripper_object = GripperObject.nothing


class HandoverAction(Action):
    @staticmethod
    def execute(robot: Robot) -> bool:
        robot.arm.execute("MoveArmToHandover")
        robot.arm_pose = ArmPose.interaction
        robot.offered_object = True
        if not robot_api.ArmForceTorqueObserverAction.execute(robot.arm, 5.0, 25.0):
            return False

        robot.arm.execute("ReleaseGripper")
        return True


# Main demo class which orchestrates planning and execution.

class Demo:
    BASE_START_POSE_NAME = "base_start_pose"

    def __init__(self) -> None:
        self.robot = Robot("mobipick")
        self.poses: Dict[str, Pose] = {}
        self.load_waypoints()
        # Define types and fluents for planning.
        self.planning = Planning()
        self.planning.create_types([Robot, Pose, ArmPose, GripperObject])
        self.robot_at = self.planning.create_fluent("At", [Robot, Pose])
        self.robot_arm_at = self.planning.create_fluent("ArmAt", [Robot, ArmPose])
        self.robot_has = self.planning.create_fluent("Has", [Robot, GripperObject])
        self.robot_offered = self.planning.create_fluent("Offered", [Robot])

    def load_waypoints(self) -> None:
        # Load poses from mobipick config file.
        filepath = f"{rospkg.RosPack().get_path('mobipick_pick_n_place')}/config/moelk.yaml"
        with open(filepath, 'r') as yaml_file:
            yaml_contents: Dict[str, List[float]] = yaml.safe_load(yaml_file)["poses"]
            for pose_name in sorted(yaml_contents.keys()):
                if pose_name.startswith("base_") and pose_name.endswith("_pose"):
                    pose = yaml_contents[pose_name]
                    position, orientation = pose[:3], pose[4:] + [pose[3]]
                    self.poses[pose_name] = TuplePose.to_pose((position, orientation))
                    robot_api.add_waypoint(pose_name, (position, orientation))
        # Add current robot pose as named pose and waypoint.
        self.poses[self.BASE_START_POSE_NAME] = self.robot.pose
        robot_api.add_waypoint(self.BASE_START_POSE_NAME, TuplePose.from_pose(self.robot.pose))

    def run(self) -> None:
        # Define objects for planning.
        mobipick = self.planning.create_object("mobipick", self.robot)
        (base_handover_pose, base_home_pose, base_pick_pose, base_place_pose, _) \
            = self.planning.create_objects(self.poses)
        (_, arm_pose_home, arm_pose_transport, arm_pose_interaction) \
            = self.planning.create_objects({pose.name: pose for pose in ArmPose})
        (nothing, power_drill) = self.planning.create_objects({obj.name: obj for obj in GripperObject})

        # Define actions for planning.
        move_base, (robot, x, y) = self.planning.create_action(MoveBaseAction)
        move_base.add_precondition(self.robot_at(robot, x))
        move_base.add_precondition(self.robot_has(robot, nothing))
        move_base.add_precondition(self.robot_arm_at(robot, arm_pose_home))
        move_base.add_effect(self.robot_at(robot, x), False)
        move_base.add_effect(self.robot_at(robot, y), True)

        move_base, (robot, x, y) = self.planning.create_action(TransportAction)
        move_base.add_precondition(self.robot_at(robot, x))
        move_base.add_precondition(self.robot_has(robot, power_drill))
        move_base.add_precondition(self.robot_arm_at(robot, arm_pose_transport))
        move_base.add_effect(self.robot_at(robot, x), False)
        move_base.add_effect(self.robot_at(robot, y), True)

        move_arm, (robot, x, y) = self.planning.create_action(MoveArmAction)
        move_arm.add_precondition(self.robot_arm_at(robot, x))
        move_arm.add_effect(self.robot_arm_at(robot, x), False)
        move_arm.add_effect(self.robot_arm_at(robot, y), True)

        pick, (robot,) = self.planning.create_action(PickAction)
        pick.add_precondition(self.robot_has(robot, nothing))
        pick.add_precondition(self.robot_at(robot, base_pick_pose))
        pick.add_precondition(self.robot_arm_at(robot, arm_pose_home))
        pick.add_effect(self.robot_has(robot, nothing), False)
        pick.add_effect(self.robot_has(robot, power_drill), True)
        pick.add_effect(self.robot_arm_at(robot, arm_pose_home), False)
        pick.add_effect(self.robot_arm_at(robot, arm_pose_interaction), True)

        place, (robot,) = self.planning.create_action(PlaceAction)
        place.add_precondition(self.robot_has(robot, power_drill))
        place.add_precondition(self.robot_at(robot, base_place_pose))
        place.add_precondition(self.robot_arm_at(robot, arm_pose_transport))
        place.add_effect(self.robot_has(robot, power_drill), False)
        place.add_effect(self.robot_has(robot, nothing), True)
        place.add_effect(self.robot_arm_at(robot, arm_pose_transport), False)
        place.add_effect(self.robot_arm_at(robot, arm_pose_interaction), True)

        hand_over, (robot,) = self.planning.create_action(HandoverAction)
        hand_over.add_precondition(self.robot_has(robot, power_drill))
        hand_over.add_precondition(self.robot_at(robot, base_handover_pose))
        hand_over.add_precondition(self.robot_arm_at(robot, arm_pose_transport))
        hand_over.add_effect(self.robot_arm_at(robot, arm_pose_transport), False)
        hand_over.add_effect(self.robot_arm_at(robot, arm_pose_interaction), True)
        hand_over.add_effect(self.robot_has(robot, power_drill), False)
        hand_over.add_effect(self.robot_has(robot, nothing), True)
        hand_over.add_effect(self.robot_offered(robot), True)

        # Now handle the pick and place task.
        complete = False
        while not complete:
            # Define problem based on current state.
            problem = self.planning.init_problem()
            base_pose_name = self.robot.base.get_pose_name(xy_tolerance=math.inf, yaw_tolerance=math.pi)
            problem.set_initial_value(self.robot_at(mobipick, self.planning.objects[base_pose_name]), True)
            problem.set_initial_value(self.robot_arm_at(mobipick,
                self.planning.objects[self.robot.arm_pose.name]), True)
            problem.set_initial_value(self.robot_has(mobipick,
                self.planning.objects[self.robot.gripper_object.name]), True)
            problem.set_initial_value(self.robot_offered(mobipick), self.robot.offered_object)
            problem.add_goal(self.robot_offered(mobipick))
            problem.add_goal(self.robot_has(mobipick, nothing))
            problem.add_goal(self.robot_at(mobipick, base_home_pose))

            # Plan
            actions = self.planning.plan()
            if not actions:
                print("Execution ended because no plan could be found.")
                break

            print("> Plan:")
            print('\n'.join(str(up_action) for up_action, _ in actions))
            # ... and execute.
            print("> Execution:")
            for up_action, api_action in actions:
                print(up_action)
                result = api_action()
                if result is not None and not result:
                    print("-- Action failed! Need to replan.")
                    # In this simple demo, failed actions are no longer available for planning and execution.
                    for action_type in list(self.planning.actions.keys()):
                        if isinstance(api_action, action_type):
                            del self.planning.actions[action_type]
                    # Abort execution and loop to planning.
                    break
            else:
                complete = True
                print("Task complete.")


if __name__ == '__main__':
    Demo().run()
