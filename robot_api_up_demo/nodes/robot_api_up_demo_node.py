#!/usr/bin/env python3
from typing import Any, Dict, List, Optional, Tuple, Type
from enum import IntEnum
import math
import time
import yaml
import rospkg
from geometry_msgs.msg import Pose
from robot_api_up_demo.plan_visualization import PlanVisualization
from robot_api_up_demo.up_planning import Action, Planning
from robot_api import TuplePose
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
        self.robot.arm.execute("CaptureObject")
        self.robot.arm_pose = ArmPose.interaction
        self.robot.arm.execute("PickUpObject")
        if not self.robot.arm.get_result().result:
            return False

        self.robot.gripper_object = GripperObject.power_drill
        return True


class PlaceAction(RobotAction):
    def __call__(self) -> bool:
        self.robot.arm.execute("PlaceObject")
        self.robot.arm_pose = ArmPose.interaction
        self.robot.gripper_object = GripperObject.nothing
        return True


class HandoverAction(RobotAction):
    def __call__(self) -> bool:
        self.robot.arm.execute("MoveArmToHandover")
        self.robot.arm_pose = ArmPose.interaction
        self.robot.offered_object = True
        if not self.robot.arm.observe_force_torque(5.0, 25.0):
            return False

        self.robot.arm.execute("ReleaseGripper")
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
        # Visualize plan.
        self.visualization: Optional[PlanVisualization] = None

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
        (base_handover_pose, base_home_pose, base_pick_pose, base_place_pose, _) = self.planning.create_objects(
            self.poses
        )
        (_, arm_pose_home, arm_pose_transport, arm_pose_interaction) = self.planning.create_objects(
            {pose.name: pose for pose in ArmPose}
        )
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
            print(base_pose_name, self.robot.arm_pose.name, self.robot.gripper_object.name)
            problem.set_initial_value(self.robot_at(mobipick, self.planning.objects[base_pose_name]), True)
            problem.set_initial_value(
                self.robot_arm_at(mobipick, self.planning.objects[self.robot.arm_pose.name]), True
            )
            problem.set_initial_value(
                self.robot_has(mobipick, self.planning.objects[self.robot.gripper_object.name]), True
            )
            problem.set_initial_value(self.robot_offered(mobipick), self.robot.offered_object)
            problem.add_goal(self.robot_offered(mobipick))
            problem.add_goal(self.robot_has(mobipick, nothing))
            problem.add_goal(self.robot_at(mobipick, base_home_pose))

            # Plan
            actions = self.planning.plan_actions()
            if not actions:
                print("Execution ended because no plan could be found.")
                break

            print("> Plan:")
            action_names = [str(up_action) for up_action, _ in actions]
            print('\n'.join(action_names))
            if self.visualization:
                self.visualization.set_actions(action_names)
            else:
                self.visualization = PlanVisualization(action_names)
            # ... and execute.
            print("> Execution:")
            for index, (up_action, api_action) in enumerate(actions):
                print(up_action)
                self.visualization.execute(index)
                result = api_action()
                if result is None or result:
                    self.visualization.succeed(index)
                else:
                    print("-- Action failed! Need to replan.")
                    self.visualization.fail(index)
                    time.sleep(3.0)
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
