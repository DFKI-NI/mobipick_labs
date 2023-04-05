#!/usr/bin/env python3
from up_esb.bridge import Bridge
from up_esb.plexmo import PlanDispatcher


# Application domain definitions
class Pose:
    def __init__(self, position: list):
        self.pose = position

    def __repr__(self):
        return f"Pose({self.pose})"


class Robot:
    def __init__(self, pose: Pose) -> None:
        self.pose = pose

    def move(self, from_pose: Pose, to_pose: Pose) -> bool:
        """Move from from_pose to to_pose."""
        if self.pose != from_pose:
            return False
        self.pose = to_pose
        return True


def robot_at(pose: Pose) -> bool:
    return robot.pose == pose


pose1, pose2 = Pose(position=[1, 0, 0]), Pose(position=[2, 0, 0])
robot = Robot(pose1)

# Pass application representations to bridge interface to get UP representations.
bridge = Bridge()
bridge.create_types([Robot, Pose])
up_robot = bridge.create_object("robot", robot)
up_pose1, up_pose2 = bridge.create_objects({"pose1": pose1, "pose2": pose2})
up_robot_at = bridge.create_fluent_from_function(robot_at)
up_move, (robot_param, from_pose, to_pose) = bridge.create_action_from_function(Robot.move)
up_move.add_precondition(up_robot_at(from_pose))
up_move.add_effect(up_robot_at(from_pose), False)
up_move.add_effect(up_robot_at(to_pose), True)

# Define and solve planning problem.
problem = bridge.define_problem()
bridge.set_initial_values(problem)
problem.add_goal(up_robot_at(up_pose2))
plan = bridge.solve(problem)

# Initial Pose
print(robot.pose)

# Execute actions.
graph = bridge.get_executable_graph(plan)
dispatcher = PlanDispatcher()
dispatcher.execute_plan(graph)

# Final Pose
print(robot.pose)
