from typing import Optional
import math
from unified_planning.model.problem import Problem
from tables_demo_planning.demo_domain import Domain
from tables_demo_planning.plan_visualization import PlanVisualization


class Executor:
    def __init__(self, domain: Domain) -> None:
        self.domain = domain
        self.visualization: Optional[PlanVisualization] = None

    def set_values(self, problem: Problem) -> None:
        base_pose_name = self.domain.api_robot.base.get_pose_name(xy_tolerance=math.inf, yaw_tolerance=math.pi)
        fluents = problem.fluents
        if self.domain.robot_at in fluents:
            for pose in self.domain.poses:
                problem.set_initial_value(self.domain.robot_at(self.domain.robot, pose), pose.name == base_pose_name)
        if self.domain.robot_arm_at in fluents:
            for arm_pose in self.domain.arm_poses:
                problem.set_initial_value(
                    self.domain.robot_arm_at(self.domain.robot, arm_pose),
                    arm_pose.name == self.domain.api_robot.arm_pose.name,
                )
        if self.domain.robot_has in fluents:
            for item in self.domain.items:
                problem.set_initial_value(
                    self.domain.robot_has(self.domain.robot, item), item.name == self.domain.api_robot.item.name
                )
        if self.domain.robot_offered in fluents:
            problem.set_initial_value(self.domain.robot_offered(self.domain.robot), self.domain.api_robot.item_offered)

    def run(self) -> None:
        active = True
        while active:
            # Create problem based on current state.
            self.problem = self.domain.initialize_problem()
            self.set_values(self.problem)
            self.domain.set_goals(self.problem)

            # Plan
            actions = self.domain.solve(self.problem)
            if not actions:
                print("Execution ended because no plan could be found.")
                break

            print("> Plan:")
            up_actions = [up_action for up_action, _ in actions]
            print('\n'.join(map(str, up_actions)))
            if self.visualization:
                self.visualization.set_actions(up_actions)
            else:
                self.visualization = PlanVisualization(up_actions)
            # ... and execute.
            print("> Execution:")
            for up_action, api_action in actions:
                print(up_action)
                self.visualization.execute(up_action)
                result = api_action()
                if result is None or result:
                    self.visualization.succeed(up_action)
                else:
                    print("-- Action failed! Need to replan.")
                    self.visualization.fail(up_action)
                    # Abort execution and loop to planning.
                    break
            else:
                active = False
                print("Task complete.")
