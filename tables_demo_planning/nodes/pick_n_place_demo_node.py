#!/usr/bin/env python3
import unified_planning
from unified_planning.model.problem import Problem
from tables_demo_planning.demo_domain import Domain
from tables_demo_planning.plan_execution import Executor

"""Execution of the pick & place demo."""


class PickAndPlace(Domain):
    def initialize_problem(self) -> Problem:
        actions = [self.move_base, self.transport, self.move_arm, self.pick, self.place]
        if not self.api_robot.item_offered:
            actions.append(self.hand_over)
        return self.define_problem(
            fluents=(self.robot_at, self.robot_arm_at, self.robot_has, self.robot_offered),
            items=(self.nothing, self.power_drill),
            actions=actions,
        )

    def set_goals(self, problem: Problem) -> None:
        problem.add_goal(self.robot_offered(self.robot))
        problem.add_goal(self.robot_has(self.robot, self.nothing))
        problem.add_goal(self.robot_at(self.robot, self.base_home_pose))


if __name__ == '__main__':
    unified_planning.shortcuts.get_env().credits_stream = None
    Executor(PickAndPlace()).run()
