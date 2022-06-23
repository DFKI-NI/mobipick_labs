#!/usr/bin/env python3
import unified_planning
from unified_planning.model.problem import Problem
from tables_demo_planning.demo_domain import Domain
from tables_demo_planning.plan_execution import Executor

"""
Main execution node of the tables demo.
Currently, it only performs the simple pick & place demo.
"""


class TablesDemo(Domain):
    def __init__(self) -> None:
        super().__init__()
        self.problem = self.define_problem()

    def initialize_problem(self) -> Problem:
        return self.problem

    def set_goals(self, problem: Problem) -> None:
        problem.add_goal(self.robot_offered(self.robot))
        problem.add_goal(self.robot_has(self.robot, self.nothing))
        problem.add_goal(self.robot_at(self.robot, self.base_home_pose))


if __name__ == '__main__':
    unified_planning.shortcuts.get_env().credits_stream = None
    Executor(TablesDemo()).run()
