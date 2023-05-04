#!/usr/bin/env python3

# Software License Agreement (BSD License)
#
#  Copyright (c) 2022, DFKI GmbH
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#   * Neither the name of Willow Garage nor the names of its
#     contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.
#
#
# Authors: Sebastian Stock

"""Main execution node of the Pick and Place demo using plexmo for execution."""


import rospy
import unified_planning
from typing import Optional, Set
from tables_demo_planning.pick_n_place_demo import PickAndPlaceDomain
from unified_planning.plans import Plan, ActionInstance
from unified_planning.shortcuts import OneshotPlanner, Problem
from unified_planning.engines import OptimalityGuarantee
from unified_planning.model import UPState
from unified_planning.model.metrics import MinimizeSequentialPlanLength
from up_esb.plexmo import SequentialPlanDispatcher
from up_esb.plexmo import SequentialPlanMonitor


class PickAndPlaceOrchestrator:
    def __init__(self) -> None:
        self._domain = PickAndPlaceDomain()

    def solve_problem(self, problem: Problem, planner_name: Optional[str] = None) -> Plan:
        result = OneshotPlanner(
            name=planner_name,
            problem_kind=problem.kind,
            optimality_guarantee=OptimalityGuarantee.SOLVED_OPTIMALLY,
        ).solve(problem)
        return result.plan

    def dispatch_cb(self, action: ActionInstance) -> bool:
        action_name = f"{len(self._executed_actions) + 1} {self._domain.label(action)}"

        self._domain.set_initial_values(self._problem)
        monitor = SequentialPlanMonitor(self._problem)
        state = UPState(self._problem.initial_values)
        unsatisfied = monitor.check_preconditions(action, state)[1]
        if len(unsatisfied) > 0:
            print("Preconditions of action %s are not satisfied" % action_name)
            print("Unsatisfied preconditions: %s" % str(unsatisfied))
            print("State is: ")
            for k, v in self._problem.initial_values.items():
                print(f"  {str(k)} := {str(v)}")
            # return False   # commented out until issue with arm state is resolved

        print("Dispatching " + action_name)
        executable_action, parameters = self._domain.get_executable_action(action)
        result = executable_action(*parameters)
        self._executed_actions.add(action_name)
        if result is None or result:
            return True
        else:
            return False

    def generate_and_execute_plan(self) -> None:
        self._executed_actions: Set[object] = set()

        # generate problem and plan
        self._problem = self._domain.initialize_problem()
        self._problem.add_quality_metric(MinimizeSequentialPlanLength())
        self._domain.set_initial_values(self._problem)
        self._domain.set_goals(self._problem)  # TODO take goal from external action
        # TODO use bridge.solve() once it returns plan
        print(self._problem)
        plan = self.solve_problem(self._problem)

        if not plan:
            print("Could not find a plan. Exiting.")
            return

        print("> Plan:")
        print("\n".join(map(str, plan.actions)))

        # execute the plan
        print("> Execution:")
        dispatcher = SequentialPlanDispatcher()
        dispatcher.set_dispatch_callback(self.dispatch_cb)
        dispatcher_result = dispatcher.execute_plan(plan)
        print(">Dispatcher finished with result " + str(dispatcher_result))


if __name__ == "__main__":
    unified_planning.shortcuts.get_environment().credits_stream = None
    try:
        PickAndPlaceOrchestrator().generate_and_execute_plan()
    except rospy.ROSInterruptException:
        pass
