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
# Authors: Sebastian Stock, Alexander Sung

"""
Main execution node of the tables demo using plexmo for execution.
Development in progress.
"""


import sys
import rospy
import unified_planning
from typing import Optional, Set
from tables_demo_planning.mobipick_components import Location
from tables_demo_planning.tables_demo_api import TablesDemoAPIDomain
from tables_demo_planning.subplan_visualization import SubPlanVisualization
from unified_planning.plans import Plan, ActionInstance
from unified_planning.shortcuts import OneshotPlanner, Problem
from unified_planning.engines import OptimalityGuarantee
from up_esb.plexmo import SequentialPlanMonitor, SequentialPlanDispatcher
from unified_planning.model import UPState


class TablesDemoOrchestrator:
    def __init__(self) -> None:
        self._domain = TablesDemoAPIDomain()
        self.visualization: Optional[SubPlanVisualization] = None
        self.espeak_pub: Optional[rospy.Publisher] = None
        self._trigger_replanning = False  # Temporary solution until it is provided by dispatcher

    def solve_problem(self, problem: Problem, planner_name: Optional[str] = None) -> Plan:
        result = OneshotPlanner(
            name=planner_name,
            problem_kind=problem.kind,
            optimality_guarantee=OptimalityGuarantee.SOLVED_OPTIMALLY,
        ).solve(problem)
        return result.plan

    def dispatch_cb(self, action: ActionInstance) -> bool:
        action_name = f"{len(self._executed_actions) + 1} {self._domain.label(action)}"

        # check preconditions
        self._domain.set_initial_values(self._domain.problem)
        monitor = SequentialPlanMonitor(self._domain.problem)
        state = UPState(self._domain.problem.initial_values)
        unsatisfied = monitor.check_preconditions(action, state)[1]
        if len(unsatisfied) > 0:
            print("Preconditions of action %s are not satisfied" % action_name)
            print("Unsatisfied preconditions: %s" % str(unsatisfied))
            print("State is: ")
            for k, v in self._domain.problem.initial_values.items():
                print(f"  {str(k)} := {str(v)}")
            # return False # TODO commented out until issue with wrong state is resolved

        # Execute the action
        print("Dispatching " + action_name)
        executable_action, parameters = self._domain.get_executable_action(action)
        result = executable_action(*parameters)
        self._executed_actions.add(action_name)
        if rospy.is_shutdown():
            return False

        if self._domain.env.item_search:
            # Check whether an obsolete item search invalidates the previous plan.
            if (
                self._domain.env.believed_item_locations.get(self._domain.env.item_search, self._domain.anywhere)
                != self._domain.anywhere
            ):
                print(f"Search for {self._domain.env.item_search.name} OBSOLETE.")
                if self.visualization:
                    self.visualization.cancel(action_name)
                self._trigger_replanning = True
                result = None
            else:
                item_search_result = self.search_item(action_name)
                if not item_search_result:
                    result = None
                    self._trigger_replanning = True

        return result is not None and result

    def search_item(self, previous_action_name) -> object:
        try:
            # Search for item by creating and executing a subplan.
            self._domain.set_initial_values(self._domain.subproblem)
            self._domain.set_search_goals()
            subactions = self._domain.solve(self._domain.subproblem)
            assert subactions, f"No solution for: {self._domain.subproblem}"
            print("- Search plan:")
            print('\n'.join(map(str, subactions)))
            if self.visualization:
                self.visualization.set_actions(
                    [
                        f"{len(self._executed_actions)}{chr(number)} {self._domain.label(subaction)}"
                        for number, subaction in enumerate(subactions, start=97)
                    ],
                    preserve_actions=self._executed_actions,
                    predecessor=previous_action_name,
                )
            print("- Search execution:")
            subaction_execution_count = 0
            for subaction in subactions:
                executable_subaction, subparameters = self._domain.get_executable_action(subaction)
                label = self._domain.label(subaction)
                subaction_name = f"{len(self._executed_actions)}{chr(subaction_execution_count + 97)} {label}"
                print(subaction)
                if self.visualization:
                    self.visualization.execute(subaction_name)
                if self.espeak_pub:
                    self.espeak_pub.publish(self._domain.label(subaction))
                # Execute search action.
                result = executable_subaction(*subparameters)
                subaction_execution_count += 1
                if rospy.is_shutdown():
                    return "Interrupted"

                if result is not None:
                    if result:
                        # Note: True result only means any subaction succeeded.
                        # Check if the search actually succeeded.
                        if (
                            self._domain.env.item_search is None
                            and len(self._domain.env.newly_perceived_item_locations) <= 1
                        ):
                            print("- Continue with plan.")
                            if self.visualization:
                                self.visualization.succeed(subaction_name)
                            result = "Success"
                            break
                        # Check if the search found another item.
                        elif self._domain.env.newly_perceived_item_locations:
                            self._domain.env.newly_perceived_item_locations.clear()
                            print("- Found another item, search ABORTED.")
                            if self.visualization:
                                self.visualization.cancel(subaction_name)
                            if self.espeak_pub:
                                self.espeak_pub.publish("Found another item. Make a new plan.")
                            result = None
                            self._trigger_replanning = True
                            break
                    else:
                        if self.visualization:
                            self.visualization.fail(subaction_name)
                        break
            # Note: The conclude action at the end of any search always fails.
        finally:
            # Always end the search at this point.
            self._domain.env.item_search = None
        return result

    def generate_and_execute_plan(self, target_location: Location) -> None:
        print(
            "Scenario: Mobipick shall bring the box with the multimeter inside to"
            f"{self._domain.objects[target_location.name]}."
        )
        self._executed_actions: Set[str] = set()
        # retries_before_abortion = self._domain.RETRIES_BEFORE_ABORTION  # FIXME Currently not used
        # error_counts: Dict[str, int] = defaultdict(int) # FIXME Currently not used

        self._trigger_replanning = True  # only for entering the loop
        while self._trigger_replanning:
            self._trigger_replanning = False
            # Plan
            self._domain.set_initial_values(self._domain.problem)
            self._domain.set_goals(target_location)
            plan = self.solve_problem(self._domain.problem)
            if not plan:
                print("Could not find a plan. Exiting.")
                return

            print("> Plan:")
            print("\n".join(map(str, plan.actions)))

            # Execute the plan
            print("> Execution:")

            dispatcher = SequentialPlanDispatcher()
            dispatcher.set_dispatch_callback(self.dispatch_cb)
            dispatcher_result = dispatcher.execute_plan(plan)
            print(">Dispatcher finished with result " + str(dispatcher_result))
            if self._trigger_replanning:
                print("Replanning")


if __name__ == '__main__':
    unified_planning.shortcuts.get_environment().credits_stream = None
    try:
        target_location = Location.table_2
        if len(sys.argv) >= 2:
            parameter = sys.argv[1]
            if parameter in ("1", "table1", "table_1"):
                target_location = Location.table_1
            elif parameter in ("3", "table3", "table_3"):
                target_location = Location.table_3
            else:
                rospy.logwarn(f"Unknown parameter '{parameter}', using default table.")
        TablesDemoOrchestrator().generate_and_execute_plan(target_location)
    except rospy.ROSInterruptException:
        pass
