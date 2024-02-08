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
from typing import Optional, Set, Dict
from tables_demo_planning.components import Item, Location
from tables_demo_planning.tables_demo_api import TablesDemoAPI
from tables_demo_planning.subplan_visualization import SubPlanVisualization
from unified_planning.plans import ActionInstance
from up_esb.plexmo import PlanMonitor, PlanDispatcher
from unified_planning.model import UPState


class TablesDemoOrchestrator:
    def __init__(self, item_locations: Dict[Item, Location]) -> None:
        self._demo_api = TablesDemoAPI(item_locations)
        self.visualization = SubPlanVisualization()
        self.espeak_pub: Optional[rospy.Publisher] = None
        self._trigger_replanning = False  # Temporary solution until it is provided by dispatcher

    def dispatch_cb(self, action: ActionInstance) -> bool:
        action_name = f"{len(self._executed_actions) + 1} {self._demo_api.label(action)}"

        # check preconditions
        self._demo_api.set_initial_values(self._demo_api.problem)
        monitor = PlanMonitor(self._demo_api.problem)
        state = UPState(self._demo_api.problem.initial_values)
        unsatisfied = monitor.check_preconditions(action, state)[1]
        if len(unsatisfied) > 0:
            print("Preconditions of action %s are not satisfied" % action_name)
            print("Unsatisfied preconditions: %s" % str(unsatisfied))
            print("State is: ")
            for k, v in self._demo_api.problem.initial_values.items():
                print(f"  {str(k)} := {str(v)}")
            # return False # TODO commented out until issue with wrong state is resolved

        # Execute the action
        print("Dispatching " + action_name)
        executable_action, parameters = self._demo_api.get_executable_action(action)
        result = executable_action(*parameters)
        self._executed_actions.add(action_name)
        if rospy.is_shutdown():
            return False

        if self._demo_api.env.item_search:
            # Check whether an obsolete item search invalidates the previous plan.
            if (
                self._demo_api.env.believed_item_locations.get(self._demo_api.env.item_search, self._demo_api.anywhere)
                != self._demo_api.anywhere
            ):
                print(f"Search for {self._demo_api.env.item_search.name} OBSOLETE.")
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
            self._demo_api.set_initial_values(self._demo_api.subproblem)
            self._demo_api.set_search_goals()
            subactions = self._demo_api.solve(self._demo_api.subproblem)
            assert subactions, f"No solution for: {self._demo_api.subproblem}"
            print("- Search plan:")
            print('\n'.join(map(str, subactions)))
            if self.visualization:
                self.visualization.set_actions(
                    [
                        f"{len(self._executed_actions)}{chr(number)} {self._demo_api.label(subaction)}"
                        for number, subaction in enumerate(subactions, start=97)
                    ],
                    preserve_actions=self._executed_actions,
                    predecessor=previous_action_name,
                )
            print("- Search execution:")
            subaction_execution_count = 0
            for subaction in subactions:
                executable_subaction, subparameters = self._demo_api.get_executable_action(subaction)
                label = self._demo_api.label(subaction)
                subaction_name = f"{len(self._executed_actions)}{chr(subaction_execution_count + 97)} {label}"
                print(subaction)
                if self.visualization:
                    self.visualization.execute(subaction_name)
                if self.espeak_pub:
                    self.espeak_pub.publish(self._demo_api.label(subaction))
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
                            self._demo_api.env.item_search is None
                            and len(self._demo_api.env.newly_perceived_item_locations) <= 1
                        ):
                            print("- Continue with plan.")
                            if self.visualization:
                                self.visualization.succeed(subaction_name)
                            result = "Success"
                            break
                        # Check if the search found another item.
                        elif self._demo_api.env.newly_perceived_item_locations:
                            self._demo_api.env.newly_perceived_item_locations.clear()
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
            self._demo_api.env.item_search = None
        return result

    def generate_and_execute_plan(
        self, goal_strs: Optional[str] = None, target_location: Optional[Location] = None
    ) -> None:
        if goal_strs:
            self._demo_api.domain.set_goals_by_strs(self._demo_api.problem, goal_strs)
        else:  # Standard tables demo
            print(
                "Scenario: Mobipick shall bring the box with the multimeter"
                f" inside to {self._demo_api.domain.objects[target_location.name]}."
            )
            self._demo_api.domain.set_goals(self._demo_api.problem, list(item_locations.keys()), target_location)

        self._executed_actions: Set[str] = set()

        self._trigger_replanning = True  # only for entering the loop
        while self._trigger_replanning:
            self._trigger_replanning = False
            # Plan
            self._demo_api.domain.set_initial_values(self._demo_api.problem)
            plan = self._demo_api.domain.solve(self._demo_api.problem)
            if not plan:
                print("Could not find a plan. Exiting.")
                return

            print("> Plan:")
            print("\n".join(map(str, plan.actions)))

            # Execute the plan
            print("> Execution:")

            dispatcher = PlanDispatcher()
            dispatcher.set_dispatch_callback(self.dispatch_cb)
            graph = self._demo_api.domain.get_executable_graph(plan)
            dispatcher_result = dispatcher.execute_plan(plan, graph)
            print(">Dispatcher finished with result " + str(dispatcher_result))
            if self._trigger_replanning:
                print("Replanning")


if __name__ == '__main__':
    unified_planning.shortcuts.get_environment().credits_stream = None

    # Define environment values.
    item_locations = {
        Item.get("multimeter_1"): Location.get("table_3"),
        Item.get("klt_1"): Location.get("table_2"),
        Item.get("klt_2"): Location.get("table_1"),
        Item.get("klt_3"): Location.get("table_3"),
    }

    try:
        # Define goal.
        goal_strs = sys.argv[1:]
        tdo = TablesDemoOrchestrator(item_locations)

        if goal_strs:
            tdo.generate_and_execute_plan(goal_strs)
        else:  # Standard tables Demo goal
            target_location = Location.get("table_2")
            tdo.generate_and_execute_plan(goal_strs=None, target_location=target_location)

    except rospy.ROSInterruptException:
        pass
