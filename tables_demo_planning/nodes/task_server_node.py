#!/usr/bin/env python3

# Software License Agreement (BSD License)
#
#  Copyright (c) 2023, DFKI GmbH
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
# Authors: Marc Vinci, Sebastian Stock

import rospy
import actionlib
from collections import defaultdict
from typing import Dict
from std_msgs.msg import String
from up_esb.plexmo import PlanDispatcher
from tables_demo_planning.msg import (
    PlanAndExecuteTasksAction,
    PlanAndExecuteTasksGoal,
    PlanAndExecuteTasksResult,
    PlanAndExecuteTasksFeedback,
)
from unified_planning.shortcuts import get_environment
from unified_planning.plans import PlanKind
from tables_demo_planning.hierarchical_domain import HierarchicalDomain
from tables_demo_planning.UP_plan_visualization import UPPlanVisualization
from tables_demo_planning.components import Location, Item


class TaskServerNode:
    def __init__(self) -> None:
        rospy.init_node("task_server_node")
        task_server_name = rospy.get_param(
            "~task_server_name",
            default="/mobipick/task_planning",
        )

        domain_class = rospy.get_param("~domain_class", default="HierarchicalDomain")
        domain_module = rospy.get_param("~domain_module", default="tables_demo_planning.hierarchical_domain")

        try:
            # Initialize domain by importing python class and calling __init__ without arguments
            # using the module and class specified in the ros parameters
            self._domain = getattr(__import__(domain_module, fromlist=[domain_class]), domain_class)()
        except ImportError as e:
            print(f"Could not import {domain_module} module for {domain_class} domain: {e}")

        # Initialize plan visualization
        self.visualization = UPPlanVisualization()
        self.espeak_pub = rospy.Publisher("/espeak_node/speak_line", String, queue_size=1)

        self.initial_item_locations = self._domain.tables_demo_api.initial_item_locations

        self._dispatcher = PlanDispatcher()

        self._task_server = actionlib.SimpleActionServer(
            task_server_name,
            PlanAndExecuteTasksAction,
            execute_cb=self.generate_and_execute_plan,
            auto_start=False,
        )
        self._task_server.register_preempt_callback(self.preempt_cb)
        self._task_server.start()

    def preempt_cb(self) -> None:
        self._domain.problem.clear_goals()

    def set_item_locations(self) -> None:
        # add only items, which are not already set to avoid overriding perceived locations
        item_loc = {}
        for item, loc in self.initial_item_locations.items():
            if item not in self._domain.env.believed_item_locations:
                item_loc[item] = loc
        self._domain.env.believed_item_locations.update(item_loc)

    def generate_and_execute_plan(self, request: PlanAndExecuteTasksGoal) -> None:
        retries_before_abortion = 3
        error_counts: Dict[str, int] = defaultdict(int)
        self.set_item_locations()

        result_msg = PlanAndExecuteTasksResult()

        for task in request.tasks:
            plan = self._domain.create_plan(task.task, task.parameters)

            exec_result = True
            exec_msg = f"{task.task}{task.parameters} successfully executed!"

            if not plan:
                print("Could not find a plan for task.")
                result_msg.success.append(False)
                result_msg.message.append(f"Could not find a plan for task {task.task}{task.parameters}!")
                continue

            if plan.kind == PlanKind.HIERARCHICAL_PLAN:
                actions = HierarchicalDomain.get_actions_from_plan(plan)
                if actions is None:
                    result_msg.success.append(False)
                    result_msg.message.append("Received unexpected kind of plan!")
                    continue
            elif plan.kind == PlanKind.SEQUENTIAL_PLAN:
                actions = plan.actions
            else:
                rospy.logerr("Received unexpected kind of plan!")
                result_msg.success.append(False)
                result_msg.message.append("Received unexpected kind of plan!")
                continue

            if len(actions) == 0:
                empty_plan_printout = f"The plan for task {task.task}{task.parameters} is empty."
                print(empty_plan_printout)
                self.espeak_pub.publish(empty_plan_printout)

            # Variables to handle item search
            item_to_search = None
            if task.task == "search_item":
                item_to_search = task.parameters[1]

            # Loop action execution as long as there are actions.
            while actions:
                print("> Plan:")
                print("\n".join(map(str, actions)))
                self.visualization.set_plan(plan)
                print("> Execution:")
                for action in actions:
                    # if an item search is ongoing, stop when item is found
                    if item_to_search is not None and self._domain.env.believed_item_locations[
                        Item.get(item_to_search)
                    ] != Location.get("anywhere"):
                        print(f"Search for {item_to_search} finished.")
                        self.visualization.cancel(action)
                        break
                    executable_action, parameters = self._domain.domain.get_executable_action(action)
                    print(action)
                    self.visualization.execute(action)
                    self.espeak_pub.publish(self.visualization.label(action))

                    # Execute action.
                    result = executable_action(*parameters)
                    if rospy.is_shutdown():
                        result_msg.success.append(False)
                        result_msg.message.append(
                            f"Plan execution failed for task {task.task}{task.parameters}. ROS Node was shut down!"
                        )
                        self._task_server.set_aborted(result_msg)
                        return
                    if result is not None:
                        if result:
                            retries_before_abortion = self._domain.tables_demo_api.RETRIES_BEFORE_ABORTION
                            self.visualization.succeed(action)
                        else:
                            self.visualization.fail(action)
                            self.espeak_pub.publish("Action failed.")
                            error_counts[self.visualization.label(action)] += 1
                            # Note: This will also fail if two different failures occur successively.
                            if retries_before_abortion <= 0 or any(count >= 3 for count in error_counts.values()):
                                print("Task could not be completed even after retrying.")
                                result_msg.success.append(False)
                                result_msg.message.append(
                                    f"Task {task.task}{task.parameters} could not be completed even after retrying."
                                )
                                self._task_server.set_aborted(result_msg)
                                return

                            retries_before_abortion -= 1
                            plan = self._domain.create_plan(task.task, task.parameters)
                            actions = plan.action_plan.actions if plan is not None else None
                            break
                    else:
                        # Is this part still needed?
                        retries_before_abortion = 3
                        plan = self._domain.create_plan(task.task, task.parameters)
                        actions = plan.action_plan.actions if plan is not None else None
                        break
                else:
                    break
                if actions is None:
                    exec_result = False
                    exec_msg = (
                        f"Plan execution ended because no plan could be found for task {task.task}{task.parameters}!"
                    )
                    break
                elif item_to_search is not None and self._domain.env.believed_item_locations[
                    Item.get(item_to_search)
                ] != Location.get("anywhere"):
                    break
            self._task_server.publish_feedback(PlanAndExecuteTasksFeedback(success=exec_result, message=exec_msg))
            result_msg.success.append(exec_result)
            result_msg.message.append(exec_msg)

        if not any(result_msg.success):
            self._task_server.set_aborted(result_msg)
        else:
            print("Plan successfully executed.")
            self.espeak_pub.publish("Plan successfully executed.")
            self._task_server.set_succeeded(result_msg)


if __name__ == "__main__":
    get_environment().credits_stream = None
    try:
        TaskServerNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
