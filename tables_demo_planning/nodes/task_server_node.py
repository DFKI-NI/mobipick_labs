#!/usr/bin/env python3

import rospy
import actionlib
from collections import defaultdict
from typing import Dict, List
from up_esb.plexmo import PlanDispatcher
from tables_demo_planning.msg import (
    PlanAndExecuteTasksAction,
    PlanAndExecuteTasksGoal,
    PlanAndExecuteTasksResult,
    PlanAndExecuteTasksFeedback,
)
from unified_planning.shortcuts import get_environment


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

    def set_goals(self, task: str, parameters: List[str]) -> None:
        """Set the goals given by the task message."""
        # TODO OLD uses goals and fluents from castle demo
        # Used to map tasks send via ros message to goals for the planner
        self._domain.problem.clear_goals()
        if task == "bring_item" and parameters and len(parameters) == 1:
            self._domain.problem.add_goal(self._domain.item_offered(self._domain.objects[parameters[0]]))
        elif task == "move_item" and parameters and len(parameters) == 2:
            self._domain.problem.add_goal(
                self._domain.believe_item_at(
                    self._domain.objects[parameters[0]],
                    self._domain.objects[parameters[1]],
                )
            )

    def set_item_locations(self) -> None:
        # TODO OLD still using items and locations from castle demo
        # HACK hardcoded initial object locations
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

            print("> Plan:")
            print("\n".join(map(str, plan.action_plan.actions)))

            actions = plan.action_plan.actions

            # Loop action execution as long as there are actions.
            while actions:
                print("> Execution:")
                for action in actions:
                    executable_action, parameters = self._domain.domain.get_executable_action(action)
                    print(action)

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
                        if not result:
                            error_counts[self._domain.tables_demo_api.label(action)] += 1
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
            self._task_server.publish_feedback(PlanAndExecuteTasksFeedback(success=exec_result, message=exec_msg))
            result_msg.success.append(exec_result)
            result_msg.message.append(exec_msg)

        if not any(result_msg.success):
            self._task_server.set_aborted(result_msg)
        else:
            print("Tasks complete.")
            self._task_server.set_succeeded(result_msg)


if __name__ == "__main__":
    get_environment().credits_stream = None
    try:
        TaskServerNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
