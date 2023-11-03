#!/usr/bin/env python3

import rospy
import unified_planning
import actionlib
from collections import defaultdict
from typing import Dict
from up_esb.plexmo import PlanDispatcher
from tables_demo_planning.msg import (
    PlanAndExecuteTaskAction,
    PlanAndExecuteTaskGoal,
    PlanAndExecuteTaskResult,
)


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

        self._dispatcher = PlanDispatcher()

        self._task_server = actionlib.SimpleActionServer(
            task_server_name,
            PlanAndExecuteTaskAction,
            execute_cb=self.generate_and_execute_plan,
            auto_start=False,
        )
        self._task_server.register_preempt_callback(self.preempt_cb)
        self._task_server.start()

    def preempt_cb(self) -> None:
        self._domain.problem.clear_goals()

    def generate_and_execute_plan(self, request: PlanAndExecuteTaskGoal) -> None:
        retries_before_abortion = 3
        error_counts: Dict[str, int] = defaultdict(int)
        plan = self._domain.create_plan(request.task, request.parameters)

        if not plan:
            print("Could not find a plan. Exiting.")
            self._task_server.set_aborted(
                result=PlanAndExecuteTaskResult(success=False, message="Could not find a plan!")
            )
            return

        print("> Plan:")
        print("\n".join(map(str, plan.action_plan.actions)))

        actions = plan.action_plan.actions

        # Loop action execution as long as there are actions.
        while actions:
            print("> Execution:")
            for action in actions:
                if action.action.name == "trigger_replanning":
                    plan = self._domain.create_plan(request.task, request.parameters)
                    actions = plan.action_plan.actions
                    break
                executable_action, parameters = self._domain.get_executable_action(action)
                print(action)

                # Execute action.
                result = executable_action(*parameters)
                if rospy.is_shutdown():
                    self._task_server.set_aborted(
                        PlanAndExecuteTaskResult(success=False, message="Plan execution failed!")
                    )
                    return
                if result is not None:
                    if not result:
                        error_counts[self._domain.label(action)] += 1
                        # Note: This will also fail if two different failures occur successively.
                        if retries_before_abortion <= 0 or any(count >= 3 for count in error_counts.values()):
                            print("Task could not be completed even after retrying.")
                            return

                        retries_before_abortion -= 1
                        plan = self._domain.create_plan(request.task, request.parameters)
                        actions = plan.action_plan.actions
                        break
                else:
                    retries_before_abortion = 3
                    plan = self._domain.create_plan(request.task, request.parameters)
                    actions = plan.action_plan.actions
                    break
            else:
                self._task_server.set_aborted(PlanAndExecuteTaskResult(success=False, message="Plan execution failed!"))
                break
            if actions is None:
                print("Execution ended because no plan could be found.")
                self._task_server.set_aborted(PlanAndExecuteTaskResult(success=False, message="Plan execution failed!"))
                break

        print("Demo complete.")

        self._task_server.set_succeeded(PlanAndExecuteTaskResult(success=True, message="Successfully executed task!"))


if __name__ == "__main__":
    unified_planning.shortcuts.get_environment().credits_stream = None
    try:
        TaskServerNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
