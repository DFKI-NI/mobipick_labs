#!/usr/bin/env python3

import rospy
import unified_planning
import actionlib
from typing import List
from unified_planning.plans import Plan
from unified_planning.shortcuts import OneshotPlanner, Problem
from unified_planning.engines import OptimalityGuarantee
from up_esb.plexmo import PlanDispatcher
from tables_demo_planning.msg import (
    PlanAndExecuteTaskAction,
    PlanAndExecuteTaskGoal,
    PlanAndExecuteTaskResult,
)
from tables_demo_planning.mobipick_components import Item, Location


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
        plan = self._domain.create_plan(request.task, request.parameters)

        if not plan:
            print("Could not find a plan. Exiting.")
            self._task_server.set_aborted(
                result=PlanAndExecuteTaskResult(success=False, message="Could not find a plan!")
            )
            return

        print("> Plan:")
        print("\n".join(map(str, plan.action_plan.actions)))

        graph = self._domain.get_executable_graph(plan.action_plan)

        # execute the plan
        print("> Execution:")
        dispatcher_result = self._dispatcher.execute_plan(plan.action_plan, graph)
        print(">Dispatcher finished with result " + str(dispatcher_result))
        dispatcher_result = True
        if dispatcher_result:
            self._task_server.set_succeeded(
                PlanAndExecuteTaskResult(success=True, message="Successfully executed task!")
            )
        else:
            self._task_server.set_aborted(PlanAndExecuteTaskResult(success=False, message="Plan execution failed!"))


if __name__ == "__main__":
    unified_planning.shortcuts.get_environment().credits_stream = None
    try:
        TaskServerNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
