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

        domain_class = rospy.get_param("~domain_class", default="DemoDayDomain")
        domain_module = rospy.get_param("~domain_module", default="tables_demo_planning.ai_day_demo")

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

    def solve_problem(self, problem: Problem) -> Plan:
        # TODO Remove or adapt for new domain
        """Solve planning problem and return plan."""
        result = OneshotPlanner(
            problem_kind=problem.kind,
            optimality_guarantee=OptimalityGuarantee.SOLVED_OPTIMALLY,
        ).solve(problem)
        if result.status and result.status.value > 2:
            if result.log_messages:
                rospy.logerr(f"Error during plan generation: {result.log_messages[0].message}")
            else:
                rospy.logerr(f"Error during plan generation: {result.status}")
        rospy.loginfo(f"Result received from '{result.engine_name}' planner.")
        return result.plan if result.plan else None

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
        initial_item_locations = {}
        initial_item_locations[Item.get("multimeter_1")] = Location.table_3
        initial_item_locations[Item.get("relay_1")] = Location.table_3
        initial_item_locations[Item.get("screwdriver_1")] = Location.table_3
        initial_item_locations[Item.get("klt_1")] = Location.table_2
        initial_item_locations[Item.get("power_drill_with_grip_1")] = Location.table_2
        # add only items, which are not already set to avoid overriding perceived locations
        for item in list(initial_item_locations.keys()):
            if item in self._domain.env.believed_item_locations:
                del initial_item_locations[item]
        self._domain.env.believed_item_locations.update(initial_item_locations)

    def generate_plan(self, request) -> Plan:
        # TODO Remove or change to new domain
        # generate problem and plan
        self._domain.set_initial_values(self._domain.problem)
        self.set_goals(request.task, request.parameters)
        return self.solve_problem(self._domain.problem)

    def generate_and_execute_plan(self, request: PlanAndExecuteTaskGoal) -> None:
        # HACK setting initial item locations
        # self.set_item_locations()

        # TODO replace with function provided by domain to generate a plan
        # plan = self.generate_plan(request)
        plan = self._domain.calculate_plan_to_get_all_items(["Item_1", "Item_2"])

        if not plan:
            print("Could not find a plan. Exiting.")
            self._task_server.set_aborted(
                result=PlanAndExecuteTaskResult(success=False, message="Could not find a plan!")
            )
            return

        print("> Plan:")
        print("\n".join(map(str, plan)))

        # TODO Uncomment execution, change as needed for new domain

        # graph = self._domain.get_executable_graph(plan)

        # # execute the plan
        # print("> Execution:")
        # dispatcher_result = self._dispatcher.execute_plan(plan, graph)
        # print(">Dispatcher finished with result " + str(dispatcher_result))
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
