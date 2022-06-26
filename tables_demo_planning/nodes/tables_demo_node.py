#!/usr/bin/env python3
from typing import Dict
import unified_planning
import rospy
from std_srvs.srv import SetBool
from geometry_msgs.msg import Pose
from unified_planning.model.problem import Problem
from unified_planning.shortcuts import Equals
from symbolic_fact_generation import on_fact_generator
from tables_demo_planning.demo_domain import ArmPose, Domain, Item, Location, Robot
from tables_demo_planning.subplan_visualization import SubPlanVisualization

"""
Main execution node of the tables demo.
Development in progress.
"""


class TableDemoRobot(Robot):
    activate_pose_selector = rospy.ServiceProxy('/pose_selector_activate', SetBool)

    def __init__(self, namespace: str) -> None:
        super().__init__(namespace)
        self.believed_item_locations: Dict[Item, Location] = {}

    def pick_item(self, pose: Pose, item: Item) -> bool:
        self.arm.move("observe100cm_right")
        self.arm_pose = ArmPose.observe100cm_right
        self.perceive()
        return True

    def perceive(self) -> Dict[Item, Location]:
        self.arm.move("observe100cm_right")
        rospy.wait_for_service('/pose_selector_activate')
        self.activate_pose_selector(True)
        rospy.sleep(5)
        facts = on_fact_generator.get_current_facts()
        self.activate_pose_selector(False)
        item_values = [item.value for item in Item]
        location_values = [location.value for location in Location]
        perceived_item_locations: Dict[Item, Location] = {}
        for fact in facts:
            if fact.name == "on":
                item, table = fact.values
                print(f"{item} on {table} detected.")
                if item in item_values and table in location_values:
                    perceived_item_locations[Item(item)] = Location(table)
        self.believed_item_locations.update(perceived_item_locations)
        print(self.believed_item_locations)
        return perceived_item_locations


class TablesDemo(Domain):
    def __init__(self) -> None:
        super().__init__(TableDemoRobot)
        self.pick_item, (robot, x, item) = self.create_action(TableDemoRobot, TableDemoRobot.pick_item)
        self.pick_item.add_precondition(Equals(self.robot_has(robot), self.nothing))
        self.pick_item.add_precondition(Equals(self.robot_at(robot), x))
        self.pick_item.add_effect(self.robot_has(robot), item)

        self.problem = self.define_problem(
            fluents=(self.robot_at, self.robot_arm_at, self.robot_has),
            actions=(self.move_base, self.move_arm, self.pick_item),
        )

    def initialize_problem(self) -> Problem:
        return self.problem

    def set_goals(self, problem: Problem) -> None:
        problem.add_goal(Equals(self.robot_has(self.robot), self.power_drill))
        problem.add_goal(Equals(self.robot_at(self.robot), self.objects[self.BASE_TABLE_1_POSE]))

    def run(self) -> None:
        visualization = SubPlanVisualization()
        active = True
        while active:
            # Create problem based on current state.
            self.problem = self.initialize_problem()
            self.set_values(self.problem)
            self.set_goals(self.problem)

            # Plan
            actions = self.solve(self.problem)
            if not actions:
                print("Execution ended because no plan could be found.")
                break

            print("> Plan:")
            up_actions = [up_action for up_action, _ in actions]
            print('\n'.join(map(str, up_actions)))
            visualization.set_actions(up_actions)
            # ... and execute.
            print("> Execution:")
            for up_action, (method, parameters) in actions:
                print(up_action)
                visualization.execute(up_action)
                result = method(*parameters)
                if result is None or result:
                    visualization.succeed(up_action)
                else:
                    print("-- Action failed! Need to replan.")
                    visualization.fail(up_action)
                    # Abort execution and loop to planning.
                    break
            else:
                active = False
                print("Task complete.")


if __name__ == '__main__':
    unified_planning.shortcuts.get_env().credits_stream = None
    TablesDemo().run()
