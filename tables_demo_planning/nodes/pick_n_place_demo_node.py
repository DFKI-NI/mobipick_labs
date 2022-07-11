#!/usr/bin/env python3
from typing import Optional, Set
import rospy
import unified_planning
from std_msgs.msg import String
from unified_planning.model.problem import Problem
from unified_planning.shortcuts import Not
from tables_demo_planning.demo_domain import ArmPose, Domain, Item, Robot
from tables_demo_planning.subplan_visualization import SubPlanVisualization

"""Execution of the pick & place demo."""


class PickAndPlaceRobot(Robot):
    def get_initial_item(self) -> Item:
        self.arm.execute("HasAttachedObjects")
        return Item.power_drill if self.arm.get_result().result else Item.nothing

    def pick_power_drill(self) -> bool:
        self.arm.execute("CaptureObject")
        self.arm_pose = self.get_arm_pose()
        self.arm.execute("PickUpObject")
        self.arm_pose = self.get_arm_pose()
        if not self.arm.get_result().result:
            return False

        self.item = Item.power_drill
        return True

    def place_power_drill(self) -> bool:
        self.arm.execute("PlaceObject")
        self.arm_pose = ArmPose.place
        self.item = Item.nothing
        return True

    def hand_over(self) -> bool:
        self.arm.execute("MoveArmToHandover")
        self.arm_pose = self.get_arm_pose()
        self.item_offered = True
        if not self.arm.observe_force_torque(5.0, 25.0):
            return False

        self.arm.execute("ReleaseGripper")
        return True


class PickAndPlace(Domain):
    def __init__(self) -> None:
        super().__init__(PickAndPlaceRobot("mobipick"))
        self.pick, (robot,) = self.create_action(PickAndPlaceRobot, PickAndPlaceRobot.pick_power_drill)
        self.pick.add_precondition(self.robot_has(robot, self.nothing))
        self.pick.add_precondition(self.robot_at(robot, self.base_table_2_pose))
        self.pick.add_precondition(self.robot_arm_at(robot, self.arm_pose_home))
        self.pick.add_effect(self.robot_has(robot, self.nothing), False)
        self.pick.add_effect(self.robot_has(robot, self.power_drill), True)
        self.pick.add_effect(self.robot_arm_at(robot, self.arm_pose_home), False)
        self.pick.add_effect(self.robot_arm_at(robot, self.arm_pose_interaction), True)
        self.place, (robot,) = self.create_action(PickAndPlaceRobot, PickAndPlaceRobot.place_power_drill)
        self.place.add_precondition(self.robot_has(robot, self.power_drill))
        self.place.add_precondition(self.robot_at(robot, self.base_table_3_pose))
        self.place.add_precondition(self.robot_arm_at(robot, self.arm_pose_transport))
        self.place.add_effect(self.robot_has(robot, self.power_drill), False)
        self.place.add_effect(self.robot_has(robot, self.nothing), True)
        self.place.add_effect(self.robot_arm_at(robot, self.arm_pose_transport), False)
        self.place.add_effect(self.robot_arm_at(robot, self.arm_pose_interaction), True)
        self.hand_over, (robot,) = self.create_action(PickAndPlaceRobot, PickAndPlaceRobot.hand_over)
        self.hand_over.add_precondition(Not(self.robot_has(robot, self.nothing)))
        self.hand_over.add_precondition(self.robot_at(robot, self.base_handover_pose))
        self.hand_over.add_precondition(self.robot_arm_at(robot, self.arm_pose_transport))
        self.hand_over.add_effect(self.robot_arm_at(robot, self.arm_pose_transport), False)
        self.hand_over.add_effect(self.robot_arm_at(robot, self.arm_pose_interaction), True)
        for item in self.items:
            self.hand_over.add_effect(self.robot_has(robot, item), item == self.nothing)
        self.hand_over.add_effect(self.robot_offered(robot), True)
        self.visualization: Optional[SubPlanVisualization] = None
        self.method_labels.update(
            {
                self.pick: lambda _: "Pick up power drill",
                self.place: lambda _: "Place power drill on table",
                self.hand_over: lambda _: "Hand over power drill to person",
            }
        )
        self.espeak_pub = rospy.Publisher("/espeak_node/speak_line", String, queue_size=1)

    def initialize_problem(self) -> Problem:
        actions = [self.move_base, self.move_base_with_item, self.move_arm, self.pick, self.place]
        if not self.api_robot.item_offered:
            actions.append(self.hand_over)
        return self.define_problem(
            fluents=(self.robot_at, self.robot_arm_at, self.robot_has, self.robot_offered),
            items=(self.nothing, self.power_drill),
            locations=[],
            actions=actions,
        )

    def set_goals(self, problem: Problem) -> None:
        problem.add_goal(self.robot_offered(self.robot))
        problem.add_goal(self.robot_has(self.robot, self.nothing))
        problem.add_goal(self.robot_at(self.robot, self.base_home_pose))

    def run(self) -> None:
        self.visualization = SubPlanVisualization()
        executed_actions: Set[str] = set()
        error_count = 0
        active = True
        while active:
            # Create problem based on current state.
            self.problem = self.initialize_problem()
            self.set_initial_values(self.problem)
            self.set_goals(self.problem)

            # Plan
            actions = self.solve(self.problem)
            if not actions:
                print("Execution ended because no plan could be found.")
                self.visualization.add_node("Mission impossible", "red")
                return

            print("> Plan:")
            up_actions = [up_action for up_action, _ in actions]
            print('\n'.join(map(str, up_actions)))
            action_names = [
                f"{len(executed_actions) + index + 1} {self.label(up_action)}"
                for index, up_action in enumerate(up_actions)
            ]
            self.visualization.set_actions(action_names, preserve_actions=executed_actions)
            # ... and execute.
            print("> Execution:")
            for up_action, (method, parameters) in actions:
                action_name = f"{len(executed_actions) + 1} {self.label(up_action)}"
                print(up_action)
                self.visualization.execute(action_name)
                self.espeak_pub.publish(self.label(up_action))
                result = method(*parameters)
                executed_actions.add(action_name)
                if rospy.is_shutdown():
                    return

                if result is None or result:
                    self.visualization.succeed(action_name)
                else:
                    print("-- Action failed! Need to replan.")
                    error_count += 1
                    self.visualization.fail(action_name)
                    self.espeak_pub.publish("Action failed.")
                    if error_count >= 3:
                        print("Execution ended after too many failures.")
                        self.espeak_pub.publish("Mission impossible!")
                        self.visualization.add_node("Mission impossible", "red")
                        return

                    # Abort execution and loop to planning.
                    break
            else:
                active = False
                print("Demo complete.")
                self.espeak_pub.publish("Demo complete.")
                self.visualization.add_node("Demo complete", "green")


if __name__ == '__main__':
    unified_planning.shortcuts.get_env().credits_stream = None
    try:
        PickAndPlace().run()
    except rospy.ROSInterruptException:
        pass
