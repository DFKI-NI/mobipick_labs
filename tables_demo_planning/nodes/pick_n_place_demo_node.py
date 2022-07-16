#!/usr/bin/env python3
from typing import Optional, Set
import rospy
import unified_planning
from std_msgs.msg import String
from unified_planning.model.problem import Problem
from unified_planning.shortcuts import Not
from tables_demo_planning.mobipick_components import ArmPose, EnvironmentRepresentation, Item, Robot
from tables_demo_planning.demo_domain import Domain
from tables_demo_planning.subplan_visualization import SubPlanVisualization

"""Pick and Place application in the Mobipick domain"""


class PickAndPlaceRobot(Robot):
    def __init__(self, namespace: str, env: 'PickAndPlaceEnv') -> None:
        super().__init__(namespace)
        self.env = env

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
        self.env.item_offered = True
        if not self.arm.observe_force_torque(5.0, 25.0):
            return False

        self.arm.execute("ReleaseGripper")
        return True


class PickAndPlaceEnv(EnvironmentRepresentation[PickAndPlaceRobot]):
    def __init__(self) -> None:
        # Note: Instantiating the robot from env enables mutual references on initialization.
        super().__init__(PickAndPlaceRobot("mobipick", self))
        self.item_offered = False

    def get_item_offered(self) -> bool:
        return self.item_offered


class PickAndPlaceDomain(Domain):
    def __init__(self, env: PickAndPlaceEnv) -> None:
        super().__init__(env)
        self.item_offered = self.create_fluent(env.get_item_offered)

        self.pick, (_,) = self.create_action(PickAndPlaceRobot.pick_power_drill)
        self.pick.add_precondition(self.robot_at(self.base_table_2_pose))
        self.pick.add_precondition(self.robot_arm_at(self.arm_pose_home))
        self.pick.add_precondition(self.robot_has(self.nothing))
        self.pick.add_effect(self.robot_arm_at(self.arm_pose_home), False)
        self.pick.add_effect(self.robot_arm_at(self.arm_pose_interaction), True)
        self.pick.add_effect(self.robot_has(self.nothing), False)
        self.pick.add_effect(self.robot_has(self.power_drill), True)
        self.place, (_,) = self.create_action(PickAndPlaceRobot.place_power_drill)
        self.place.add_precondition(self.robot_at(self.base_table_3_pose))
        self.place.add_precondition(self.robot_arm_at(self.arm_pose_transport))
        self.place.add_precondition(self.robot_has(self.power_drill))
        self.place.add_effect(self.robot_arm_at(self.arm_pose_transport), False)
        self.place.add_effect(self.robot_arm_at(self.arm_pose_interaction), True)
        self.place.add_effect(self.robot_has(self.power_drill), False)
        self.place.add_effect(self.robot_has(self.nothing), True)
        self.hand_over, (_,) = self.create_action(PickAndPlaceRobot.hand_over)
        self.hand_over.add_precondition(self.robot_at(self.base_handover_pose))
        self.hand_over.add_precondition(self.robot_arm_at(self.arm_pose_transport))
        self.hand_over.add_precondition(Not(self.robot_has(self.nothing)))
        self.hand_over.add_effect(self.robot_arm_at(self.arm_pose_transport), False)
        self.hand_over.add_effect(self.robot_arm_at(self.arm_pose_interaction), True)
        for item in self.items:
            self.hand_over.add_effect(self.robot_has(item), item == self.nothing)
        self.hand_over.add_effect(self.item_offered, True)
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
        if not self.env.item_offered:
            actions.append(self.hand_over)
        return self.define_mobipick_problem(
            actions=actions,
            poses=[self.base_handover_pose, self.base_home_pose, self.base_table_2_pose, self.base_table_3_pose],
            items=[self.power_drill],
            locations=[],
        )

    def set_goals(self, problem: Problem) -> None:
        problem.add_goal(self.robot_at(self.base_home_pose))
        problem.add_goal(self.robot_has(self.nothing))
        problem.add_goal(self.item_offered)

    def run(self) -> None:
        self.visualization = SubPlanVisualization()
        executed_actions: Set[str] = set()
        error_count = 0
        active = True
        while active:
            # Create problem based on current state.
            problem = self.initialize_problem()
            self.set_initial_values(problem)
            self.set_goals(problem)

            # Plan
            actions = self.solve(problem)
            if not actions:
                print("Execution ended because no plan could be found.")
                self.visualization.add_node("Mission impossible", "red")
                return

            print("> Plan:")
            print('\n'.join(map(str, actions)))
            action_names = [
                f"{len(executed_actions) + index + 1} {self.label(action)}" for index, action in enumerate(actions)
            ]
            self.visualization.set_actions(action_names, preserve_actions=executed_actions)
            # ... and execute.
            print("> Execution:")
            for action in actions:
                function, parameters = self.get_executable_action(action)
                action_name = f"{len(executed_actions) + 1} {self.label(action)}"
                print(action)
                self.visualization.execute(action_name)
                self.espeak_pub.publish(self.label(action))
                result = function(*parameters)
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
        PickAndPlaceDomain(PickAndPlaceEnv()).run()
    except rospy.ROSInterruptException:
        pass
