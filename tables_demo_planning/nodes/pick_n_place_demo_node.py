#!/usr/bin/env python3
import unified_planning
from unified_planning.model.problem import Problem
from unified_planning.shortcuts import Equals
from tables_demo_planning.demo_domain import ArmPose, Domain, Item, Robot
from tables_demo_planning.plan_execution import Executor

"""Execution of the pick & place demo."""


class PickAndPlaceRobot(Robot):
    def get_initial_item(self) -> Item:
        self.arm.execute("HasAttachedObjects")
        return Item.power_drill if self.arm.get_result().result else Item.nothing

    def pick(self) -> bool:
        self.arm.execute("CaptureObject")
        self.arm_pose = self.get_arm_pose()
        self.arm.execute("PickUpObject")
        self.arm_pose = self.get_arm_pose()
        if not self.arm.get_result().result:
            return False

        self.item = Item.power_drill
        return True

    def place(self) -> bool:
        self.arm.execute("PlaceObject")
        self.arm_pose = ArmPose.place_1
        self.item = Item.nothing
        return True


class PickAndPlace(Domain):
    def __init__(self) -> None:
        super().__init__(PickAndPlaceRobot)
        self.pick, (robot,) = self.create_action(PickAndPlaceRobot, PickAndPlaceRobot.pick)
        self.pick.add_precondition(Equals(self.robot_has(robot), self.nothing))
        self.pick.add_precondition(Equals(self.robot_at(robot), self.base_pick_pose))
        self.pick.add_precondition(Equals(self.robot_arm_at(robot), self.arm_pose_home))
        self.pick.add_effect(self.robot_has(robot), self.power_drill)
        self.pick.add_effect(self.robot_arm_at(robot), self.arm_pose_interaction)
        self.place, (robot,) = self.create_action(PickAndPlaceRobot, PickAndPlaceRobot.place)
        self.place.add_precondition(Equals(self.robot_has(robot), self.power_drill))
        self.place.add_precondition(Equals(self.robot_at(robot), self.base_place_pose))
        self.place.add_precondition(Equals(self.robot_arm_at(robot), self.arm_pose_transport))
        self.place.add_effect(self.robot_has(robot), self.nothing)
        self.place.add_effect(self.robot_arm_at(robot), self.arm_pose_interaction)

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
        problem.add_goal(Equals(self.robot_has(self.robot), self.nothing))
        problem.add_goal(Equals(self.robot_at(self.robot), self.base_home_pose))


if __name__ == '__main__':
    unified_planning.shortcuts.get_env().credits_stream = None
    Executor(PickAndPlace()).run()
