#!/usr/bin/env python3
from typing import Dict, List, Optional
from geometry_msgs.msg import Point, Pose
import unified_planning
import unified_planning.shortcuts
from unified_planning.plans import ActionInstance
from tables_demo_planning.refactor.components import ArmPose, Item, Location, Robot
from tables_demo_planning.refactor.tables_demo import EnvironmentRepresentation, TablesDemoDomain

"""This test script tests planning, re-planning, and text-based execution without ROS."""


# Note: Purpose of this robot class is to showcase usage of methods for robot actions
#  in a separate class. They could be easily implemented in the Simulation class instead.
class SimRobot(Robot):
    def __init__(self, name: str, sim: 'Simulation') -> None:
        super().__init__(name)
        self.sim = sim

    def move_base(self, _: Pose, pose: Pose) -> bool:
        print(f"{self} moves its base to {self.sim.api_pose_names[id(pose)]}.")
        return self.sim.env.move_base(self, _, pose)

    def move_base_with_item(self, item: Item, _: Pose, pose: Pose) -> bool:
        return self.move_base(_, pose)

    def move_arm(self, _: ArmPose, arm_pose: ArmPose) -> bool:
        print(f"{self} moves its arm to {arm_pose}.")
        return self.sim.env.move_arm(self, _, arm_pose)


class Simulation:
    def __init__(self, item_locations: Dict[Item, Location]) -> None:
        self.mobipick = SimRobot("Mobipick", self)
        self.domain = TablesDemoDomain(self.mobipick)
        self.api_poses = {
            name: Pose(position=Point(x=float(index)))
            for index, name in enumerate(
                (
                    "base_handover_pose",
                    "base_home_pose",
                    "base_table_1_pose",
                    "base_table_2_pose",
                    "base_table_3_pose",
                    "tool_search_pose",
                    "klt_search_pose",
                    "unknown_pose",
                )
            )
        }
        self.api_pose_names = {id(pose): name for name, pose in self.api_poses.items()}
        self.poses = self.domain.create_objects(self.api_poses)
        self.tables = [
            self.domain.create_object(name, location)
            for name, location in Location.instances.items()
            if name.startswith("table_")
        ]
        self.env = EnvironmentRepresentation(item_locations.keys())
        self.env.initialize_robot_states(self.domain.api_robot, self.api_poses["base_home_pose"])

        self.domain.set_fluent_functions(
            [
                self.env.get_robot_at,
                self.env.get_robot_arm_at,
                self.env.get_robot_has,
                self.env.get_believe_item_at,
                self.env.get_searched_at,
                self.env.get_item_offered,
                self.domain.get_pose_at,
            ]
        )
        self.domain.create_move_base_action(self.mobipick.move_base)
        self.domain.create_move_base_with_item_action(self.mobipick.move_base_with_item)
        self.domain.create_move_arm_action(self.mobipick.move_arm)
        self.domain.create_pick_item_action(self.env.pick_item)
        self.domain.create_place_item_action(self.env.place_item)
        self.domain.create_store_item_action(self.env.store_item)
        self.domain.create_search_at_action(self.env.search_at)
        self.domain.create_search_tool_action(self.env.search_tool)
        self.domain.create_search_klt_action(self.env.search_klt)
        self.domain.create_conclude_tool_search_action(self.env.conclude_tool_search)
        self.domain.create_conclude_klt_search_action(self.env.conclude_klt_search)
        self.problem = self.domain.define_problem()

    def replan(self) -> Optional[List[ActionInstance]]:
        self.env.print_believed_item_locations()
        self.domain.set_initial_values(self.problem)
        plan = self.domain.solve(self.problem)
        return plan.actions if plan else None

    def run(self, target_location: Location) -> None:
        """Run the mobipick tables demo."""
        print(f"Scenario: Mobipick shall bring the KLT with the multimeter inside to {target_location.name}.")

        # Solve overall problem.
        self.domain.set_goals(self.problem, target_location)
        actions = self.replan()
        if actions is None:
            print("Execution ended because no plan could be found.")
            return

        while actions:
            print("> Plan:")
            print('\n'.join(map(str, actions)))
            print("> Execution:")
            for action in actions:
                executable_action, parameters = self.domain.get_executable_action(action)
                print(action)

                # Execute action.
                executable_action(*(parameters[1:] if isinstance(parameters[0], Robot) else parameters))
            else:
                break


def test_tables_demo() -> None:
    unified_planning.shortcuts.get_environment().credits_stream = None
    # Define environment values.
    item_locations = {
        Item.get("multimeter_1"): Location.get("table_3"),
        Item.get("klt_1"): Location.get("table_1"),
    }
    # Define goal.
    target_location = Location.get("table_2")

    Simulation(item_locations).run(target_location)


if __name__ == "__main__":
    test_tables_demo()
