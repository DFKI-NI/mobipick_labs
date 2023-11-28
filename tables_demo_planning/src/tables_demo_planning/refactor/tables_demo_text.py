from typing import Dict, List, Optional
from collections import defaultdict
from geometry_msgs.msg import Point, Pose
from unified_planning.plans import ActionInstance
from tables_demo_planning.refactor.components import ArmPose, Item, Location, Robot
from tables_demo_planning.refactor.tables_demo import EnvironmentRepresentation, TablesDemoDomain

"""This script provides text-based simulation and execution methods without ROS."""


# Note: Purpose of this Robot subclass is to showcase usage of methods for robot actions
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

    def pick_item(self, pose: Pose, location: Location, item: Item) -> bool:
        location = self.sim.env.resolve_search_location(location)
        perceived_item_locations = self.perceive(location)
        if perceived_item_locations.get(item) != location:
            print(f"Cannot find {item.name} at {location.name}. Pick item FAILED!")
            return False

        if self.sim.env.actual_item_locations[item] != location:
            print(f"Pick up {item.name} at {location.name} FAILED!")
            return False

        self.sim.env.actual_item_locations[item] = Location.get("on_robot")
        return self.sim.env.pick_item(self, pose, location, item)

    def place_item(self, pose: Pose, location: Location, item: Item) -> bool:
        location = self.sim.env.resolve_search_location(location)
        if item != self.sim.env.robot_items[self]:
            print(f"Item {item.name} is not on the robot. Place item FAILED!")
            return False

        if self.sim.env.actual_item_locations[item] != Location.get("on_robot"):
            print(f"Place down {item.name} at {location.name} FAILED!")
            return False

        self.sim.env.actual_item_locations[item] = location
        return self.sim.env.place_item(self, pose, location, item)

    def store_item(self, pose: Pose, location: Location, tool: Item, klt: Item) -> bool:
        location = self.sim.env.resolve_search_location(location)
        perceived_item_locations = self.perceive(location)
        if perceived_item_locations.get(klt) != location:
            print(f"Cannot find '{klt.name}' at {location.name}. Store item FAILED!")
            return False

        if (
            self.sim.env.actual_item_locations[tool] != Location.get("on_robot")
            or self.sim.env.actual_item_locations[klt] != location
        ):
            print(f"Store {tool.name} into '{klt.name}' at {location.name} FAILED!")
            return False

        return self.sim.env.store_item(self, pose, location, tool, klt)

    def hand_over_item(self, item: Item) -> bool:
        print(f"{self} tries to hand over {item}.")
        return self.sim.env.hand_over_item(self, item)

    def perceive(self, location: Location) -> Dict[Item, Location]:
        # Determine newly perceived items and their locations.
        perceived_item_locations = {
            check_item: check_location
            for check_item, check_location in self.sim.env.actual_item_locations.items()
            if check_location == location
        }
        self.sim.env.newly_perceived_item_locations.clear()
        for perceived_item, perceived_location in perceived_item_locations.items():
            if (
                perceived_item not in self.sim.env.believed_item_locations.keys()
                or self.sim.env.believed_item_locations[perceived_item] != location
            ):
                self.sim.env.newly_perceived_item_locations[perceived_item] = perceived_location
        print(f"Newly perceived items: {[item.name for item in self.sim.env.newly_perceived_item_locations.keys()]}")
        # Remove all previously perceived items at location.
        for check_item, check_location in list(self.sim.env.believed_item_locations.items()):
            if check_location == location:
                del self.sim.env.believed_item_locations[check_item]
        # Add all currently perceived items at location.
        self.sim.env.believed_item_locations.update(perceived_item_locations)
        self.sim.env.searched_locations.add(location)
        self.sim.env.print_believed_item_locations()
        return perceived_item_locations


class Simulation:
    RETRIES_BEFORE_ABORTION = 2

    def __init__(self, item_locations: Dict[Item, Location]) -> None:
        self.mobipick = SimRobot("Mobipick", self)
        self.domain = TablesDemoDomain(self.mobipick)
        self.api_poses = {
            name: Pose(position=Point(x=float(index)))
            for index, name in enumerate(
                (
                    "base_home_pose",
                    "base_handover_pose",
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
        self.items = self.domain.create_objects({item.name: item for item in item_locations.keys()})
        self.tables = [
            self.domain.create_object(name, Location.get(name)) for name in ("table_1", "table_2", "table_3")
        ]
        self.env = EnvironmentRepresentation(item_locations)
        self.env.perceive = lambda _, location: self.mobipick.perceive(location)
        self.env.initialize_robot_states(self.domain.api_robot, self.api_poses["base_home_pose"])
        self.demo_items = list(item_locations.keys())

        self.domain.set_fluent_functions(
            [
                self.env.get_robot_at,
                self.env.get_robot_arm_at,
                self.env.get_robot_has,
                self.env.get_believe_item_at,
                self.env.get_believe_item_in,
                self.env.get_searched_at,
                self.env.get_item_offered,
                self.domain.get_pose_at,
            ]
        )
        self.domain.create_move_base_action(self.mobipick.move_base)
        self.domain.create_move_base_with_item_action(self.mobipick.move_base_with_item)
        self.domain.create_move_arm_action(self.mobipick.move_arm)
        self.domain.create_pick_item_action(self.mobipick.pick_item)
        self.domain.create_place_item_action(self.mobipick.place_item)
        self.domain.create_store_item_action(self.mobipick.store_item)
        self.domain.create_hand_over_item_action(self.mobipick.hand_over_item)
        self.domain.create_search_at_action(self.env.search_at, ["home", "observe100cm_right", "transport"])
        self.domain.create_search_tool_action(self.env.search_tool)
        self.domain.create_search_klt_action(self.env.search_klt)
        self.domain.create_conclude_tool_search_action(self.env.conclude_tool_search)
        self.domain.create_conclude_klt_search_action(self.env.conclude_klt_search)
        self.problem = self.domain.initialize_tables_demo_problem()
        self.subproblem = self.domain.initialize_item_search_problem()

    def replan(self) -> Optional[List[ActionInstance]]:
        self.env.print_believed_item_locations()
        self.domain.set_initial_values(self.problem)
        return self.domain.solve(self.problem)

    def run(self, target_location: Optional[Location] = None) -> None:
        """
        Run planning and execution loop.
        Note: Set the goals for self.problem before using planning.
        """
        retries_before_abortion = self.RETRIES_BEFORE_ABORTION
        error_counts: Dict[str, int] = defaultdict(int)
        # Solve overall problem.
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
                # Explicitly do not pick up KLT from target_table since planning does not handle it yet.
                if target_location and action.action.name == "pick_item" and parameters[-1].name.startswith("klt_"):
                    assert isinstance(parameters[-2], Location)
                    location = self.env.resolve_search_location(parameters[-2])
                    if location == target_location:
                        print("Picking up KLT OBSOLETE.")
                        actions = self.replan()
                        break

                # Execute action.
                result = executable_action(
                    *(parameters[1:] if hasattr(self.mobipick, action.action.name) else parameters)
                )

                # Handle item search as an inner execution loop.
                # Rationale: It has additional stop criteria, and might continue the outer loop.
                if self.env.item_search:
                    try:
                        # Check whether an obsolete item search invalidates the previous plan.
                        if self.env.believed_item_locations[self.env.item_search] != Location.get("anywhere"):
                            print(f"Search for {self.env.item_search.name} OBSOLETE.")
                            actions = self.replan()
                            break

                        # Search for item by creating and executing a subplan.
                        self.domain.set_initial_values(self.subproblem)
                        self.domain.set_search_goals(self.subproblem, self.env.item_search)
                        subactions = self.domain.solve(self.subproblem)
                        assert subactions, f"No solution for: {self.subproblem}"
                        print("- Search plan:")
                        print('\n'.join(map(str, subactions)))
                        print("- Search execution:")
                        subaction_execution_count = 0
                        for subaction in subactions:
                            executable_subaction, subparameters = self.domain.get_executable_action(subaction)
                            print(subaction)
                            # Execute search action.
                            result = executable_subaction(
                                *(subparameters[1:] if hasattr(self.mobipick, subaction.action.name) else subparameters)
                            )
                            subaction_execution_count += 1

                            if result is not None:
                                if result:
                                    # Note: True result only means any subaction succeeded.
                                    # Check if the search actually succeeded.
                                    if (
                                        self.env.item_search is None
                                        and len(self.env.newly_perceived_item_locations) <= 1
                                    ):
                                        print("- Continue with plan.")
                                        break
                                    # Check if the search found another item.
                                    elif self.env.newly_perceived_item_locations:
                                        self.env.newly_perceived_item_locations.clear()
                                        print("- Found another item, search ABORTED.")
                                        # Set result to None to trigger replanning.
                                        result = None
                                        break
                                else:
                                    break
                        # Note: The conclude action at the end of any search always fails.
                    finally:
                        # Always end the search at this point.
                        self.env.item_search = None

                if result is not None:
                    if result:
                        retries_before_abortion = self.RETRIES_BEFORE_ABORTION
                    else:
                        error_counts[self.domain.label(action)] += 1
                        # Note: This will also fail if two different failures occur successively.
                        if retries_before_abortion <= 0 or any(count >= 3 for count in error_counts.values()):
                            print("Task could not be completed even after retrying.")
                            return

                        retries_before_abortion -= 1
                        actions = self.replan()
                        break
                else:
                    retries_before_abortion = self.RETRIES_BEFORE_ABORTION
                    actions = self.replan()
                    break
            else:
                break
            if actions is None:
                print("Execution ended because no plan could be found.")
                return

        print("Demo complete.")
