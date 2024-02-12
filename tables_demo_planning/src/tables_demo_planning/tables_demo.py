# Software License Agreement (BSD License)
#
#  Copyright (c) 2022, 2023, DFKI GmbH
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#   * Neither the name of Willow Garage nor the names of its
#     contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.
#
#
# Authors: Alexander Sung, Oscar Lima, Marc Vinci

"""
The Tables Demo domain and environment specify concrete instances of the scenario for planning
 and include methods valid for both simulation and reality. It does not handle execution yet.
"""


from typing import Callable, Dict, Iterable, List, Optional, Sequence, Set, Tuple
from collections import defaultdict
import re
from geometry_msgs.msg import Pose
from unified_planning.exceptions import UPValueError
from unified_planning.model import Object, Problem
from unified_planning.plans import ActionInstance
from unified_planning.shortcuts import And, Or
from tables_demo_planning.domain import Domain
from tables_demo_planning.components import ArmPose, Item, Location, Robot


def parse_goal(goal_str: str) -> Tuple[str, List[str]]:
    # goal should have the format fluent_name(param1, param2, ..., paramn)
    regex = r"(\w+)\((\w+(?:,\s*\w+)*)\)"
    match = re.match(regex, goal_str)
    if match:
        goal_name = match.group(1)
        params_str = match.group(2)
        params = [param.strip() for param in params_str.split(',')]
        return goal_name, params
    else:
        raise ValueError(f"Invalid goal string: {goal_str}")


class TablesDemoDomain(Domain):
    def __init__(self, robot: Robot) -> None:
        super().__init__()
        self.api_robot = robot
        self.robot = self.get(Robot, robot.name)
        self.pose_locations: Dict[Object, Object] = {}
        self.arm_pose_unknown = self.get(ArmPose, "unknown")
        self.nothing = self.get(Item, "nothing")
        self.anywhere = self.get(Location, "anywhere")

        self.fluent_name_alternatives: Dict[str, str] = {}
        self.fluent_name_alternatives["robot_at"] = "get_robot_at"
        self.fluent_name_alternatives["robot_arm_at"] = "get_robot_arm_at"
        self.fluent_name_alternatives["arm_at"] = "get_robot_arm_at"
        self.fluent_name_alternatives["robot_has"] = "get_robot_has"
        self.fluent_name_alternatives["has"] = "get_robot_has"
        self.fluent_name_alternatives["believe_item_at"] = "get_believe_item_at"
        self.fluent_name_alternatives["item_at"] = "get_believe_item_at"
        self.fluent_name_alternatives["item_offered"] = "get_item_offered"
        self.fluent_name_alternatives["offered"] = "get_item_offered"

        # Create visualization labels for actions as functions of their parameters.
        self.method_labels: Dict[str, Callable[[Sequence[str]], str]] = {
            "move_base": lambda parameters: f"Move to {parameters[-1]}",
            "move_base_with_item": lambda parameters: f"Transport {parameters[1]} to {parameters[-1]}",
            "move_arm": lambda parameters: f"Move arm to its {parameters[-1]} pose",
            "pick_item": lambda parameters: f"Pick up {parameters[-1]}",
            "place_item": lambda parameters: f"Place {parameters[-1]} onto table",
            "store_item": lambda parameters: f"Place {parameters[-2]} into KLT {parameters[-1]}",
            "hand_over_item": lambda parameters: f"Handover {parameters[-1]} to person",
            "search_at": lambda parameters: f"Search at {parameters[-1]}",
            "search_tool": lambda parameters: f"Search tables for {parameters[-1]}",
            "search_klt": lambda _: "Search tables for the KLT",
            "conclude_tool_search": lambda parameters: f"Conclude search for {parameters[-1]}",
            "conclude_klt_search": lambda _: "Conclude search for the KLT",
        }
        self.parameter_labels: Dict[str, str] = {
            "base_home_pose": "home",
            "base_handover_pose": "handover",
            "base_pick_pose": "pick",
            "base_place_pose": "place",
            "base_table_1_pose": "table_1",
            "base_table_2_pose": "table_2",
            "base_table_3_pose": "table_3",
            "tool_search_pose": "where tool has been found",
            "klt_search_pose": "where KLT has been found",
        }

    def initialize_pose_locations(self) -> None:
        """Initialize information which pose is at which location."""
        for name, obj in self.objects.items():
            if name.startswith("base_table_") and name.endswith("_pose"):
                self.pose_locations[obj] = self.get(Location, name[5:-5])
            elif name.endswith("_search_pose"):
                self.pose_locations[obj] = self.get(Location, name[:-4] + "location")
        assert self.pose_locations

    def get_pose_at(self, pose: Object, location: Object) -> bool:
        """Return fluent value whether pose is at location."""
        if not self.pose_locations:
            self.initialize_pose_locations()
        return location == (self.pose_locations[pose] if pose in self.pose_locations.keys() else self.anywhere)

    def initialize_tables_demo_problem(self) -> Problem:
        """
        Create UP Problem including all fluents and actions needed by the overall Tables Demo.
        Note: Initial states and goals are not defined yet.
        """
        problem = self.define_problem(
            fluents=(
                self.robot_at,
                self.robot_arm_at,
                self.robot_has,
                self.believe_item_at,
                self.believe_item_in,
                self.pose_at,
                self.item_offered,
            ),
            actions=(
                self._actions["move_base"],
                self._actions["move_base_with_item"],
                self._actions["move_arm"],
                self._actions["pick_item"],
                self._actions["place_item"],
                self._actions["store_item"],
                self._actions["hand_over_item"],
                self._actions["search_tool"],
                self._actions["search_klt"],
            ),
        )
        return problem

    def initialize_item_search_problem(self) -> Problem:
        """
        Create UP Problem including all fluents and actions needed by the item search subproblem.
        Note: Initial states and goals are not defined yet.
        """
        problem = self.define_problem(
            fluents=(
                self.robot_at,
                self.robot_arm_at,
                self.robot_has,
                self.believe_item_at,
                self.believe_item_in,
                self.searched_at,
                self.pose_at,
            ),
            actions=(
                self._actions["move_base"],
                self._actions["move_base_with_item"],
                self._actions["move_arm"],
                self._actions["search_at"],
                self._actions["conclude_tool_search"],
                self._actions["conclude_klt_search"],
            ),
        )
        return problem

    def set_goals_by_strs(self, problem: Problem, goal_strs: List[str]) -> None:
        for goal_str in goal_strs:
            try:
                goal_fluent_name, params = parse_goal(goal_str)
                if goal_fluent_name in self.fluent_name_alternatives.keys():
                    goal_fluent_name = self.fluent_name_alternatives[goal_fluent_name]
                if problem.has_fluent(goal_fluent_name):
                    goal_fluent = problem.fluent(goal_fluent_name)
                param_objs = [problem.object(param) for param in params]
                problem.add_goal(goal_fluent(*param_objs))
            except ValueError as e:
                print(e)
                print("Goal should have the format fluent_name(param1, param2, ..., paramn).")
                return
            except UPValueError as e:
                print("Could not set the goal for the goal string.")
                print(e)
                print("Available fluents: %s" % problem.fluents)
                print("Available parameters: %s" % problem.all_objects)

    def set_goals(self, problem: Problem, demo_items: List[Item], target_location: Location) -> None:
        """Set the goals for the overall demo."""
        assert self.believe_item_at in problem.fluents
        assert all(problem.object(item.name) for item in demo_items)
        assert problem.object(target_location.name)
        problem.clear_goals()
        if any(item.name.startswith("klt_") for item in demo_items):
            assert problem.object("in_klt")
            if any(item.name.startswith("multimeter_") for item in demo_items):
                problem.add_goal(
                    Or(
                        And(
                            self.believe_item_in(self.objects[item.name], klt),
                            self.believe_item_at(klt, self.objects[target_location.name]),
                        )
                        for item in demo_items
                        if item.name.startswith("multimeter_")
                        for klt in self.get_klt_objects()
                    )
                )
        else:
            # Note: Just for testing without KLT.
            if any(item.name.startswith("multimeter_") for item in demo_items):
                for item in demo_items:
                    if item.name.startswith("multimeter_"):
                        problem.add_goal(
                            self.believe_item_at(self.objects[item.name], self.objects[target_location.name])
                        )

    def set_search_goals(self, problem: Problem, item_search: Item) -> None:
        """Set the goals for the current item_search subproblem."""
        problem.clear_goals()
        assert self.believe_item_at in problem.fluents
        if item_search.name.startswith("klt_"):
            assert problem.object(item_search.name) and problem.object("klt_search_location")
            problem.add_goal(
                self.believe_item_at(self.get(Item, item_search.name), self.get(Location, "klt_search_location"))
            )
        else:
            assert problem.object(item_search.name) and problem.object("tool_search_location")
            problem.add_goal(
                self.believe_item_at(self.get(Item, item_search.name), self.get(Location, "tool_search_location"))
            )

    def label(self, action: ActionInstance) -> str:
        """Return a user-friendly label for visualizing action."""
        parameters = [parameter.object() for parameter in action.actual_parameters]
        parameter_labels = [self.parameter_labels.get(parameter.name, str(parameter)) for parameter in parameters]
        return self.method_labels[action.action.name](parameter_labels)


class EnvironmentRepresentation:
    def __init__(self, api_items: Iterable[Item]) -> None:
        self.api_items = api_items
        self.table_locations = [location for name, location in Location.instances.items() if name.startswith("table_")]
        self.robot_home_poses: Dict[Robot, Pose] = {}
        self.robot_poses: Dict[Robot, Pose] = {}
        self.robot_arm_poses: Dict[Robot, ArmPose] = defaultdict(lambda: ArmPose.get("unknown"))
        self.robot_items: Dict[Robot, Item] = defaultdict(lambda: Item.get("nothing"))
        self.believed_item_locations: Dict[Item, Location] = defaultdict(lambda: Location.get("anywhere"))
        self.believed_klt_contents: Dict[Item, Set[Item]] = defaultdict(set)
        self.newly_perceived_item_locations: Dict[Item, Location] = {}
        self.item_search: Optional[Item] = None
        self.searched_locations: Set[Location] = set()
        self.search_location = Location.get("anywhere")
        self.offered_items: Set[Item] = set()

        # Provide a function which returns all perceived items with their locations.
        #  Note: Due to request, this is not an abstract method anymore.
        self.perceive: Callable[[Robot, Location], Dict[Item, Location]]

    def initialize_robot_states(
        self,
        robot: Robot,
        pose: Optional[Pose] = None,
        arm_pose: Optional[ArmPose] = None,
        item: Optional[Item] = None,
    ) -> None:
        if pose:
            self.robot_home_poses[robot] = self.robot_poses[robot] = pose
        if arm_pose:
            self.robot_arm_poses[robot] = arm_pose
        if item:
            self.robot_items[robot] = item

    def get_robot_at(self, robot: Robot, pose: Pose) -> bool:
        """Return fluent value whether robot is at pose."""
        return self.robot_poses[robot] == pose

    def get_robot_arm_at(self, robot: Robot, arm_pose: ArmPose) -> bool:
        """Return fluent value whether robot arm is at arm_pose."""
        return self.robot_arm_poses[robot] == arm_pose

    def get_robot_has(self, robot: Robot, item: Item) -> bool:
        """Return fluent value whether robot has item."""
        return self.robot_items[robot] == item

    def get_item_offered(self, item: Item) -> bool:
        """Return fluent value whether item has already been offered."""
        return item in self.offered_items

    def get_believe_item_at(self, item: Item, location: Location) -> bool:
        """Return fluent value whether item is believed to be at location."""
        return location == self.believed_item_locations[item]

    def get_believe_item_in(self, item: Item, klt: Item) -> bool:
        """Return fluent value whether item is believed to be in klt."""
        return item in self.believed_klt_contents[klt]

    def get_searched_at(self, location: Location) -> bool:
        """Return fluent value whether robot has already searched at location."""
        return location in self.searched_locations

    def resolve_search_location(self, location: Location) -> Location:
        """Resolve a location symbol to the actual table where the search succeeded."""
        if location.name not in ("tool_search_location", "klt_search_location"):
            return location

        assert self.search_location in self.table_locations
        return self.search_location

    def print_believed_item_locations(self) -> None:
        """Print at which locations the items are believed to be."""
        print("The believed item locations are:")
        for item in self.api_items:
            print(f"- {item.name}:", self.believed_item_locations[item].name)

    def move_base(self, robot: Robot, _: Pose, pose: Pose) -> bool:
        """Move robot base to pose."""
        self.robot_poses[robot] = pose
        return True

    def move_base_with_item(self, robot: Robot, item: Item, _: Pose, pose: Pose) -> bool:
        """Move robot base with item to pose."""
        # Note: Same action as move_base(), just with item and arm in transport pose.
        return self.move_base(robot, _, pose)

    def move_arm(self, robot: Robot, _: ArmPose, arm_pose: ArmPose) -> bool:
        """Move robot arm to arm_pose."""
        self.robot_arm_poses[robot] = arm_pose
        return True

    def pick_item(self, robot: Robot, pose: Pose, location: Location, item: Item) -> bool:
        """
        At pose, let robot look for item at location, pick it up, then move its arm to transport pose.
        """
        print(f"Successfully picked up {item.name}.")
        self.robot_items[robot] = item
        self.believed_item_locations[item] = Location.get("on_robot")
        for contents in self.believed_klt_contents.values():
            if item in contents:
                contents.remove(item)
        self.robot_arm_poses[robot] = ArmPose.get("transport")
        return True

    def place_item(self, robot: Robot, pose: Pose, location: Location, item: Item) -> bool:
        """At pose, let robot place item at location, then move its arm to home pose."""
        print(f"Successfully placed {item.name}.")
        self.robot_items[robot] = Item.get("nothing")
        self.believed_item_locations[item] = location
        self.robot_arm_poses[robot] = ArmPose.get("home")
        return True

    def store_item(self, robot: Robot, pose: Pose, location: Location, tool: Item, klt: Item) -> bool:
        """At pose, let robot store tool into KLT at location, then move its arm to home pose."""
        print(f"Successfully inserted {tool.name} into KLT {klt.name}.")
        self.robot_items[robot] = Item.get("nothing")
        self.believed_item_locations[tool] = Location.get("in_klt")
        self.believed_klt_contents[klt].add(tool)
        self.robot_arm_poses[robot] = ArmPose.get("home")
        return True

    def hand_over_item(self, robot: Robot, item: Item) -> bool:
        """At pose, let robot hand over item held to a person."""
        print(f"Successfully handed {item.name} over.")
        self.robot_items[robot] = Item.get("nothing")
        self.believed_item_locations[item] = Location.get("anywhere")
        self.robot_arm_poses[robot] = ArmPose.get("handover")
        self.offered_items.add(item)
        return True

    def search_at(self, robot: Robot, pose: Pose, location: Location) -> bool:
        """At pose, let robot search for item_search at location."""
        item_locations = self.perceive(robot, location)
        if self.item_search is not None:
            self.search_location = location
            item = self.item_search
            assert item
            if item in item_locations.keys():
                print(f"Search for {item.name} SUCCESSFUL.")
                self.item_search = None
        return True

    def check_reset_search(self) -> None:
        """Reset search if all tables have been searched."""
        if self.searched_locations.issuperset(self.table_locations):
            self.searched_locations.clear()

    def search_tool(self, robot: Robot, item: Item) -> bool:
        """Let robot initiate search for the tool item."""
        self.item_search = item
        self.check_reset_search()
        return True

    def search_klt(self, robot: Robot, item: Item) -> bool:
        """Let robot initiate search for the KLT item."""
        self.item_search = item
        self.check_reset_search()
        return True

    def conclude_tool_search(self, item: Item) -> bool:
        """Conclue search for tool item as failed. Success is determined in search_at()."""
        print(f"Search for {item.name} FAILED!")
        return False

    def conclude_klt_search(self, item: Item) -> bool:
        """Conclue search for KLT item as failed. Success is determined in search_at()."""
        print(f"Search for KLT {item.name} FAILED!")
        return False
