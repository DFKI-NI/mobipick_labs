# Software License Agreement (BSD License)
#
#  Copyright (c) 2022, DFKI GmbH
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
These Mobipick components connect the planning domain with the application domain,
 and include methods valid for both simulation and reality.
"""


from typing import Dict, Generic, List, Optional, Set, TypeVar
from abc import ABC, abstractmethod
from collections import defaultdict
import rospy
from geometry_msgs.msg import Pose
from unified_planning.model import Object
from unified_planning.model.metrics import MinimizeSequentialPlanLength
from unified_planning.plans import ActionInstance
from unified_planning.shortcuts import Equals, Not, Or, Exists, Variable, And
from tables_demo_planning.demo_domain import Domain
from tables_demo_planning.mobipick_components import ArmPose, EnvironmentRepresentation, Item, Location, Robot
from tables_demo_planning.subplan_visualization import SubPlanVisualization


E = TypeVar('E', bound='TablesDemoEnv')


class TablesDemoRobot(Robot, ABC, Generic[E]):
    def __init__(self, env: E) -> None:
        Robot.__init__(self)
        self.env = env

    def pick_item(self, pose: Pose, location: Location, item: Item) -> bool:
        """At pose, look for item at location, pick it up, then move arm to transport pose."""
        print(f"Successfully picked up {item.name}.")
        self.item = item
        self.env.believed_item_locations[item] = Location.on_robot
        self.arm_pose = ArmPose.transport
        return True

    def place_item(self, pose: Pose, location: Location, item: Item) -> bool:
        """At pose, place item at location, then move arm to home pose."""
        print(f"Successfully placed {item.name}.")
        self.item = Item.NOTHING
        self.env.believed_item_locations[item] = location
        self.arm_pose = ArmPose.home
        return True

    def store_item(self, pose: Pose, location: Location, item: Item, klt: Item) -> bool:
        """At pose, store item into box at location, then move arm to home pose."""
        print(f"Successfully inserted {item.name} into box {klt.name}.")
        self.item = Item.NOTHING
        self.env.believed_item_locations[item] = Location.in_box
        if klt not in self.env.believe_klt_contains:
            self.env.believe_klt_contains[klt] = set()
        self.env.believe_klt_contains[klt].add(item)
        self.arm_pose = ArmPose.home
        return True

    def hand_over_item(self, pose: Pose, item: Item) -> bool:
        """At pose, hand over item held to a person."""
        print(f"Successfully handed {item.name} over.")
        self.item = Item.NOTHING
        self.env.believed_item_locations[item] = Location.anywhere
        self.arm_pose = ArmPose.handover
        self.env.offered_items.add(item)
        return True

    def search_at(self, pose: Pose, location: Location) -> bool:
        """At pose, search for item_search at location."""
        item_locations = self.perceive(location)
        if self.env.item_search is not None:
            self.env.search_location = location
            item = self.env.item_search
            assert item
            if item in item_locations.keys():
                print(f"Search for {item.name} SUCCESSFUL.")
                self.env.item_search = None
        return True

    def check_reset_search(self) -> None:
        """Reset search if all tables have been searched."""
        if all(table in self.env.searched_locations for table in TablesDemoDomain.TABLE_LOCATIONS):
            self.env.searched_locations.clear()

    def search_tool(self, item: Item) -> bool:
        """Initiate search for item."""
        self.env.item_search = item
        self.check_reset_search()
        return True

    def search_box(self) -> bool:
        """Initiate search for box."""  # Note: Only implemented for klt_1.
        self.env.item_search = Item.get("klt_1")
        self.check_reset_search()
        return True

    def conclude_tool_search(self, item: Item) -> bool:
        """Conclude tool search as failed. Success is determined in search_at()."""
        print(f"Search for {item.name} FAILED!")
        return False

    def conclude_box_search(self) -> bool:
        """Conclude box search as failed. Success is determined in search_at()."""
        print("Search for box FAILED!")
        return False

    @abstractmethod
    def perceive(self, location: Location) -> Dict[Item, Location]:
        """Move arm into observation pose and return all perceived items with their locations."""


R = TypeVar('R', bound=TablesDemoRobot)


class TablesDemoEnv(EnvironmentRepresentation[R]):
    def __init__(self, robot: R) -> None:
        super().__init__(robot)
        self.believed_item_locations: Dict[Item, Location] = {}
        self.believe_klt_contains: Dict[Item, Set[Item]] = {}
        self.newly_perceived_item_locations: Dict[Item, Location] = {}
        self.item_search: Optional[Item] = None
        self.searched_locations: Set[Location] = set()
        self.search_location = Location.anywhere
        self.offered_items: Set[Item] = set()

    def get_is_in_klt(self, item: Item, klt: Item) -> bool:
        """Return fluent value wheter the item is in the klt."""
        if not klt.name.startswith("klt"):
            return False
        return item in self.believe_klt_contains.get(klt, [])

    def get_item_offered(self, item: Item) -> bool:
        return item in self.offered_items

    def get_believe_item_at(self, item: Item, location: Location) -> bool:
        """Return fluent value whether item is believed to be at location."""
        return location == self.believed_item_locations.get(item, Location.anywhere)

    def get_searched_at(self, location: Location) -> bool:
        """Return fluent value whether robot has already searched at location."""
        return location in self.searched_locations

    def resolve_search_location(self, location: Location) -> Location:
        """Resolve a location symbol to the actual table where the search succeeded."""
        if location not in (Location.tool_search_location, Location.box_search_location):
            return location

        assert self.search_location in TablesDemoDomain.TABLE_LOCATIONS
        return self.search_location

    def print_believed_item_locations(self) -> None:
        """Print at which locations the items are believed to be."""
        print("The believed item locations are:")
        for item in TablesDemoDomain.DEMO_ITEMS.values():
            print(f"- {item.name}:", self.believed_item_locations.get(item, Location.anywhere).name)


class TablesDemoDomain(Domain[E]):
    DEMO_ITEMS = {item.name: item for item in (Item.get("klt_1"), Item.get("klt_2"), Item.get("multimeter_1"))}
    TABLE_LOCATIONS = (Location.table_1, Location.table_2, Location.table_3)
    RETRIES_BEFORE_ABORTION = 2

    def __init__(self, env: E) -> None:
        super().__init__(env)
        self.pose_locations = {
            self.base_table_1_pose: self.table_1,
            self.base_table_2_pose: self.table_2,
            self.base_table_3_pose: self.table_3,
            self.tool_search_pose: self.tool_search_location,
            self.box_search_pose: self.box_search_location,
        }

        self.believe_item_at = self.create_fluent_from_function(self.env.get_believe_item_at)
        self.searched_at = self.create_fluent_from_function(self.env.get_searched_at)
        self.pose_at = self.create_fluent("get_pose_at", pose=Pose, location=Location)
        self.item_offered = self.create_fluent_from_function(self.env.get_item_offered)
        self.is_in_klt = self.create_fluent_from_function(self.env.get_is_in_klt)
        self.set_fluent_functions((self.get_pose_at,))

        self.pick_item, (_, pose, location, item) = self.create_action_from_function(
            TablesDemoRobot.pick_item, set_callable=False
        )
        self.pick_item.add_precondition(self.robot_at(pose))
        self.pick_item.add_precondition(self.robot_has(self.nothing))
        self.pick_item.add_precondition(self.believe_item_at(item, location))
        self.pick_item.add_precondition(self.pose_at(pose, location))
        self.pick_item.add_precondition(
            Or(
                Equals(location, table) for table in [*self.tables, self.tool_search_location, self.box_search_location]
            ),
        )
        self.pick_item.add_precondition(Not(Equals(item, self.nothing)))
        self.pick_item.add_effect(self.robot_has(self.nothing), False)
        self.pick_item.add_effect(self.robot_has(item), True)
        for arm_pose in self.arm_poses:
            self.pick_item.add_effect(self.robot_arm_at(arm_pose), arm_pose == self.arm_pose_transport)
        self.pick_item.add_effect(self.believe_item_at(item, location), False)
        self.pick_item.add_effect(self.believe_item_at(item, self.on_robot), True)
        self.place_item, (_, pose, location, item) = self.create_action_from_function(
            TablesDemoRobot.place_item, set_callable=False
        )
        self.place_item.add_precondition(self.robot_at(pose))
        self.place_item.add_precondition(self.robot_has(item))
        self.place_item.add_precondition(self.believe_item_at(item, self.on_robot))
        self.place_item.add_precondition(self.pose_at(pose, location))
        self.place_item.add_precondition(Not(Equals(location, self.anywhere)))
        self.place_item.add_effect(self.robot_has(item), False)
        self.place_item.add_effect(self.robot_has(self.nothing), True)
        for arm_pose in self.arm_poses:
            self.place_item.add_effect(self.robot_arm_at(arm_pose), arm_pose == self.arm_pose_home)
        self.place_item.add_effect(self.believe_item_at(item, self.on_robot), False)
        self.place_item.add_effect(self.believe_item_at(item, location), True)
        self.store_item, (_, pose, location, item, klt) = self.create_action_from_function(
            TablesDemoRobot.store_item, set_callable=False
        )
        self.store_item.add_precondition(self.robot_at(pose))
        self.store_item.add_precondition(self.robot_has(item))
        self.store_item.add_precondition(self.believe_item_at(item, self.on_robot))
        self.store_item.add_precondition(self.believe_item_at(klt, location))
        self.store_item.add_precondition(self.pose_at(pose, location))
        self.store_item.add_precondition(Not(Equals(location, self.anywhere)))
        self.store_item.add_effect(self.robot_has(item), False)
        self.store_item.add_effect(self.robot_has(self.nothing), True)
        for arm_pose in self.arm_poses:
            self.store_item.add_effect(self.robot_arm_at(arm_pose), arm_pose == self.arm_pose_home)
        self.store_item.add_effect(self.believe_item_at(item, self.on_robot), False)
        self.store_item.add_effect(self.believe_item_at(item, self.in_box), True)
        self.store_item.add_effect(self.is_in_klt(item, klt), True)
        self.handover_item, (_, pose, item) = self.create_action_from_function(
            TablesDemoRobot.hand_over_item, set_callable=False
        )
        self.handover_item.add_precondition(self.robot_at(pose))
        self.handover_item.add_precondition(self.robot_has(item))
        self.handover_item.add_precondition(self.believe_item_at(item, self.on_robot))
        self.handover_item.add_precondition(Not(self.item_offered(item)))
        self.handover_item.add_effect(self.robot_has(item), False)
        self.handover_item.add_effect(self.robot_has(self.nothing), True)
        for arm_pose in self.arm_poses:
            self.handover_item.add_effect(self.robot_arm_at(arm_pose), arm_pose == self.arm_pose_handover)
        self.handover_item.add_effect(self.believe_item_at(item, self.on_robot), False)
        self.handover_item.add_effect(self.believe_item_at(item, self.anywhere), True)
        self.handover_item.add_effect(self.item_offered(item), True)
        self.search_at, (_, pose, location) = self.create_action_from_function(TablesDemoRobot.search_at)
        self.search_at.add_precondition(self.robot_at(pose))
        self.search_at.add_precondition(
            Or(
                self.robot_arm_at(arm_pose)
                for arm_pose in (self.arm_pose_home, self.arm_pose_observe, self.arm_pose_transport)
            )
        )
        self.search_at.add_precondition(Not(self.searched_at(location)))
        self.search_at.add_precondition(Or(Equals(location, table) for table in self.tables))
        self.search_at.add_precondition(self.pose_at(pose, location))
        self.search_at.add_effect(self.searched_at(location), True)
        self.search_tool, (_, item) = self.create_action_from_function(TablesDemoRobot.search_tool)
        self.search_tool.add_precondition(Not(self.robot_at(self.tool_search_pose)))
        self.search_tool.add_precondition(self.robot_has(self.nothing))
        self.search_tool.add_precondition(self.believe_item_at(item, self.anywhere))
        self.search_tool.add_precondition(
            Or(Equals(item, tool) for tool in (self.multimeter, self.relay, self.screwdriver))
        )
        for pose in self.poses:
            self.search_tool.add_effect(self.robot_at(pose), pose == self.tool_search_pose)
        self.search_tool.add_effect(self.believe_item_at(item, self.anywhere), False)
        self.search_tool.add_effect(self.believe_item_at(item, self.tool_search_location), True)
        self.search_box, (_,) = self.create_action_from_function(TablesDemoRobot.search_box)
        self.search_box.add_precondition(Not(self.robot_at(self.box_search_pose)))
        self.search_box.add_precondition(self.believe_item_at(self.box, self.anywhere))
        for pose in self.poses:
            self.search_box.add_effect(self.robot_at(pose), pose == self.box_search_pose)
        self.search_box.add_effect(self.believe_item_at(self.box, self.anywhere), False)
        self.search_box.add_effect(self.believe_item_at(self.box, self.box_search_location), True)
        self.conclude_tool_search, (_, item) = self.create_action_from_function(TablesDemoRobot.conclude_tool_search)
        self.conclude_tool_search.add_precondition(self.believe_item_at(item, self.anywhere))
        for table in self.tables:
            self.conclude_tool_search.add_precondition(self.searched_at(table))
        self.conclude_tool_search.add_precondition(Not(Equals(item, self.box)))
        self.conclude_tool_search.add_effect(self.believe_item_at(item, self.anywhere), False)
        self.conclude_tool_search.add_effect(self.believe_item_at(item, self.tool_search_location), True)
        self.conclude_box_search, (_,) = self.create_action_from_function(TablesDemoRobot.conclude_box_search)
        self.conclude_box_search.add_precondition(self.believe_item_at(self.box, self.anywhere))
        for table in self.tables:
            self.conclude_box_search.add_precondition(self.searched_at(table))
        self.conclude_box_search.add_effect(self.believe_item_at(self.box, self.anywhere), False)
        self.conclude_box_search.add_effect(self.believe_item_at(self.box, self.box_search_location), True)

        self.problem = self.define_problem(
            fluents=(
                self.robot_at,
                self.robot_arm_at,
                self.robot_has,
                self.believe_item_at,
                self.pose_at,
                self.item_offered,
                self.is_in_klt,
            ),
            actions=(
                self.move_base,
                self.move_base_with_item,
                self.move_arm,
                self.pick_item,
                self.place_item,
                self.store_item,
                self.handover_item,
                self.search_tool,
                self.search_box,
            ),
        )
        self.problem.add_quality_metric(MinimizeSequentialPlanLength())
        self.subproblem = self.define_problem(
            fluents=(
                self.robot_at,
                self.robot_arm_at,
                self.robot_has,
                self.believe_item_at,
                self.searched_at,
                self.pose_at,
            ),
            actions=(
                self.move_base,
                self.move_base_with_item,
                self.move_arm,
                self.search_at,
                self.conclude_tool_search,
                self.conclude_box_search,
            ),
        )
        self.subproblem.add_quality_metric(MinimizeSequentialPlanLength())

        self.method_labels.update(
            {
                self.pick_item: lambda parameters: f"Pick up {parameters[-1]}",
                self.place_item: lambda parameters: f"Place {parameters[-1]} onto table",
                self.store_item: lambda parameters: f"Place {parameters[-1]} into box",
                self.handover_item: lambda parameters: f"Handover {parameters[-1]} to person",
                self.search_at: lambda parameters: f"Search at {parameters[-1]}",
                self.search_tool: lambda parameters: f"Search tables for {parameters[-1]}",
                self.search_box: lambda _: "Search tables for the box",
                self.conclude_tool_search: lambda parameters: f"Conclude search for {parameters[-1]}",
                self.conclude_box_search: lambda _: "Conclude search for the box",
            }
        )

        self.visualization: Optional[SubPlanVisualization] = None
        self.espeak_pub: Optional[rospy.Publisher] = None

    def get_pose_at(self, pose: Object, location: Object) -> bool:
        return location == (self.pose_locations[pose] if pose in self.pose_locations.keys() else self.anywhere)

    def set_goals(self, target_location: Location) -> None:
        """Set the goals for the overall demo."""
        self.problem.clear_goals()
        if any(name.startswith("klt_") for name in self.DEMO_ITEMS.keys()):
            if "multimeter_1" in self.DEMO_ITEMS.keys():
                self.problem.add_goal(self.believe_item_at(self.multimeter, self.in_box))
            if "relay_1" in self.DEMO_ITEMS.keys():
                self.problem.add_goal(self.believe_item_at(self.relay, self.in_box))
            if "screwdriver_1" in self.DEMO_ITEMS.keys():
                self.problem.add_goal(self.believe_item_at(self.screwdriver, self.in_box))
            self.problem.add_goal(self.believe_item_at(self.box, self.objects[target_location.name]))
        else:
            if "multimeter_1" in self.DEMO_ITEMS.keys():
                self.problem.add_goal(self.believe_item_at(self.multimeter, self.objects[target_location.name]))
            if "relay_1" in self.DEMO_ITEMS.keys():
                self.problem.add_goal(self.believe_item_at(self.relay, self.objects[target_location.name]))
            if "screwdriver_1" in self.DEMO_ITEMS.keys():
                self.problem.add_goal(self.believe_item_at(self.screwdriver, self.objects[target_location.name]))

        # Using any klt in goal:
        # Option1: Using OR in goals:
        # this does not work with fast-downward, if optimality-guarantee is set when calling the solver
        # Goal until we have in_box-fluent:
        # multimeter_1 is in box and one klt is on the target location
        # klts = [item for (name, item) in self.DEMO_ITEMS.items() if name.startswith('klt_')]
        # subgoals_klt_on = []
        # for klt_item in klts:
        #     klt_up_obj = self.objects[klt_item.name]
        #     # subgoals_klt_on.append(self.believe_item_at(klt_up_obj, self.objects[target_location.name]))
        #     subgoals_klt_on.append(self.believe_item_at(klt_up_obj, self.anywhere))
        # if klts:
        #     self.problem.add_goal(Or(subgoals_klt_on))

        # Option2: Trying to set the goal using existential quantifier:
        # for some reason this does not work, i.e., it cannot find a plan even for the most simple goal:
        # However, it works with fast-downward if we don't set the optimality_guarantee when calling the solver
        # k = Variable("k", self.get_type(Item))
        # self.problem.add_goal(Exists(self.believe_item_at(k, self.objects[target_location.name]), k))
        # # strangely, it finds a plan if we just use 'klt_1' in the expression, instead of the variable:
        # # self.problem.add_goal(Exists(self.believe_item_at(self.box, self.objects[target_location.name]), k))

    def set_search_goals(self) -> None:
        """Set the goals for the current item_search subproblem."""
        assert self.env.item_search
        self.subproblem.clear_goals()
        self.subproblem.add_goal(
            self.believe_item_at(self.box, self.box_search_location)
            if self.env.item_search.name.startswith("klt_")
            else self.believe_item_at(self.objects[self.env.item_search.name], self.tool_search_location)
        )

    def replan(self) -> Optional[List[ActionInstance]]:
        """Print believed item locations, initialize UP problem, and solve it."""
        self.env.print_believed_item_locations()
        self.set_initial_values(self.problem)
        print(self.problem)
        return self.solve(self.problem)

    def run(self, target_location: Location) -> None:
        """Run the mobipick tables demo."""
        print(
            "Scenario: Mobipick shall bring the box with the multimeter inside to"
            f" {self.objects[target_location.name]}."
        )

        executed_action_names: Set[str] = set()  # Note: For visualization purposes only.
        retries_before_abortion = self.RETRIES_BEFORE_ABORTION
        error_counts: Dict[str, int] = defaultdict(int)
        # Solve overall problem.
        self.set_goals(target_location)
        actions = self.replan()
        if actions is None:
            print("Execution ended because no plan could be found.")
            return

        # Loop action execution as long as there are actions.
        while actions:
            print("> Plan:")
            print('\n'.join(map(str, actions)))
            if self.visualization:
                self.visualization.set_actions(
                    [
                        f"{number + len(executed_action_names)} {self.label(action)}"
                        for number, action in enumerate(actions, start=1)
                    ],
                    preserve_actions=executed_action_names,
                )
            print("> Execution:")
            for action in actions:
                executable_action, parameters = self.get_executable_action(action)
                action_name = f"{len(executed_action_names) + 1} {self.label(action)}"
                print(action)
                # Explicitly do not pick up box from target_table since planning does not handle it yet.
                if action.action.name == "pick_item" and parameters[-1].name.startswith("klt_"):
                    assert isinstance(parameters[-2], Location)
                    location = self.env.resolve_search_location(parameters[-2])
                    if location == target_location:
                        print("Picking up box OBSOLETE.")
                        if self.visualization:
                            self.visualization.cancel(action_name)
                        actions = self.replan()
                        break

                if self.visualization:
                    self.visualization.execute(action_name)
                if self.espeak_pub:
                    self.espeak_pub.publish(self.label(action))

                # Execute action.
                result = executable_action(*parameters)
                executed_action_names.add(action_name)
                if rospy.is_shutdown():
                    return

                # Handle item search as an inner execution loop.
                # Rationale: It has additional stop criteria, and might continue the outer loop.
                if self.env.item_search:
                    try:
                        # Check whether an obsolete item search invalidates the previous plan.
                        if self.env.believed_item_locations.get(self.env.item_search, self.anywhere) != self.anywhere:
                            print(f"Search for {self.env.item_search.name} OBSOLETE.")
                            if self.visualization:
                                self.visualization.cancel(action_name)
                            actions = self.replan()
                            break

                        # Search for item by creating and executing a subplan.
                        self.set_initial_values(self.subproblem)
                        self.set_search_goals()
                        subactions = self.solve(self.subproblem)
                        assert subactions, f"No solution for: {self.subproblem}"
                        print("- Search plan:")
                        print('\n'.join(map(str, subactions)))
                        if self.visualization:
                            self.visualization.set_actions(
                                [
                                    f"{len(executed_action_names)}{chr(number)} {self.label(subaction)}"
                                    for number, subaction in enumerate(subactions, start=97)
                                ],
                                preserve_actions=executed_action_names,
                                predecessor=action_name,
                            )
                        print("- Search execution:")
                        subaction_execution_count = 0
                        for subaction in subactions:
                            executable_subaction, subparameters = self.get_executable_action(subaction)
                            subaction_name = (
                                f"{len(executed_action_names)}{chr(subaction_execution_count + 97)}"
                                f" {self.label(subaction)}"
                            )
                            print(subaction)
                            if self.visualization:
                                self.visualization.execute(subaction_name)
                            if self.espeak_pub:
                                self.espeak_pub.publish(self.label(subaction))
                            # Execute search action.
                            result = executable_subaction(*subparameters)
                            subaction_execution_count += 1
                            if rospy.is_shutdown():
                                return

                            if result is not None:
                                if result:
                                    # Note: True result only means any subaction succeeded.
                                    # Check if the search actually succeeded.
                                    if (
                                        self.env.item_search is None
                                        and len(self.env.newly_perceived_item_locations) <= 1
                                    ):
                                        print("- Continue with plan.")
                                        if self.visualization:
                                            self.visualization.succeed(subaction_name)
                                        break
                                    # Check if the search found another item.
                                    elif self.env.newly_perceived_item_locations:
                                        self.env.newly_perceived_item_locations.clear()
                                        print("- Found another item, search ABORTED.")
                                        if self.visualization:
                                            self.visualization.cancel(subaction_name)
                                        if self.espeak_pub:
                                            self.espeak_pub.publish("Found another item. Make a new plan.")
                                        # Set result to None to trigger replanning.
                                        result = None
                                        break
                                else:
                                    if self.visualization:
                                        self.visualization.fail(subaction_name)
                                    break
                        # Note: The conclude action at the end of any search always fails.
                    finally:
                        # Always end the search at this point.
                        self.env.item_search = None

                if result is not None:
                    if result:
                        if self.visualization:
                            self.visualization.succeed(action_name)
                        retries_before_abortion = self.RETRIES_BEFORE_ABORTION
                    else:
                        if self.visualization:
                            self.visualization.fail(action_name)
                        if self.espeak_pub:
                            self.espeak_pub.publish("Action failed.")
                        error_counts[self.label(action)] += 1
                        # Note: This will also fail if two different failures occur successively.
                        if retries_before_abortion <= 0 or any(count >= 3 for count in error_counts.values()):
                            print("Task could not be completed even after retrying.")
                            if self.visualization:
                                self.visualization.add_node("Mission impossible", "red")
                            if self.espeak_pub:
                                self.espeak_pub.publish("Mission impossible!")
                            return

                        retries_before_abortion -= 1
                        actions = self.replan()
                        break
                else:
                    if self.visualization:
                        self.visualization.cancel(action_name)
                    retries_before_abortion = self.RETRIES_BEFORE_ABORTION
                    actions = self.replan()
                    break
            else:
                break
            if actions is None:
                print("Execution ended because no plan could be found.")
                return

        print("Demo complete.")
        if self.visualization:
            self.visualization.add_node("Demo complete", "green")
        if self.espeak_pub:
            self.espeak_pub.publish("Demo complete.")
