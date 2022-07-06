#!/usr/bin/env python3
from typing import Dict, Optional, Set
import unified_planning
import rospy
import actionlib
from std_srvs.srv import Empty, SetBool
from geometry_msgs.msg import Pose
from pbr_msgs.msg import PickObjectAction, PickObjectGoal, PlaceObjectAction, PlaceObjectGoal
from unified_planning.model.problem import Problem
from unified_planning.shortcuts import And, Equals, Not, Or
from symbolic_fact_generation import on_fact_generator
from tables_demo_planning.demo_domain import ArmPose, Domain, Item, Location, Robot
from tables_demo_planning.subplan_visualization import SubPlanVisualization

"""
Main execution node of the tables demo.
Development in progress.
"""


class TablesDemoRobot(Robot):
    def __init__(self, domain: 'TablesDemo') -> None:
        super().__init__("mobipick")
        self.domain = domain
        self.activate_pose_selector = rospy.ServiceProxy("/pick_pose_selector_node/pose_selector_activate", SetBool)
        self.open_gripper = rospy.ServiceProxy("/mobipick/pose_teacher/open_gripper", Empty)
        self.pick_object_action_client = actionlib.SimpleActionClient("/mobipick/pick_object", PickObjectAction)
        self.pick_object_goal = PickObjectGoal()
        self.place_object_action_client = actionlib.SimpleActionClient("/mobipick/place_object", PlaceObjectAction)
        self.place_object_goal = PlaceObjectGoal()

    def pick_item(self, pose: Pose, location: Location, item: Item) -> bool:
        """At pose, look for item at location, pick it up, then move arm to transport pose."""
        rospy.loginfo("Waiting for pick object action server.")
        if not self.pick_object_action_client.wait_for_server(timeout=rospy.Duration(2.0)):
            rospy.logerr("Pick Object action server not available!")
            return False

        rospy.loginfo("Found pick object action server.")
        location = self.domain.resolve_search_location(location)
        perceived_item_locations = self.perceive(location)
        if item not in perceived_item_locations.keys():
            rospy.logwarn(f"Cannot find {item.name} at {location.name}. Pick up FAILED!")
            return False

        rospy.loginfo(f"Sending pick '{item.name}' goal to pick object action server.")
        self.pick_object_goal.object_name = item.name
        self.pick_object_action_client.send_goal(self.pick_object_goal)
        rospy.loginfo("Wait for result from pick object action server.")
        if not self.pick_object_action_client.wait_for_result(timeout=rospy.Duration(50.0)):
            rospy.logwarn(f"Pick up {item.name} at {location.name} FAILED due to timeout!")
            return False

        result = self.pick_object_action_client.get_result()
        rospy.loginfo(f"The pick object server is done with execution, resuĺt was: '{result}'")
        if not result or not result.success:
            rospy.logwarn(f"Pick up {item.name} at {location.name} FAILED!")
            return False

        print(f"Successfully picked up {item.name}.")
        self.item = item
        self.domain.believed_item_locations[item] = Location.on_robot
        self.arm.move("transport")
        return True

    def place_box(self, pose: Pose, location: Location) -> bool:
        """At pose, place box down at location."""
        rospy.loginfo("Waiting for place object action server.")
        if not self.place_object_action_client.wait_for_server(timeout=rospy.Duration(10.0)):
            rospy.logerr("Place Object action server not available!")
            return False

        rospy.loginfo("Found place object action server.")
        self.perceive(location)
        rospy.loginfo("Sending place box goal to place object action server.")
        self.place_object_goal.support_surface_name = location.name
        self.place_object_goal.observe_before_place = False
        self.place_object_action_client.send_goal(self.place_object_goal)
        rospy.loginfo("Wait for result from place object action server.")
        if not self.place_object_action_client.wait_for_result(timeout=rospy.Duration(50.0)):
            rospy.logwarn(f"Place box at {location.name} FAILED due to timeout!")
            return False

        result = self.place_object_action_client.get_result()
        rospy.loginfo(f"The place object server is done with execution, result was: '{result}'")
        if not result or not result.success:
            rospy.logwarn(f"Place box at {location.name} FAILED!")
            return False

        print(f"Successfully placed box.")
        self.item = Item.nothing
        self.domain.believed_item_locations[Item.box] = location
        self.arm.move("home")
        return True

    def store_item(self, pose: Pose, location: Location, item: Item) -> bool:
        """At pose, store item into box at location."""
        self.arm.move("above")
        self.open_gripper()
        self.item = Item.nothing
        self.domain.believed_item_locations[item] = Location.in_box
        self.arm.move("home")
        return True

    def search_at(self, pose: Pose, location: Location) -> bool:
        """At pose, search for item_search at location."""
        self.domain.search_location = location
        item_locations = self.perceive(location)
        item = self.domain.item_search
        assert item
        if item in item_locations.keys():
            print(f"Search for {item.name} SUCCESSFUL.")
            self.domain.item_search = None
        return True

    def search_tool(self, item: Item) -> bool:
        """Initiate search for item."""
        self.domain.item_search = item
        return True

    def search_box(self) -> bool:
        """Initiate search for box."""
        self.domain.item_search = Item.box
        return True

    def conclude_tool_search(self, item: Item) -> bool:
        """Conclude tool search as failed. Success is determined in search_at()."""
        print(f"Search for {item.name} FAILED!")
        return False

    def conclude_box_search(self) -> bool:
        """Conclude box search as failed. Success is determined in search_at()."""
        print("Search for box FAILED!")
        return False

    def perceive(self, location: Location) -> Dict[Item, Location]:
        """Move arm into observation pose and return all perceived items with their locations."""
        self.arm.move("observe100cm_right")
        self.arm_pose = ArmPose.observe
        rospy.loginfo("Wait for pose selector service ...")
        rospy.wait_for_service("/pick_pose_selector_node/pose_selector_activate", timeout=rospy.Duration(2.0))
        rospy.loginfo(f"Clear facts for {location.name}.")
        on_fact_generator.clear_facts_and_poses_for_table(location.name)
        self.activate_pose_selector(True)
        rospy.sleep(5)
        rospy.loginfo("Get facts from fact generator.")
        facts = on_fact_generator.get_current_facts()
        self.activate_pose_selector(False)
        perceived_item_locations: Dict[Item, Location] = {}
        for fact in facts:
            if fact.name == "on":
                item_name, table_name = fact.values
                rospy.loginfo(f"{item_name} on {table_name} returned by pose_selector and fact_generator.")
                if item_name in [item.value for item in self.domain.DEMO_ITEMS] and table_name == location.name:
                    rospy.loginfo(f"{item_name} is perceived as on {table_name}.")
                    perceived_item_locations[Item(item_name)] = Location(table_name)
        # Determine newly perceived items and their locations.
        self.domain.newly_perceived_item_locations.clear()
        for perceived_item, perceived_location in perceived_item_locations.items():
            if (
                perceived_item not in self.domain.believed_item_locations.keys()
                or self.domain.believed_item_locations[perceived_item] != location
            ):
                self.domain.newly_perceived_item_locations[perceived_item] = perceived_location
        rospy.loginfo(f"Newly perceived items: {self.domain.newly_perceived_item_locations.keys()}")
        # Remove all previously perceived items at location.
        for check_item, check_location in list(self.domain.believed_item_locations.items()):
            if check_location == location:
                del self.domain.believed_item_locations[check_item]
        # Add all currently perceived items at location.
        self.domain.believed_item_locations.update(perceived_item_locations)
        self.domain.print_believed_item_locations()
        return perceived_item_locations


class TablesDemo(Domain):
    DEMO_ITEMS = (Item.box, Item.multimeter)

    def __init__(self) -> None:
        super().__init__(TablesDemoRobot(self))
        self.believed_item_locations: Dict[Item, Location] = {}
        self.newly_perceived_item_locations: Dict[Item, Location] = {}
        self.item_search: Optional[Item] = None
        self.search_location = Location.anywhere
        self.target_location = Location.table_2
        self.target_table = self.objects[self.target_location.name]

        self.pick_item, (robot, pose, location, item) = self.create_action(TablesDemoRobot, TablesDemoRobot.pick_item)
        self.pick_item.add_precondition(self.robot_at(robot, pose))
        self.pick_item.add_precondition(
            Or(self.robot_arm_at(robot, arm_pose) for arm_pose in (self.arm_pose_home, self.arm_pose_observe))
        )
        self.pick_item.add_precondition(self.robot_has(robot, self.nothing))
        self.pick_item.add_precondition(self.believe_item_at(item, location))
        self.pick_item.add_precondition(self.pose_at(pose, location))
        self.pick_item.add_precondition(
            Or(
                Equals(location, table) for table in [*self.tables, self.tool_search_location, self.box_search_location]
            ),
        )
        self.pick_item.add_precondition(Not(Equals(item, self.nothing)))
        self.pick_item.add_effect(self.robot_has(robot, self.nothing), False)
        self.pick_item.add_effect(self.robot_has(robot, item), True)
        for arm_pose in self.arm_poses:
            self.pick_item.add_effect(self.robot_arm_at(robot, arm_pose), arm_pose == self.arm_pose_transport)
        self.pick_item.add_effect(self.believe_item_at(item, location), False)
        self.pick_item.add_effect(self.believe_item_at(item, self.on_robot), True)
        self.place_box, (robot, pose, location) = self.create_action(TablesDemoRobot, TablesDemoRobot.place_box)
        self.place_box.add_precondition(self.robot_at(robot, pose))
        self.place_box.add_precondition(self.robot_arm_at(robot, self.arm_pose_transport))
        self.place_box.add_precondition(self.robot_has(robot, self.box))
        self.place_box.add_precondition(self.believe_item_at(self.box, self.on_robot))
        self.place_box.add_precondition(self.pose_at(pose, location))
        self.place_box.add_effect(self.robot_has(robot, self.box), False)
        self.place_box.add_effect(self.robot_has(robot, self.nothing), True)
        self.place_box.add_effect(self.robot_arm_at(robot, self.arm_pose_transport), False)
        self.place_box.add_effect(self.robot_arm_at(robot, self.arm_pose_home), True)
        self.place_box.add_effect(self.believe_item_at(self.box, self.on_robot), False)
        self.place_box.add_effect(self.believe_item_at(self.box, location), True)
        self.store_item, (robot, pose, location, item) = self.create_action(TablesDemoRobot, TablesDemoRobot.store_item)
        self.store_item.add_precondition(self.robot_at(robot, pose))
        self.store_item.add_precondition(self.robot_arm_at(robot, self.arm_pose_transport))
        self.store_item.add_precondition(self.robot_has(robot, item))
        self.store_item.add_precondition(self.believe_item_at(item, self.on_robot))
        self.store_item.add_precondition(self.believe_item_at(self.box, location))
        self.store_item.add_precondition(self.pose_at(pose, location))
        self.store_item.add_precondition(Not(Equals(location, self.anywhere)))
        self.store_item.add_effect(self.robot_has(robot, item), False)
        self.store_item.add_effect(self.robot_has(robot, self.nothing), True)
        self.store_item.add_effect(self.robot_arm_at(robot, self.arm_pose_transport), False)
        self.store_item.add_effect(self.robot_arm_at(robot, self.arm_pose_home), True)
        self.store_item.add_effect(self.believe_item_at(item, self.on_robot), False)
        self.store_item.add_effect(self.believe_item_at(item, self.in_box), True)
        self.search_at, (robot, pose, location) = self.create_action(TablesDemoRobot, TablesDemoRobot.search_at)
        self.search_at.add_precondition(self.robot_at(robot, pose))
        self.search_at.add_precondition(
            Or(
                self.robot_arm_at(robot, arm_pose)
                for arm_pose in (self.arm_pose_home, self.arm_pose_observe, self.arm_pose_transport)
            )
        )
        self.search_at.add_precondition(Not(self.searched_at(location)))
        self.search_at.add_precondition(Or(Equals(location, table) for table in self.tables))
        self.search_at.add_precondition(self.pose_at(pose, location))
        self.search_at.add_effect(self.searched_at(location), True)
        self.search_tool, (robot, item) = self.create_action(TablesDemoRobot, TablesDemoRobot.search_tool)
        self.search_tool.add_precondition(Not(self.robot_at(robot, self.tool_search_pose)))
        self.search_tool.add_precondition(self.robot_arm_at(robot, self.arm_pose_home))
        self.search_tool.add_precondition(self.robot_has(robot, self.nothing))
        self.search_tool.add_precondition(self.believe_item_at(item, self.anywhere))
        self.search_tool.add_precondition(
            Or(Equals(item, tool) for tool in (self.multimeter, self.relay, self.screwdriver))
        )
        for pose in self.poses:
            self.search_tool.add_effect(self.robot_at(robot, pose), pose == self.tool_search_pose)
        self.search_tool.add_effect(self.believe_item_at(item, self.anywhere), False)
        self.search_tool.add_effect(self.believe_item_at(item, self.tool_search_location), True)
        self.search_box, (robot,) = self.create_action(TablesDemoRobot, TablesDemoRobot.search_box)
        self.search_box.add_precondition(Not(self.robot_at(robot, self.box_search_pose)))
        self.search_box.add_precondition(
            Or(self.robot_arm_at(robot, arm_pose) for arm_pose in (self.arm_pose_home, self.arm_pose_transport))
        )
        self.search_box.add_precondition(self.believe_item_at(self.box, self.anywhere))
        for pose in self.poses:
            self.search_box.add_effect(self.robot_at(robot, pose), pose == self.box_search_pose)
        self.search_box.add_effect(self.believe_item_at(self.box, self.anywhere), False)
        self.search_box.add_effect(self.believe_item_at(self.box, self.box_search_location), True)
        self.conclude_tool_search, (robot, item) = self.create_action(
            TablesDemoRobot, TablesDemoRobot.conclude_tool_search
        )
        self.conclude_tool_search.add_precondition(self.believe_item_at(item, self.anywhere))
        self.conclude_tool_search.add_precondition(And(self.searched_at(table) for table in self.tables))
        self.conclude_tool_search.add_precondition(Not(Equals(item, self.box)))
        self.conclude_tool_search.add_effect(self.believe_item_at(item, self.anywhere), False)
        self.conclude_tool_search.add_effect(self.believe_item_at(item, self.tool_search_location), True)
        self.conclude_box_search, (robot,) = self.create_action(TablesDemoRobot, TablesDemoRobot.conclude_box_search)
        self.conclude_box_search.add_precondition(self.believe_item_at(self.box, self.anywhere))
        self.conclude_box_search.add_precondition(And(self.searched_at(table) for table in self.tables))
        self.conclude_box_search.add_effect(self.believe_item_at(self.box, self.anywhere), False)
        self.conclude_box_search.add_effect(self.believe_item_at(self.box, self.box_search_location), True)

        self.problem = self.define_problem(
            fluents=(self.robot_at, self.robot_arm_at, self.robot_has, self.believe_item_at, self.pose_at),
            actions=(
                self.move_base,
                self.move_base_with_item,
                self.move_arm,
                self.pick_item,
                self.place_box,
                self.store_item,
                self.search_tool,
                self.search_box,
            ),
        )
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

        self.method_labels.update(
            {
                self.pick_item: lambda parameters: f"Pick up {parameters[-1]}",
                self.place_box: lambda _: "Place box on table",
                self.store_item: lambda parameters: f"Place {parameters[-1]} into box",
                self.search_at: lambda parameters: f"Search at {parameters[-1]}",
                self.search_tool: lambda parameters: f"Search tables for {parameters[-1]}",
                self.search_box: lambda _: "Search tables for the box",
                self.conclude_tool_search: lambda parameters: f"Conclude search for {parameters[-1]}",
                self.conclude_box_search: lambda _: "Conclude search for the box",
            }
        )

    def set_initial_values(self, problem: Problem) -> None:
        """Set initial values for UP problem based on the fluents used and the current state."""
        super().set_initial_values(problem)
        if self.believe_item_at in problem.fluents:
            for item in Item:
                for location in Location:
                    problem.set_initial_value(
                        self.believe_item_at(
                            self.objects[item.name],
                            self.objects[location.name],
                        ),
                        location == self.believed_item_locations.get(item, Location.anywhere),
                    )

    def set_goals(self) -> None:
        """Set the goals for the overall demo."""
        self.problem.clear_goals()
        if Item.box in self.DEMO_ITEMS:
            if Item.multimeter in self.DEMO_ITEMS:
                self.problem.add_goal(self.believe_item_at(self.multimeter, self.in_box))
            if Item.relay in self.DEMO_ITEMS:
                self.problem.add_goal(self.believe_item_at(self.relay, self.in_box))
            if Item.screwdriver in self.DEMO_ITEMS:
                self.problem.add_goal(self.believe_item_at(self.screwdriver, self.in_box))
            self.problem.add_goal(self.believe_item_at(self.box, self.target_table))
        else:
            if Item.multimeter in self.DEMO_ITEMS:
                self.problem.add_goal(self.believe_item_at(self.multimeter, self.target_table))
            if Item.relay in self.DEMO_ITEMS:
                self.problem.add_goal(self.believe_item_at(self.relay, self.target_table))
            if Item.screwdriver in self.DEMO_ITEMS:
                self.problem.add_goal(self.believe_item_at(self.screwdriver, self.target_table))

    def set_search_goals(self) -> None:
        """Set the goals for the item_search subproblem."""
        assert self.item_search
        self.subproblem.clear_goals()
        self.subproblem.add_goal(
            self.believe_item_at(self.box, self.box_search_location)
            if self.item_search == Item.box
            else self.believe_item_at(self.objects[self.item_search.name], self.tool_search_location)
        )

    def resolve_search_location(self, location: Location) -> Location:
        """Resolve a location symbol to the actual table where the search succeeded."""
        if location not in (Location.tool_search_location, Location.box_search_location):
            return location

        assert self.search_location in (Location.table_1, Location.table_2, Location.table_3)
        return self.search_location

    def print_believed_item_locations(self) -> None:
        """Print at which locations the items are believed to be."""
        print("The believed item locations are:")
        for item in self.DEMO_ITEMS:
            print(f"- {item.name}:", self.believed_item_locations.get(item, Location.anywhere).name)

    def run(self) -> None:
        print(f"Scenario: Mobipick shall bring all items inside the box to {self.target_table}.")

        visualization = SubPlanVisualization()
        executed_actions: Set[str] = set()
        retry_on_failure = True
        # Solve overall problem.
        self.print_believed_item_locations()
        self.set_initial_values(self.problem)
        self.set_goals()
        actions = self.solve(self.problem)
        if not actions:
            print("Execution ended because no plan could be found.")
        # Loop action execution as long as there are actions.
        while actions:
            print("> Plan:")
            up_actions = [up_action for up_action, _ in actions]
            print('\n'.join(map(str, up_actions)))
            visualization.set_actions(
                [
                    f"{len(executed_actions) + index + 1} {self.label(up_action)}"
                    for index, up_action in enumerate(up_actions)
                ],
                preserve_actions=executed_actions,
            )
            print("> Execution:")
            for up_action, (method, parameters) in actions:
                action_name = f"{len(executed_actions) + 1} {self.label(up_action)}"
                print(up_action)
                # Explicitly do not pick up box from target_table since planning does not handle it yet.
                if method == TablesDemoRobot.pick_item and parameters[-1] == Item.box:
                    location = self.resolve_search_location(parameters[-2])
                    if location == self.target_location:
                        print(f"Picking up box OBSOLETE.")
                        visualization.fail(action_name)
                        self.print_believed_item_locations()
                        self.set_initial_values(self.problem)
                        actions = self.solve(self.problem)
                        break

                visualization.execute(action_name)
                # Execute action.
                result = method(*parameters)
                executed_actions.add(action_name)
                if rospy.is_shutdown():
                    return

                # Handle item search as an inner execution loop.
                # Rationale: It has additional stop criteria, and might continue the outer loop.
                if self.item_search:
                    try:
                        # Check whether an obsolete item search invalidates the previous plan.
                        if self.believed_item_locations.get(self.item_search, self.anywhere) != self.anywhere:
                            print(f"Search for {self.item_search.name} OBSOLETE.")
                            visualization.fail(action_name)
                            self.print_believed_item_locations()
                            self.set_initial_values(self.problem)
                            actions = self.solve(self.problem)
                            break

                        # Search for item by creating and executing a subplan.
                        self.set_initial_values(self.subproblem)
                        self.set_search_goals()
                        subactions = self.solve(self.subproblem)
                        assert subactions, f"No solution for: {self.subproblem}"
                        print("- Search plan:")
                        up_subactions = [up_subaction for up_subaction, _ in subactions]
                        print('\n'.join(map(str, up_subactions)))
                        visualization.set_actions(
                            [
                                f"{len(executed_actions)}{chr(index + 97)} {self.label(up_subaction)}"
                                for index, up_subaction in enumerate(up_subactions)
                            ],
                            preserve_actions=executed_actions,
                            predecessor=action_name,
                        )
                        print("- Search execution:")
                        subaction_execution_count = 0
                        for up_subaction, (submethod, subparameters) in subactions:
                            subaction_name = f"{len(executed_actions)}{chr(subaction_execution_count + 97)} {self.label(up_subaction)}"
                            print(up_subaction)
                            visualization.execute(subaction_name)
                            # Execute search action.
                            result = submethod(*subparameters)
                            subaction_execution_count += 1
                            if rospy.is_shutdown():
                                return

                            if result is not None:
                                if result:
                                    # Note: True result only means any subaction succeeded.
                                    # Check if the search actually succeeded.
                                    if self.item_search is None:
                                        print("- Continue with plan.")
                                        visualization.succeed(subaction_name)
                                        break
                                    # Check if the search found another item.
                                    elif self.newly_perceived_item_locations:
                                        print("- Found another item, search ABORTED.")
                                        visualization.fail(subaction_name)
                                        # Set result to False to trigger replanning.
                                        result = False
                                        break
                                else:
                                    visualization.fail(subaction_name)
                                    break
                        # Note: The conclude action at the end of any search always fails.
                    finally:
                        # Always end the search at this point.
                        self.item_search = None

                if result is not None:
                    if result:
                        visualization.succeed(action_name)
                        retry_on_failure = True
                    else:
                        visualization.fail(action_name)
                        # Note: This will also fail if two different failures occur successively.
                        if not retry_on_failure:
                            print("Task could not be completed even after retrying.")
                            return

                        retry_on_failure = False
                        self.print_believed_item_locations()
                        self.set_initial_values(self.problem)
                        actions = self.solve(self.problem)
                        break
                else:
                    rospy.logwarn(f"Executing '{up_action}' did not return a result.")
            else:
                print("Task complete.")
                break


if __name__ == '__main__':
    unified_planning.shortcuts.get_env().credits_stream = None
    try:
        TablesDemo().run()
    except rospy.ROSInterruptException:
        pass
