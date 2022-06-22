from typing import Dict
from unified_planning.model import FNode, Fluent, InstantaneousAction, Object, Problem
from unified_planning.engines import PlanGenerationResult
from unified_planning.shortcuts import And, BoolType, Equals, Not, OneshotPlanner, Or, UserType


# Define object types.
Location = UserType("Location")
Item = UserType("Item")


class UnifiedPlanning:
    # Define all fluents.
    robot_at = Fluent("RobotAt", Location)
    robot_has = Fluent("RobotHas", Item)
    believe_at = Fluent("BelieveAt", Location, item=Item)
    searched_at = Fluent("SearchedAt", BoolType(), location=Location)
    is_klt = Fluent("IsKlt", Item, location=Location)

    def __init__(self, table_count: int, klt_count: int) -> None:
        # Define all objects.

        self.target_table = Object("target_table", Location)  # Note: target_table is tables[0].
        self.tables = [self.target_table] + [Object(f"table_{table}", Location) for table in range(1, table_count)]
        self.klt_locations = [Object(f"klt_{chr(klt + 65)}_location", Location) for klt in range(klt_count)]
        self.on_robot_location = Object("on_robot_location", Location)
        # Note: The tool and klt search locations must be different symbols because these locations
        #  can differ, which is relevant when placing a tool into a klt. Search locations for the
        #  same item type do not interfere with each other.
        self.searched_tool_location = Object("searched_tool_location", Location)
        self.searched_klt_location = Object("searched_klt_location", Location)
        self.unknown_location = Object("unknown_location", Location)
        self.locations = (
            self.tables
            + self.klt_locations
            + [self.on_robot_location, self.searched_tool_location, self.searched_klt_location, self.unknown_location]
        )

        self.multimeter = Object("multimeter", Item)
        self.relay = Object("relay", Item)
        self.screwdriver = Object("screwdriver", Item)
        self.tools = [self.multimeter, self.relay, self.screwdriver]
        self.klts = [Object(f"klt_{chr(klt + 65)}", Item) for klt in range(klt_count)]
        self.nothing = Object("nothing", Item)
        self.items = self.tools + self.klts + [self.nothing]

        # Define all actions.

        self.move = InstantaneousAction("Move", a=Location, b=Location)
        a, b = self.move.parameters
        self.move.add_precondition(Equals(self.robot_at, a))
        self.move.add_precondition(
            Or(
                Or(Equals(b, table) for table in self.tables),
                Equals(b, self.searched_tool_location),
                Equals(b, self.searched_klt_location),
            )
        )
        self.move.add_effect(self.robot_at, b)

        self.pick = InstantaneousAction("Pick", a=Location, item=Item)
        a, item = self.pick.parameters
        self.pick.add_precondition(Equals(self.robot_at, a))
        self.pick.add_precondition(Equals(self.robot_has, self.nothing))
        self.pick.add_precondition(Equals(self.believe_at(item), a))
        self.pick.add_precondition(Not(Equals(a, self.unknown_location)))
        self.pick.add_precondition(Not(Equals(item, self.nothing)))
        self.pick.add_effect(self.robot_has, item)
        self.pick.add_effect(self.believe_at(item), self.unknown_location)

        self.place = InstantaneousAction("Place", a=Location, item=Item)
        a, item = self.place.parameters
        self.place.add_precondition(Equals(self.robot_at, a))
        self.place.add_precondition(Equals(self.robot_has, item))
        self.place.add_precondition(Or(Equals(item, klt) for klt in self.klts))
        self.place.add_effect(self.robot_has, self.nothing)
        self.place.add_effect(self.believe_at(item), a)

        self.store = InstantaneousAction(
            "Store", a=Location, b=Location, klt_location=Location, klt_item=Item, item=Item
        )
        a, b, klt_location, klt_item, item = self.store.parameters
        self.store.add_precondition(Equals(self.robot_at, a))
        self.store.add_precondition(Equals(self.robot_has, item))
        self.store.add_precondition(Equals(self.believe_at(klt_item), b))
        self.store.add_precondition(Equals(self.is_klt(klt_location), klt_item))
        self.store.add_precondition(
            Or(Or(Equals(b, table) for table in self.tables), Equals(b, self.searched_klt_location))
        )
        self.store.add_precondition(Or(Equals(klt_item, klt) for klt in self.klts))
        self.store.add_precondition(Not(Equals(item, klt_item)))
        self.store.add_precondition(Not(Equals(item, self.nothing)))
        self.store.add_effect(self.robot_at, b)
        self.store.add_effect(self.robot_has, self.nothing)
        self.store.add_effect(self.believe_at(item), klt_location)

        self.search_tool = InstantaneousAction("SearchTool", item=Item)
        (item,) = self.search_tool.parameters
        self.search_tool.add_precondition(Not(Equals(self.robot_at, self.searched_tool_location)))
        self.search_tool.add_precondition(Equals(self.robot_has, self.nothing))
        self.search_tool.add_precondition(Equals(self.believe_at(item), self.unknown_location))
        self.search_tool.add_precondition(Or(Equals(item, tool) for tool in self.tools))
        self.search_tool.add_effect(self.robot_at, self.searched_tool_location)
        self.search_tool.add_effect(self.believe_at(item), self.searched_tool_location)

        self.search_klt = InstantaneousAction("SearchKlt", item=Item)
        (item,) = self.search_klt.parameters
        self.search_klt.add_precondition(Not(Equals(self.robot_at, self.searched_klt_location)))
        self.search_klt.add_precondition(Equals(self.believe_at(item), self.unknown_location))
        self.search_klt.add_precondition(Or(Equals(item, klt) for klt in self.klts))
        self.search_klt.add_effect(self.robot_at, self.searched_klt_location)
        self.search_klt.add_effect(self.believe_at(item), self.searched_klt_location)

        self.search_at = InstantaneousAction("SearchAt", a=Location)
        (a,) = self.search_at.parameters
        self.search_at.add_precondition(Equals(self.robot_at, a))
        self.search_at.add_precondition(Not(self.searched_at(a)))
        self.search_at.add_precondition(Or(Equals(a, table) for table in self.tables))
        self.search_at.add_effect(self.searched_at(a), True)

        self.conclude_klt_search = InstantaneousAction("ConcludeKltSearch", item=Item)
        (item,) = self.conclude_klt_search.parameters
        self.conclude_klt_search.add_precondition(Equals(self.believe_at(item), self.unknown_location))
        self.conclude_klt_search.add_precondition(And(self.searched_at(table) for table in self.tables))
        self.conclude_klt_search.add_precondition(Or(Equals(item, klt) for klt in self.klts))
        self.conclude_klt_search.add_effect(self.believe_at(item), self.searched_klt_location)

        self.conclude_tool_search = InstantaneousAction("ConcludeToolSearch", item=Item)
        (item,) = self.conclude_tool_search.parameters
        self.conclude_tool_search.add_precondition(Equals(self.believe_at(item), self.unknown_location))
        self.conclude_tool_search.add_precondition(And(self.searched_at(table) for table in self.tables))
        self.conclude_tool_search.add_precondition(Or(Equals(item, tool) for tool in self.tools))
        self.conclude_tool_search.add_effect(self.believe_at(item), self.searched_tool_location)

        # Compose base UP problem.
        self.problem = Problem("mobipick_tables")
        for fluent in (self.robot_at, self.robot_has, self.believe_at):
            self.problem.add_fluent(fluent)
        self.problem.add_fluent(self.is_klt, default_initial_value=self.nothing)
        for klt_location, klt in zip(self.klt_locations, self.klts):
            self.problem.set_initial_value(self.is_klt(klt_location), klt)
        for action in (self.move, self.pick, self.place, self.store, self.search_tool, self.search_klt):
            self.problem.add_action(action)
        self.problem.add_objects(self.locations)
        self.problem.add_objects(self.items)

        # Compose search subproblem.
        self.subproblem = Problem("mobipick_search")
        for fluent in (self.robot_at, self.believe_at, self.searched_at):
            self.subproblem.add_fluent(fluent)
        for action in (self.move, self.search_at, self.conclude_klt_search, self.conclude_tool_search):
            self.subproblem.add_action(action)
        self.subproblem.add_objects(self.locations)
        self.subproblem.add_objects(self.items)

    def plan(
        self, robot_location: Object, robot_item: Object, item_locations: Dict[Object, Object], goal: FNode
    ) -> PlanGenerationResult:
        """Generate UP plan to achieve goal from given initial fluent values."""
        self.problem.set_initial_value(self.robot_at, robot_location)
        self.problem.set_initial_value(self.robot_has, robot_item)
        for item in self.items:
            self.problem.set_initial_value(self.believe_at(item), item_locations.get(item, self.unknown_location))
        self.problem.clear_goals()
        self.problem.add_goal(goal)
        with OneshotPlanner(problem_kind=self.problem.kind) as planner:
            return planner.solve(self.problem)

    def plan_search(
        self, robot_location: Object, item_locations: Dict[Object, Object], goal: FNode
    ) -> PlanGenerationResult:
        self.subproblem.set_initial_value(self.robot_at, robot_location)
        for item in self.items:
            self.subproblem.set_initial_value(self.believe_at(item), item_locations.get(item, self.unknown_location))
        for location in self.locations:
            self.subproblem.set_initial_value(self.searched_at(location), location == robot_location)
        self.subproblem.clear_goals()
        self.subproblem.add_goal(goal)
        with OneshotPlanner(problem_kind=self.subproblem.kind) as planner:
            return planner.solve(self.subproblem)
