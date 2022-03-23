#!/usr/bin/env python3
from random import randrange
from unified_planning.model import Fluent, InstantaneousAction, Object, Problem
from unified_planning.model.fnode import FNode
from unified_planning.plan import Plan
from unified_planning.shortcuts import And, BoolType, Not, OneshotPlanner, UserType
"""Planning example with replanning with interim goal on failed action"""


# Define object types and objects.
table_count = 4
Location = UserType("Location")
tables = [Object(f"table_{table}", Location) for table in range(1, table_count + 1)]
any_table = Object("any_table", Location)
base = Object("base", Location)
unknown = Object("unknown", Location)
locations = tables + [any_table, base, unknown]

# Define all fluents.
robot_at = Fluent("RobotAt", BoolType(), [Location])
robot_has = Fluent("RobotHas", BoolType())
can_find_at = Fluent("CanFindAt", BoolType(), [Location])
searched = Fluent("Searched", BoolType(), [Location])

# Define all actions.

# Move from location a to location b.
move = InstantaneousAction("Move", a=Location, b=Location)
a, b = move.parameters()
move.add_precondition(robot_at(a))
move.add_effect(robot_at(a), False)
move.add_effect(robot_at(b), True)

pick = InstantaneousAction("Pick", a=Location)
a, = pick.parameters()
pick.add_precondition(robot_at(a))
pick.add_precondition(Not(robot_has()))
pick.add_precondition(can_find_at(a))
pick.add_precondition(Not(can_find_at(unknown)))
pick.add_effect(robot_has(), True)
pick.add_effect(can_find_at(a), False)
pick.add_effect(can_find_at(unknown), True)

search = InstantaneousAction("Search", a=Location)
a, = search.parameters()
search.add_precondition(robot_at(a))
search.add_precondition(Not(robot_has()))
search.add_precondition(can_find_at(unknown))
search.add_precondition(Not(searched(a)))
search.add_effect(searched(a), True)

# Note: find is a helper action which should never get executed but is used by the planner
#  to assume the item is found at any_table after searching all tables.
#  Actually searching the tables will hopefully reveal the true item_location.
find = InstantaneousAction("Find")
find.add_precondition(can_find_at(unknown))
for table in tables:
    find.add_precondition(searched(table))
find.add_effect(can_find_at(unknown), False)
find.add_effect(can_find_at(any_table), True)

# Define environment values.
robot_location = base
assumed_item_location = tables[0]  # table_1
index = randrange(table_count)
item_location = tables[index] if index > 0 else unknown  # random but not at table_1, can be not a table

# Compose problem by adding all fluents, actions, and objects into it.
problem = Problem("mobipick_tables")
problem.add_fluent(robot_at)
problem.add_fluent(robot_has, default_initial_value=False)
problem.add_fluent(can_find_at)
problem.add_fluent(searched)
problem.add_action(move)
problem.add_action(pick)
problem.add_action(search)
problem.add_action(find)
problem.add_objects(locations)

def get_plan(goal: FNode) -> Plan:
    # Set initial state and goals.
    for location in locations:
        problem.set_initial_value(robot_at(location), location == robot_location)
        problem.set_initial_value(can_find_at(location), location == assumed_item_location)
        problem.set_initial_value(searched(location), location == robot_location and location in tables)
    problem.clear_goals()
    problem.add_goal(goal)

    # Get plan.
    with OneshotPlanner(problem_kind=problem.kind()) as planner:
        plan = planner.solve(problem)
        print(f"{planner.name()} returned: {plan}")
    return plan

print(f"Scenario: Robot is at {robot_location}, shall fetch an item and return to base.")
print(f"Hint: True item_location is (randomly chosen) {item_location}, robot assumes it is {assumed_item_location}.")
print("-- ")
final_goal = And(robot_at(base), robot_has())
plan = get_plan(final_goal)

while plan:
    for action in plan.actions():
        if action.action() == pick and action.actual_parameters()[0].object() != item_location:
            print(action, "FAILED")
            assumed_item_location = unknown
            plan = get_plan(can_find_at(any_table))
            break
        elif action.action() == search and action.actual_parameters()[0].object() == item_location:
            print(f"{action} found item at true location")
            assumed_item_location = item_location
            plan = get_plan(final_goal)
            break
        elif action.action() == find:
            print("Planner assumed to find the item location but this did not prove true. No solution found.")
            plan = None
            break
        else:
            if action.action() == move:
                robot_location = action.actual_parameters()[-1].object()
            print(action)
    else:
        break
