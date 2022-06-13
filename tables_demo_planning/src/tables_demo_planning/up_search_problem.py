import time
import rospy
from random import randrange
from unified_planning.model import FNode, Fluent, InstantaneousAction, Object, Problem
from unified_planning.plans.plan import Plan
from unified_planning.shortcuts import And, BoolType, Equals, Not, OneshotPlanner, UserType
from tables_demo_planning.plan_visualization import PlanVisualization

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
robot_at = Fluent("RobotAt", Location)
robot_has = Fluent("RobotHas", BoolType())
can_find_at = Fluent("CanFindAt", Location)
searched = Fluent("Searched", BoolType(), location=Location)

# Define all actions.

# Move from location a to location b.
move = InstantaneousAction("Move", a=Location, b=Location)
a, b = move.parameters
move.add_precondition(Equals(robot_at, a))
move.add_effect(robot_at, b)

pick = InstantaneousAction("Pick", a=Location)
(a,) = pick.parameters
pick.add_precondition(Equals(robot_at, a))
pick.add_precondition(Not(robot_has))
pick.add_precondition(Equals(can_find_at, a))
pick.add_precondition(Not(Equals(a, unknown)))
pick.add_effect(robot_has, True)
pick.add_effect(can_find_at, unknown)

search = InstantaneousAction("Search", a=Location)
(a,) = search.parameters
search.add_precondition(Equals(robot_at, a))
search.add_precondition(Not(robot_has))
search.add_precondition(Equals(can_find_at, unknown))
search.add_precondition(Not(searched(a)))
search.add_effect(searched(a), True)

# Note: conclude_search is a helper action which should never get executed but is used by the planner
#  to assume the item is found at any_table after searching all tables. Actually searching the tables
#  will hopefully reveal the true item_location. Otherwise, plan execution fails.
conclude_search = InstantaneousAction("ConcludeSearch")
conclude_search.add_precondition(Equals(can_find_at, unknown))
for table in tables:
    conclude_search.add_precondition(searched(table))
conclude_search.add_effect(can_find_at, any_table)

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
problem.add_action(conclude_search)
problem.add_objects(locations)


def get_plan(goal: FNode) -> Plan:
    # Set initial state and goals.
    problem.set_initial_value(robot_at, robot_location)
    problem.set_initial_value(can_find_at, assumed_item_location)
    for location in locations:
        problem.set_initial_value(searched(location), location == robot_location and location in tables)
    problem.clear_goals()
    problem.add_goal(goal)

    # Get plan.
    with OneshotPlanner(problem_kind=problem.kind) as planner:
        result = planner.solve(problem)
        print(f"{result.engine_name} returned: {result.plan}")
    return result.plan


rospy.init_node("up_search_problem")
print(f"Scenario: Robot is at {robot_location}, shall fetch an item and return to base.")
print(f"Hint: True item_location is (randomly chosen) {item_location}, robot assumes it is {assumed_item_location}.")
print("-- ")
final_goal = And(Equals(robot_at, base), robot_has)
plan = get_plan(final_goal)
visualization = PlanVisualization(plan.actions)

while plan:
    for action in plan.actions:
        visualization.execute(action)
        time.sleep(3.0)
        if action.action == pick and action.actual_parameters[0].object() != item_location:
            print(action, "FAILED")
            visualization.fail(action)
            assumed_item_location = unknown
            print("Replan ...")
            time.sleep(5.0)
            plan = get_plan(Equals(can_find_at, any_table))
            visualization.set_actions(plan.actions)
            break
        elif action.action == search and action.actual_parameters[0].object() == item_location:
            print(f"{action} found item at true location")
            visualization.succeed(action)
            assumed_item_location = item_location
            print("Replan ...")
            time.sleep(5.0)
            plan = get_plan(final_goal)
            visualization.set_actions(plan.actions)
            break
        elif action.action == conclude_search:
            print("Planner assumed to find the item location but this did not prove true. No solution found.")
            visualization.fail(action)
            plan = None
            break
        else:
            if action.action == move:
                robot_location = action.actual_parameters[-1].object()
            print(action)
            visualization.succeed(action)
    else:
        break
