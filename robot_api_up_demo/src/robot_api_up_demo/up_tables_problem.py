import time
from unified_planning.model import FNode, Fluent, InstantaneousAction, Object, Problem
from unified_planning.plan import Plan
from unified_planning.shortcuts import And, Equals, Not, OneshotPlanner, Or, UserType
from robot_api_up_demo.plan_visualization import PlanVisualization

"""Tables demo purely on variables and UP"""


# Define object types and objects.

table_count = 6
Location = UserType("Location")
target_table = Object("target_table", Location)  # Note: target_table is tables[0].
tables = [target_table] + [Object(f"table_{table}", Location) for table in range(1, table_count)]
on_robot_location = Object("on_robot_location", Location)
searched_location = Object("searched_location", Location)
unknown_location = Object("unknown_location", Location)
locations = tables + [on_robot_location, searched_location, unknown_location]

Item = UserType("Item")
power_drill = Object("power_drill", Item)
remote_control = Object("remote_control", Item)
screwdriver = Object("screwdriver", Item)
nothing = Object("nothing", Item)
tools = [power_drill, remote_control, screwdriver]
items = tools + [nothing]

# Define all fluents.
robot_at = Fluent("RobotAt", Location)
robot_has = Fluent("RobotHas", Item)
believe_at = Fluent("BelieveAt", Location, item=Item)

# Define all actions.

move = InstantaneousAction("Move", a=Location, b=Location, item=Item)
a, b, item = move.parameters
move.add_precondition(Equals(robot_at, a))
move.add_precondition(Equals(robot_has, item))
move.add_precondition(Not(Equals(b, on_robot_location)))
move.add_precondition(Not(Equals(b, unknown_location)))
move.add_effect(robot_at, b)

pick = InstantaneousAction("Pick", a=Location, item=Item)
a, item = pick.parameters
pick.add_precondition(Equals(robot_at, a))
pick.add_precondition(Equals(robot_has, nothing))
pick.add_precondition(Equals(believe_at(item), a))
pick.add_precondition(Not(Equals(a, unknown_location)))
pick.add_precondition(Not(Equals(item, nothing)))
pick.add_effect(robot_has, item)
pick.add_effect(believe_at(item), unknown_location)

place = InstantaneousAction("Place", a=Location, item=Item)
a, item = place.parameters
place.add_precondition(Equals(robot_at, a))
place.add_precondition(Equals(robot_has, item))
place.add_precondition(Not(Equals(item, nothing)))
place.add_effect(robot_has, nothing)
place.add_effect(believe_at(item), a)

search = InstantaneousAction("Search", item=Item)
(item,) = search.parameters
search.add_precondition(Not(Equals(robot_at, searched_location)))
search.add_precondition(Equals(robot_has, nothing))
search.add_precondition(Equals(believe_at(item), unknown_location))
search.add_precondition(
    And(Or(Equals(believe_at(tool), target_table), Equals(believe_at(tool), unknown_location)) for tool in tools)
)
search.add_effect(robot_at, searched_location)
search.add_effect(believe_at(item), searched_location)

# Define environment values.
actual_robot_location = unknown_location
actual_robot_item = nothing
believed_item_positions = {screwdriver: tables[4]}
actual_item_positions = {power_drill: tables[1], remote_control: tables[2], screwdriver: tables[2]}
final_goal = And(Equals(believe_at(tool), target_table) for tool in tools)
sleep_step = 1.0
sleep_replan = 2.0

# Compoese base UP problem.
problem = Problem("mobipick_tables")
for fluent in (robot_at, robot_has, believe_at):
    problem.add_fluent(fluent)
for action in (move, pick, place, search):
    problem.add_action(action)
problem.add_objects(locations)
problem.add_objects(items)


def get_plan(goal: FNode) -> Plan:
    """Get UP plan to achieve goal."""
    problem.set_initial_value(robot_at, actual_robot_location)
    problem.set_initial_value(robot_has, actual_robot_item)
    for item in items:
        problem.set_initial_value(believe_at(item), believed_item_positions.get(item, unknown_location))
    problem.clear_goals()
    problem.add_goal(goal)

    with OneshotPlanner(problem_kind=problem.kind) as planner:
        result = planner.solve(problem)
        print(f"{result.planner_name} returned: {result.plan}")
    return result.plan


def replan(goal: FNode, visualization: PlanVisualization) -> Plan:
    """Replan to achieve goal and update visualization."""
    print("Replan ...")
    time.sleep(sleep_replan)
    plan = get_plan(goal)
    visualization.set_actions(plan.actions)
    return plan


if __name__ == '__main__':
    print(f"Scenario: Robot shall bring all items to {target_table}. The item locations are:")
    for tool in tools:
        print(
            f"- {tool}, believe: {believed_item_positions.get(tool, unknown_location)}, "
            f"actual: {actual_item_positions.get(tool, unknown_location)} "
        )

    plan = get_plan(final_goal)
    visualization = PlanVisualization(plan.actions)
    while plan:
        for action in plan.actions:
            # Abort plan execution on closing the visualization window.
            if not visualization.window.props.visible:
                visualization.thread.join()
                plan = None
                break

            visualization.execute(action)
            parameters = action.actual_parameters
            if action.action == move:
                item = parameters[2].object()
                print(f"Move from {actual_robot_location} to {parameters[1]} with {item}.")
                actual_robot_location = parameters[1].object()
            elif action.action == pick:
                item = parameters[1].object()
                if actual_item_positions.get(item) == actual_robot_location:
                    print(f"Pick up {item} at {actual_robot_location}.")
                    believed_item_positions[item] = on_robot_location
                    actual_item_positions[item] = on_robot_location
                else:
                    print(f"Pick up {item} at {actual_robot_location} FAILED.")
                    believed_item_positions[item] = unknown_location
                    visualization.fail(action)
                    plan = replan(final_goal, visualization)
                    break
            elif action.action == place:
                item = parameters[1].object()
                print(f"Place {item} at {actual_robot_location}.")
                believed_item_positions[item] = actual_item_positions[item] = actual_robot_location
            elif action.action == search:
                item = parameters[0].object()
                if believed_item_positions.get(item, unknown_location) != unknown_location:
                    print(f"Search for {item} OBSOLETE.")
                    plan = replan(final_goal, visualization)
                    break

                # Search for item by circling and observing the tables.
                print(f"Search for {item}.")
                index = tables.index(actual_robot_location) if actual_robot_location in tables else 0
                indices = [(index + offset - 1) % (table_count - 1) + 1 for offset in range(1, table_count)]
                has_detected_current = False
                has_detected_new = False
                for index in indices:
                    time.sleep(sleep_step)
                    print(f"- Move from {actual_robot_location} to {tables[index]}.")
                    actual_robot_location = tables[index]
                    time.sleep(sleep_step)
                    print(f"- Observe {actual_robot_location}.")
                    time.sleep(sleep_step)
                    # Detect all tools on the current table.
                    for tool in tools:
                        if actual_item_positions.get(tool) == actual_robot_location:
                            print(f"- Detect {tool}.")
                            if believed_item_positions.get(tool) != actual_robot_location:
                                believed_item_positions[tool] = actual_robot_location
                                if tool == item:
                                    has_detected_current = True
                                else:
                                    has_detected_new = True
                    # If something is detected, stop searching.
                    if has_detected_current or has_detected_new:
                        break
                else:
                    time.sleep(sleep_step)
                    print(f"Search failed. No solution found.")
                    visualization.fail(action)
                    plan = None
                    break

                # Replan if a tool is detected but not searched for.
                if has_detected_new and not has_detected_current:
                    print(f"Unexpected detection offers new opportunities.")
                    plan = replan(final_goal, visualization)
                    break
            time.sleep(sleep_step)
            visualization.succeed(action)
        else:
            break
