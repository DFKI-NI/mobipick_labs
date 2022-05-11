import time
from unified_planning.model import FNode, Fluent, InstantaneousAction, Object, Problem
from unified_planning.plan import Plan
from unified_planning.shortcuts import And, Equals, Not, OneshotPlanner, Or, UserType
from robot_api_up_demo.plan_visualization import PlanVisualization

"""Tables demo purely on variables and UP"""


# Define object types and objects.

table_count = 6
klt_count = 2
Location = UserType("Location")
target_table = Object("target_table", Location)  # Note: target_table is tables[0].
tables = [target_table] + [Object(f"table_{table}", Location) for table in range(1, table_count)]
klt_locations = [Object(f"klt_{chr(klt + 65)}_location", Location) for klt in range(klt_count)]
on_robot_location = Object("on_robot_location", Location)
searched_tool_location = Object("searched_tool_location", Location)
searched_klt_location = Object("searched_klt_location", Location)
unknown_location = Object("unknown_location", Location)
locations = (
    tables + klt_locations + [on_robot_location, searched_tool_location, searched_klt_location, unknown_location]
)

Item = UserType("Item")
power_drill = Object("power_drill", Item)
remote_control = Object("remote_control", Item)
screwdriver = Object("screwdriver", Item)
tools = [power_drill, remote_control, screwdriver]
klts = [Object(f"klt_{chr(klt + 65)}", Item) for klt in range(klt_count)]
nothing = Object("nothing", Item)
items = tools + klts + [nothing]

# Define all fluents.
robot_at = Fluent("RobotAt", Location)
robot_has = Fluent("RobotHas", Item)
believe_at = Fluent("BelieveAt", Location, item=Item)
is_klt = Fluent("IsKlt", Item, location=Location)

# Define all actions.

move = InstantaneousAction("Move", a=Location, b=Location)
a, b = move.parameters
move.add_precondition(Equals(robot_at, a))
move.add_precondition(
    Or(Or(Equals(b, table) for table in tables), Equals(b, searched_tool_location), Equals(b, searched_klt_location))
)
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
place.add_precondition(Or(Equals(item, klt) for klt in klts))
place.add_effect(robot_has, nothing)
place.add_effect(believe_at(item), a)

store = InstantaneousAction("Store", a=Location, b=Location, klt_location=Location, klt_item=Item, item=Item)
a, b, klt_location, klt_item, item = store.parameters
store.add_precondition(Equals(robot_at, a))
store.add_precondition(Equals(robot_has, item))
store.add_precondition(Equals(believe_at(klt_item), b))
store.add_precondition(Equals(is_klt(klt_location), klt_item))
store.add_precondition(Or(Or(Equals(b, table) for table in tables), Equals(b, searched_klt_location)))
store.add_precondition(Or(Equals(klt_item, klt) for klt in klts))
store.add_precondition(Not(Equals(item, klt_item)))
store.add_precondition(Not(Equals(item, nothing)))
store.add_effect(robot_at, b)
store.add_effect(robot_has, nothing)
store.add_effect(believe_at(item), klt_location)

search_tool = InstantaneousAction("SearchTool", item=Item)
(item,) = search_tool.parameters
search_tool.add_precondition(Not(Equals(robot_at, searched_tool_location)))
search_tool.add_precondition(Equals(believe_at(item), unknown_location))
search_tool.add_precondition(Or(Equals(item, tool) for tool in tools))
search_tool.add_effect(robot_at, searched_tool_location)
search_tool.add_effect(believe_at(item), searched_tool_location)

search_klt = InstantaneousAction("SearchKlt", item=Item)
(item,) = search_klt.parameters
search_klt.add_precondition(Not(Equals(robot_at, searched_klt_location)))
search_klt.add_precondition(Equals(believe_at(item), unknown_location))
search_klt.add_precondition(Or(Equals(item, klt) for klt in klts))
search_klt.add_effect(robot_at, searched_klt_location)
search_klt.add_effect(believe_at(item), searched_klt_location)

# Define environment values.
actual_robot_location = unknown_location
actual_robot_item = nothing
believed_item_locations = {power_drill: tables[1], klts[0]: tables[1]}
actual_item_locations = {
    power_drill: tables[3],
    remote_control: tables[1],
    screwdriver: tables[3],
    klts[0]: tables[2],
    klts[1]: tables[4],
}
actual_search_locations = {klt: believed_item_locations.get(klt, unknown_location) for klt in klts}
final_goal = Or(
    And(And(Equals(believe_at(tool), klt_location) for tool in tools), Equals(believe_at(klt), target_table))
    for klt_location, klt in zip(klt_locations, klts)
)
sleep_step = 1.0
sleep_replan = 2.0

# Compoese base UP problem.
problem = Problem("mobipick_tables")
for fluent in (robot_at, robot_has, believe_at):
    problem.add_fluent(fluent)
problem.add_fluent(is_klt, default_initial_value=nothing)
for klt_location, klt in zip(klt_locations, klts):
    problem.set_initial_value(is_klt(klt_location), klt)
for action in (move, pick, place, store, search_tool, search_klt):
    problem.add_action(action)
problem.add_objects(locations)
problem.add_objects(items)


def print_items() -> None:
    for item in tools + klts:
        print(
            f"- {item}, believe: {believed_item_locations.get(item, unknown_location)}, "
            f"actual: {actual_item_locations.get(item, unknown_location)} "
        )


def get_plan(goal: FNode) -> Plan:
    """Get UP plan to achieve goal."""
    problem.set_initial_value(robot_at, actual_robot_location)
    problem.set_initial_value(robot_has, actual_robot_item)
    for item in items:
        problem.set_initial_value(believe_at(item), believed_item_locations.get(item, unknown_location))
    problem.clear_goals()
    problem.add_goal(goal)

    with OneshotPlanner(problem_kind=problem.kind) as planner:
        result = planner.solve(problem)
        print(f"{result.planner_name} returned: {result.plan}")
    return result.plan


def replan(goal: FNode, visualization: PlanVisualization) -> Plan:
    """Replan to achieve goal and update visualization."""
    print_items()
    print("Replan ...")
    time.sleep(sleep_replan)
    plan = get_plan(goal)
    visualization.set_actions(plan.actions)
    return plan


if __name__ == '__main__':
    print(f"Scenario: Robot shall place all items into a klt and bring them to {target_table}.")
    print("The item locations are:")
    print_items()

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
                target_location = parameters[1].object()
                print(f"Move from {actual_robot_location} to {target_location} with {actual_robot_item}.")
                actual_robot_location = target_location
            elif action.action == pick:
                item = parameters[1].object()
                if actual_item_locations.get(item) == actual_robot_location:
                    print(f"Pick up {item} at {actual_robot_location}.")
                    believed_item_locations[item] = on_robot_location
                    actual_item_locations[item] = on_robot_location
                    actual_robot_item = item
                else:
                    print(f"Pick up {item} at {actual_robot_location} FAILED.")
                    believed_item_locations[item] = unknown_location
                    visualization.fail(action)
                    plan = replan(final_goal, visualization)
                    break
            elif action.action == place:
                item = parameters[1].object()
                print(f"Place {item} at {actual_robot_location}.")
                believed_item_locations[item] = actual_item_locations[item] = actual_robot_location
                actual_robot_item = nothing
            elif action.action == store:
                target_location = parameters[1].object()
                klt_location = parameters[2].object()
                klt_item = parameters[3].object()
                item = parameters[4].object()
                # Resolve a potentially symbolic target_location.
                if target_location == searched_klt_location:
                    assert (
                        klt_item in actual_search_locations.keys()
                        and actual_search_locations[klt_item] != unknown_location
                    )
                    target_location = actual_search_locations[klt_item]
                # Only move if target_location is different.
                if target_location != actual_robot_location:
                    print(f"Move from {actual_robot_location} to {target_location} with {actual_robot_item}.")
                    actual_robot_location = target_location
                if actual_item_locations.get(klt_item) == actual_robot_location:
                    print(f"Place {item} into {klt_item} at {actual_robot_location}.")
                    believed_item_locations[item] = actual_item_locations[item] = klt_location
                    actual_robot_item = nothing
                else:
                    print(f"Placing into {klt_item} at {actual_robot_location} FAILED.")
                    believed_item_locations[klt_item] = unknown_location
                    visualization.fail(action)
                    plan = replan(final_goal, visualization)
                    break
            elif action.action in (search_tool, search_klt):
                item = parameters[0].object()
                if believed_item_locations.get(item, unknown_location) != unknown_location:
                    print(f"Search for {item} OBSOLETE.")
                    plan = replan(final_goal, visualization)
                    break

                # Search for item by circling and observing the tables.
                print(f"Search for {item}.")
                index = tables.index(actual_robot_location) if actual_robot_location in tables else 0
                indices = [(index + offset - 1) % (table_count - 1) + 1 for offset in range(1, table_count)] + [0]
                has_detected_current = False
                has_detected_new = False
                for index in indices:
                    time.sleep(sleep_step)
                    print(f"- Move from {actual_robot_location} to {tables[index]}.")
                    actual_robot_location = tables[index]
                    time.sleep(sleep_step)
                    print(f"- Observe {actual_robot_location}.")
                    time.sleep(sleep_step)
                    # Detect all items on the current table.
                    for check_item in tools + klts:
                        if actual_item_locations.get(check_item) == actual_robot_location:
                            print(f"- Detect {check_item}.")
                            if check_item in klts:
                                actual_search_locations[check_item] = actual_robot_location
                            if believed_item_locations.get(check_item) != actual_robot_location:
                                believed_item_locations[check_item] = actual_robot_location
                                if check_item == item:
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
