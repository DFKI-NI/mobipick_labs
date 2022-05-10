import time
from unified_planning.model import FNode, Fluent, InstantaneousAction, Object, Problem
from unified_planning.plan import Plan
from unified_planning.shortcuts import And, Equals, Not, OneshotPlanner, Or, UserType
from plan_visualization import PlanVisualization

"""Tables demo purely on variables and UP"""


# Define object types and objects.

table_count = 6
klt_count = 1
Location = UserType("Location")
target_table = Object("target_table", Location)  # Note: target_table is tables[0].
tables = [target_table] + [Object(f"table_{table}", Location) for table in range(1, table_count)]
klt_locations = [Object(f"klt_{chr(klt + 65)}_location", Location) for klt in range(klt_count)]
on_robot_location = Object("on_robot_location", Location)
searched_location = Object("searched_location", Location)
unknown_location = Object("unknown_location", Location)
locations = tables + klt_locations + [on_robot_location, searched_location, unknown_location]

Item = UserType("Item")
power_drill = Object("power_drill", Item)
remote_control = Object("remote_control", Item)
screwdriver = Object("screwdriver", Item)
klts = [Object(f"klt_{chr(klt + 65)}", Item) for klt in range(klt_count)]
nothing = Object("nothing", Item)
tools = [power_drill, remote_control, screwdriver]
items = tools + klts + [nothing]

# Define all fluents.
robot_at = Fluent("RobotAt", Location)
robot_has = Fluent("RobotHas", Item)
believe_at = Fluent("BelieveAt", Location, item=Item)
is_klt = Fluent("IsKlt", Item, location=Location)

# Define all actions.

move = InstantaneousAction("Move", a=Location, b=Location, item=Item)
a, b, item = move.parameters
move.add_precondition(Equals(robot_at, a))
move.add_precondition(Equals(robot_has, item))
move.add_precondition(Or(Or(Equals(b, table) for table in tables), Equals(b, searched_location)))
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
place.add_precondition(And(Not(Equals(believe_at(klt), a)) for klt in klts))
place.add_precondition(Not(Equals(item, nothing)))
place.add_effect(robot_has, nothing)
place.add_effect(believe_at(item), a)

store = InstantaneousAction("Store", a=Location, b=Location, klt=Item, item=Item)
a, b, klt, item = store.parameters
store.add_precondition(Equals(robot_at, a))
store.add_precondition(Equals(robot_has, item))
store.add_precondition(Equals(believe_at(klt), a))
store.add_precondition(Equals(is_klt(b), klt))
store.add_precondition(Or(Equals(klt, check) for check in klts))
store.add_precondition(Not(Equals(item, klt)))
store.add_precondition(Not(Equals(item, nothing)))
store.add_effect(robot_has, nothing)
store.add_effect(believe_at(item), b)

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
# believed_item_positions = {power_drill: tables[1], klts[0]: tables[4], klts[1]: tables[1]}
believed_item_positions = {screwdriver: tables[4], klts[0]: target_table}
actual_item_positions = {
    power_drill: tables[3],
    remote_control: tables[1],
    screwdriver: tables[3],
    klts[0]: target_table,
}
# actual_item_positions = {power_drill: tables[1], remote_control: target_table, screwdriver: target_table, klts[0]: tables[4], klts[1]: tables[1]}
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
for action in (move, pick, place, store, search):
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
    print(f"Scenario: Robot shall place all items into a klt and bring them to {target_table}.")
    print("The item locations are:")
    for item in tools + klts:
        print(
            f"- {item}, believe: {believed_item_positions.get(item, unknown_location)}, "
            f"actual: {actual_item_positions.get(item, unknown_location)} "
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
            elif action.action == store:
                klt_location = parameters[1].object()
                klt = parameters[2].object()
                item = parameters[3].object()
                print(f"Place {item} into {klt} at {actual_robot_location}.")
                believed_item_positions[item] = actual_item_positions[item] = klt_location
                # TODO klt location might fail
            elif action.action == search:
                item = parameters[0].object()
                if believed_item_positions.get(item, unknown_location) != unknown_location:
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
                    if actual_item_positions.get(klt) == actual_robot_location:
                        print("- Detect klt.")
                        if believed_item_positions.get(klt) != actual_robot_location:
                            believed_item_positions[klt] = actual_robot_location
                            if klt == item:
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
