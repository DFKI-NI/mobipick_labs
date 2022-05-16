#!/usr/bin/env python3
from typing import Dict
from dataclasses import dataclass
from unified_planning.model import FNode, Object
from unified_planning.plan import Plan
from unified_planning.shortcuts import And, Equals, Or
from robot_api_up_demo.up_tables_problem import UnifiedPlanning


@dataclass
class Environment:
    actual_robot_location: Object
    actual_robot_item: Object
    believed_item_locations: Dict[Object, Object]
    actual_item_locations: Dict[Object, Object]
    actual_search_locations: Dict[Object, Object]


def print_items(up: UnifiedPlanning, env: Environment) -> None:
    for item in up.tools + up.klts:
        print(
            f"- {item}, believe: {env.believed_item_locations.get(item, up.unknown_location)}, "
            f"actual: {env.actual_item_locations.get(item, up.unknown_location)} "
        )


def get_plan(up: UnifiedPlanning, env: Environment, goal: FNode, replan: bool = False) -> Plan:
    print_items(up, env)
    if replan:
        print("Replan ...")
    result = up.plan(env.actual_robot_location, env.actual_robot_item, env.believed_item_locations, goal)
    print(f"{result.planner_name} returned: {result.plan}")
    assert result.plan
    return result.plan


def execute(up: UnifiedPlanning, env: Environment, goal: FNode) -> None:
    print(f"Scenario: Robot shall place all items into a klt and bring them to {up.target_table}.")
    print("The item locations are:")

    plan = get_plan(up, env, goal)
    while plan:
        for action in plan.actions:
            parameters = action.actual_parameters
            if action.action == up.move:
                target_location = parameters[1].object()
                print(f"Move from {env.actual_robot_location} to {target_location}" f" with {env.actual_robot_item}.")
                env.actual_robot_location = target_location
            elif action.action == up.pick:
                item = parameters[1].object()
                if env.actual_item_locations.get(item) == env.actual_robot_location:
                    print(f"Pick up {item} at {env.actual_robot_location}.")
                    env.believed_item_locations[item] = up.on_robot_location
                    env.actual_item_locations[item] = up.on_robot_location
                    env.actual_robot_item = item
                else:
                    print(f"Pick up {item} at {env.actual_robot_location} FAILED.")
                    env.believed_item_locations[item] = up.unknown_location
                    plan = get_plan(up, env, goal, replan=True)
                    break
            elif action.action == up.place:
                item = parameters[1].object()
                print(f"Place {item} at {env.actual_robot_location}.")
                env.believed_item_locations[item] = env.actual_item_locations[item] = env.actual_robot_location
                env.actual_robot_item = up.nothing
            elif action.action == up.store:
                target_location = parameters[1].object()
                klt_location = parameters[2].object()
                klt_item = parameters[3].object()
                item = parameters[4].object()
                # Resolve a potentially symbolic target_location.
                if target_location == up.searched_klt_location:
                    assert (
                        klt_item in env.actual_search_locations.keys()
                        and env.actual_search_locations[klt_item] != up.unknown_location
                    )
                    target_location = env.actual_search_locations[klt_item]
                # Only move if target_location is different.
                if target_location != env.actual_robot_location:
                    print(
                        f"Move from {env.actual_robot_location} to {target_location}" f" with {env.actual_robot_item}."
                    )
                    env.actual_robot_location = target_location
                if env.actual_item_locations.get(klt_item) == env.actual_robot_location:
                    print(f"Place {item} into {klt_item} at {env.actual_robot_location}.")
                    env.believed_item_locations[item] = env.actual_item_locations[item] = klt_location
                    env.actual_robot_item = up.nothing
                else:
                    print(f"Placing into {klt_item} at {env.actual_robot_location} FAILED.")
                    env.believed_item_locations[klt_item] = up.unknown_location
                    plan = get_plan(up, env, goal, replan=True)
                    break
            elif action.action in (up.search_tool, up.search_klt):
                item = parameters[0].object()
                if env.believed_item_locations.get(item, up.unknown_location) != up.unknown_location:
                    print(f"Search for {item} OBSOLETE.")
                    plan = get_plan(up, env, goal, replan=True)
                    break

                # Search for item by circling and observing the tables.
                print(f"Search for {item}.")
                index = up.tables.index(env.actual_robot_location) if env.actual_robot_location in up.tables else 0
                indices = [(index + offset - 1) % (len(up.tables) - 1) + 1 for offset in range(1, len(up.tables))] + [0]
                has_detected_current = False
                has_detected_new = False
                for index in indices:
                    print(f"- Move from {env.actual_robot_location} to {up.tables[index]}.")
                    env.actual_robot_location = up.tables[index]
                    print(f"- Observe {env.actual_robot_location}.")
                    # Detect all items on the current table.
                    for check_item in up.tools + up.klts:
                        if env.actual_item_locations.get(check_item) == env.actual_robot_location:
                            print(f"- Detect {check_item}.")
                            if check_item in up.klts:
                                env.actual_search_locations[check_item] = env.actual_robot_location
                            if env.believed_item_locations.get(check_item) != env.actual_robot_location:
                                env.believed_item_locations[check_item] = env.actual_robot_location
                                if check_item == item:
                                    has_detected_current = True
                                else:
                                    has_detected_new = True
                    # If something is detected, stop searching.
                    if has_detected_current or has_detected_new:
                        break
                else:
                    print(f"Search failed. No solution found.")
                    plan = None
                    break

                # Replan if a tool is detected but not searched for.
                if has_detected_new and not has_detected_current:
                    print(f"Unexpected detection offers new opportunities.")
                    plan = get_plan(up, env, goal, replan=True)
                    break
        else:
            break


def test_tables_demo() -> None:
    up = UnifiedPlanning(6, 1)
    # Define environment values.
    believed_item_locations = {up.power_drill: up.tables[1], up.klts[0]: up.tables[1]}
    actual_item_locations = {
        up.power_drill: up.tables[3],
        up.remote_control: up.tables[1],
        up.screwdriver: up.tables[3],
        up.klts[0]: up.tables[2],
    }
    if len(up.klts) > 1:
        actual_item_locations[up.klts[1]] = up.tables[4]
    actual_search_locations = {klt: believed_item_locations.get(klt, up.unknown_location) for klt in up.klts}
    env = Environment(
        up.unknown_location, up.nothing, believed_item_locations, actual_item_locations, actual_search_locations
    )
    # Define goal.
    final_goal = Or(
        And(
            And(Equals(up.believe_at(tool), klt_location) for tool in up.tools),
            Equals(up.believe_at(klt), up.target_table),
        )
        for klt_location, klt in zip(up.klt_locations, up.klts)
    )
    execute(up, env, final_goal)


def test_search_problem() -> None:
    up = UnifiedPlanning(6, 1)
    believed_item_locations = {up.power_drill: up.tables[1], up.klts[0]: up.tables[1]}
    search_goal = Equals(up.believe_at(up.screwdriver), up.searched_tool_location)
    result = up.plan_search(up.screwdriver, up.unknown_location, believed_item_locations, search_goal)
    assert result.plan and len(result.plan.actions) == 2 * len(up.tables) + 1


if __name__ == '__main__':
    test_tables_demo()
