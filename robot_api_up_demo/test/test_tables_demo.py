#!/usr/bin/env python3
from typing import Dict, Optional
from dataclasses import dataclass
import time
from unified_planning.model import FNode, Object
from unified_planning.plan import Plan
from unified_planning.shortcuts import And, Equals, Or
from robot_api_up_demo.subplan_visualization import SubPlanVisualization
from robot_api_up_demo.up_tables_demo import UnifiedPlanning


@dataclass
class Environment:
    actual_robot_location: Object
    actual_robot_item: Object
    believed_item_locations: Dict[Object, Object]
    actual_item_locations: Dict[Object, Object]
    actual_search_locations: Dict[Object, Object]
    action_success_count: int = 0
    subaction_success_count: int = 0


def print_items(up: UnifiedPlanning, env: Environment) -> None:
    for item in up.tools + up.klts:
        print(
            f"- {item}, believe: {env.believed_item_locations.get(item, up.unknown_location)}, "
            f"actual: {env.actual_item_locations.get(item, up.unknown_location)} "
        )


def get_plan(
    up: UnifiedPlanning,
    env: Environment,
    goal: FNode,
    visualization: Optional[SubPlanVisualization],
    replan: bool = False,
) -> Plan:
    print_items(up, env)
    if replan:
        print("Replan ...")
    result = up.plan(env.actual_robot_location, env.actual_robot_item, env.believed_item_locations, goal)
    print(f"{result.engine_name} returned: {result.plan}")
    assert result.plan
    if visualization:
        visualization.set_actions(
            f"{index + env.action_success_count + 1} {action}" for index, action in enumerate(result.plan.actions)
        )
        time.sleep(1.0)
    return result.plan


def get_search_plan(
    up: UnifiedPlanning,
    env: Environment,
    search_goal: FNode,
    visualization: Optional[SubPlanVisualization],
    action_name: str,
) -> Plan:
    result = up.plan_search(env.actual_robot_location, env.believed_item_locations, search_goal)
    print(f"{result.engine_name} returned: {result.plan}")
    assert result.plan
    if visualization:
        visualization.set_actions(
            [
                f"{env.action_success_count + 1}{chr(index + 97)} {action}"
                for index, action in enumerate(result.plan.actions)
            ],
            action_name,
        )
        time.sleep(1.0)
    return result.plan


def move(env: Environment, target_location: Object, prefix: str = "") -> None:
    print(f"{prefix}Move from {env.actual_robot_location} to {target_location} with {env.actual_robot_item}.")
    env.actual_robot_location = target_location


def update(visualization: Optional[SubPlanVisualization], action_name: str, success: bool) -> None:
    if visualization:
        if success:
            visualization.succeed(action_name)
        else:
            visualization.fail(action_name)
        time.sleep(1.0)


def execute(up: UnifiedPlanning, env: Environment, goal: FNode, visualize: bool) -> None:
    print(f"Scenario: Robot shall place all items into a klt and bring them to {up.target_table}.")
    print("The item locations are:")

    visualization = SubPlanVisualization() if visualize else None
    time.sleep(1.0)
    plan = get_plan(up, env, goal, visualization)
    while plan:
        for action in plan.actions:
            action_name = f"{env.action_success_count + 1} {action}"
            if visualization:
                # Abort plan execution on closing the visualization window.
                if not visualization.window.props.visible:
                    visualization.thread.join()
                    plan = None
                    break

                visualization.execute(action_name)
                time.sleep(1.0)

            parameters = action.actual_parameters
            if action.action == up.move:
                update(visualization, action_name, True)
                move(env, parameters[1].object())
            elif action.action == up.pick:
                item = parameters[1].object()
                if env.actual_item_locations.get(item) == env.actual_robot_location:
                    print(f"Pick up {item} at {env.actual_robot_location}.")
                    update(visualization, action_name, True)
                    env.believed_item_locations[item] = up.on_robot_location
                    env.actual_item_locations[item] = up.on_robot_location
                    env.actual_robot_item = item
                else:
                    print(f"Pick up {item} at {env.actual_robot_location} FAILED.")
                    update(visualization, action_name, False)
                    env.believed_item_locations[item] = up.unknown_location
                    plan = get_plan(up, env, goal, visualization, replan=True)
                    break
            elif action.action == up.place:
                item = parameters[1].object()
                print(f"Place {item} at {env.actual_robot_location}.")
                update(visualization, action_name, True)
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
                    move(env, target_location)
                    time.sleep(1.0)
                # Place item into klt_item.
                if env.actual_item_locations.get(klt_item) == env.actual_robot_location:
                    print(f"Place {item} into {klt_item} at {env.actual_robot_location}.")
                    update(visualization, action_name, True)
                    env.believed_item_locations[item] = env.actual_item_locations[item] = klt_location
                    env.actual_robot_item = up.nothing
                else:
                    print(f"Placing into {klt_item} at {env.actual_robot_location} FAILED.")
                    update(visualization, action_name, False)
                    env.believed_item_locations[klt_item] = up.unknown_location
                    plan = get_plan(up, env, goal, visualization, replan=True)
                    break
            elif action.action in (up.search_tool, up.search_klt):
                item = parameters[0].object()
                if env.believed_item_locations.get(item, up.unknown_location) != up.unknown_location:
                    print(f"Search for {item} OBSOLETE.")
                    update(visualization, action_name, False)
                    plan = get_plan(up, env, goal, visualization, replan=True)
                    break

                # Search for item by generating and executing a subplan.
                print(f"Search for {item}.")
                search_goal = Equals(
                    up.believe_at(item), up.searched_klt_location if item in up.klts else up.searched_tool_location
                )
                search_plan = get_search_plan(up, env, search_goal, visualization, action_name)
                env.subaction_success_count = 0

                has_detected_current = False
                has_detected_new = False
                for subaction in search_plan.actions:
                    subaction_name = (
                        f"{env.action_success_count + 1}{chr(env.subaction_success_count + 97)} {subaction}"
                    )
                    if visualization:
                        # Abort plan execution on closing the visualization window.
                        if not visualization.window.props.visible:
                            visualization.thread.join()
                            plan = None
                            break

                        visualization.execute(subaction_name)
                        time.sleep(1.0)

                    if subaction.action == up.move:
                        move(env, subaction.actual_parameters[1].object(), prefix="- ")
                    elif subaction.action == up.search_at:
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
                        if has_detected_current:
                            print("- Search successful. Omitting remaining search actions.")
                            update(visualization, subaction_name, True)
                            break
                        elif has_detected_new:
                            print("- Search ABORTED.")
                            update(visualization, subaction_name, False)
                            break

                    env.subaction_success_count += 1
                else:
                    print(f"Search failed. No solution found.")
                    update(visualization, action_name, False)
                    plan = None
                    break

                # Replan if a tool is detected but not searched for.
                if has_detected_new and not has_detected_current:
                    print(f"Unexpected detection offers new opportunities.")
                    update(visualization, action_name, False)
                    plan = get_plan(up, env, goal, visualization, replan=True)
                    break

                update(visualization, action_name, True)
            if plan is None:
                break
            env.action_success_count += 1
        else:
            break


def test_tables_demo(visualize: bool = False) -> None:
    up = UnifiedPlanning(6, 1)
    # Define environment values.
    believed_item_locations = {up.power_drill: up.tables[3], up.klts[0]: up.tables[3]}
    actual_item_locations = {
        up.power_drill: up.tables[2],
        up.remote_control: up.tables[3],
        up.screwdriver: up.tables[2],
        up.klts[0]: up.tables[1],
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
    execute(up, env, final_goal, visualize)


def test_search_problem() -> None:
    up = UnifiedPlanning(6, 1)
    believed_item_locations = {up.power_drill: up.tables[1], up.klts[0]: up.tables[1]}
    search_goal = Equals(up.believe_at(up.screwdriver), up.searched_tool_location)
    result = up.plan_search(up.unknown_location, believed_item_locations, search_goal)
    assert result.plan and len(result.plan.actions) == 2 * len(up.tables) + 1


if __name__ == '__main__':
    test_tables_demo(visualize=True)
