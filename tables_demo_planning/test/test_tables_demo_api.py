#!/usr/bin/env python3
from typing import List
import unified_planning
import unified_planning.shortcuts
from unified_planning.shortcuts import And, Or
from tables_demo_planning.components import Item, Location
from tables_demo_planning.tables_demo_api import TablesDemoAPI


def set_goals(api: TablesDemoAPI, demo_items: List[Item], target_location: Location) -> None:
    """Set the goals for the overall demo."""
    assert api.domain.believe_item_at in api.problem.fluents
    assert all(api.problem.object(item.name) for item in demo_items)
    assert api.problem.object(target_location.name)
    api.problem.clear_goals()
    if any(item.name.startswith("klt_") for item in demo_items):
        assert api.problem.object("in_klt")
        if any(item.name.startswith("multimeter_") for item in demo_items):
            api.problem.add_goal(
                Or(
                    And(
                        api.domain.believe_item_in(api.domain.objects[item.name], klt),
                        api.domain.believe_item_at(klt, api.domain.objects[target_location.name]),
                    )
                    for item in demo_items
                    if item.name.startswith("multimeter_")
                    for klt in api.domain.get_klt_objects()
                )
            )
    else:
        # Note: Just for testing without KLT.
        if any(item.name.startswith("multimeter_") for item in demo_items):
            for item in demo_items:
                if item.name.startswith("multimeter_"):
                    api.problem.add_goal(
                        api.domain.believe_item_at(
                            api.domain.objects[item.name], api.domain.objects[target_location.name]
                        )
                    )


def test_tables_demo_api() -> None:
    unified_planning.shortcuts.get_environment().credits_stream = None
    # Define environment values.
    item_locations = {
        Item.get("multimeter_1"): Location.get("table_3"),
        Item.get("klt_1"): Location.get("table_2"),
        Item.get("klt_2"): Location.get("table_1"),
        Item.get("klt_3"): Location.get("table_3"),
    }
    # Define goal.
    target_location = Location.get("table_2")

    # Run the Mobipick Tables Demo.
    api = TablesDemoAPI(item_locations)
    set_goals(api, list(item_locations.keys()), target_location)
    print(f"Scenario: Mobipick shall bring a KLT with a multimeter inside to {target_location.name}.")
    api.run(target_location)


if __name__ == "__main__":
    test_tables_demo_api()
