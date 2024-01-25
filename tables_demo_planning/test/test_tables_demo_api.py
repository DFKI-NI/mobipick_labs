#!/usr/bin/env python3
import unified_planning
import unified_planning.shortcuts
from tables_demo_planning.components import Item, Location
from tables_demo_planning.tables_demo_api import TablesDemoAPI


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
    api.domain.set_goals(api.problem, list(item_locations.keys()), target_location)
    print(f"Scenario: Mobipick shall bring a KLT with a multimeter inside to {target_location.name}.")
    api.run(target_location)


if __name__ == "__main__":
    test_tables_demo_api()
