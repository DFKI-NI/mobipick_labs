#!/usr/bin/env python3
import sys
import unified_planning
import unified_planning.shortcuts
from tables_demo_planning.components import Item, Location
from tables_demo_planning.tables_demo_text import Simulation


def test_tables_demo() -> None:
    unified_planning.shortcuts.get_environment().credits_stream = None
    # Define environment values.
    item_locations = {
        Item.get("multimeter_1"): Location.get("table_3"),
        Item.get("klt_1"): Location.get("table_1"),
        Item.get("klt_2"): Location.get("table_3"),
    }
    # Run the Mobipick Tables Demo.
    sim = Simulation(item_locations, table_count=3)
    # Define goal.
    goal_strs = sys.argv[1:]
    if goal_strs:
        sim.domain.set_goals_by_strs(sim.problem, goal_strs)
        sim.run()
    else:
        target_location = Location.get("table_2")
        sim.domain.set_goals(sim.problem, list(item_locations.keys()), target_location)
        print(f"Scenario: Mobipick shall bring a KLT with a multimeter inside to {target_location.name}.")
        sim.run(target_location)


if __name__ == "__main__":
    test_tables_demo()
