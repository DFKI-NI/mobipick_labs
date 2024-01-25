#!/usr/bin/env python3
from typing import List
import unified_planning
import unified_planning.shortcuts
from unified_planning.shortcuts import And, Or
from tables_demo_planning.components import Item, Location
from tables_demo_planning.tables_demo_text import Simulation


def set_goals(sim: Simulation, demo_items: List[Item], target_location: Location) -> None:
    """Set the goals for the overall demo."""
    assert sim.domain.believe_item_at in sim.problem.fluents
    assert all(sim.problem.object(item.name) for item in demo_items)
    assert sim.problem.object(target_location.name)
    sim.problem.clear_goals()
    if any(item.name.startswith("klt_") for item in demo_items):
        assert sim.problem.object("in_klt")
        if any(item.name.startswith("multimeter_") for item in demo_items):
            sim.problem.add_goal(
                Or(
                    And(
                        sim.domain.believe_item_in(sim.domain.objects[item.name], klt),
                        sim.domain.believe_item_at(klt, sim.domain.objects[target_location.name]),
                    )
                    for item in demo_items
                    if item.name.startswith("multimeter_")
                    for klt in sim.domain.get_klt_objects()
                )
            )
    else:
        # Note: Just for testing without KLT.
        if any(item.name.startswith("multimeter_") for item in demo_items):
            for item in demo_items:
                if item.name.startswith("multimeter_"):
                    sim.problem.add_goal(
                        sim.domain.believe_item_at(
                            sim.domain.objects[item.name], sim.domain.objects[target_location.name]
                        )
                    )


def test_tables_demo() -> None:
    unified_planning.shortcuts.get_environment().credits_stream = None
    # Define environment values.
    item_locations = {
        Item.get("multimeter_1"): Location.get("table_3"),
        Item.get("klt_1"): Location.get("table_1"),
        Item.get("klt_2"): Location.get("table_3"),
    }
    # Define goal.
    target_location = Location.get("table_2")

    # Run the Mobipick Tables Demo.
    sim = Simulation(item_locations)
    set_goals(sim, list(item_locations.keys()), target_location)
    print(f"Scenario: Mobipick shall bring a KLT with a multimeter inside to {target_location.name}.")
    sim.run(target_location)


if __name__ == "__main__":
    test_tables_demo()
