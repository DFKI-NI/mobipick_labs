#!/usr/bin/env python3
from typing import List
import unified_planning
import unified_planning.shortcuts
from tables_demo_planning.refactor.components import Item, Location
from tables_demo_planning.refactor.tables_demo_text import Simulation


def set_goals(sim: Simulation, demo_items: List[Item], target_location: Location) -> None:
    """Set the goals for the overall demo."""
    sim.problem.clear_goals()
    assert sim.domain.believe_item_at in sim.problem.fluents
    if any(item.name == "klt_1" for item in demo_items):
        assert sim.problem.object("in_klt")
        if any(item.name == "multimeter_1" for item in demo_items):
            assert sim.problem.object("multimeter_1")
            sim.problem.add_goal(
                sim.domain.believe_item_at(sim.domain.get(Item, "multimeter_1"), sim.domain.get(Location, "in_klt"))
            )
        if any(item.name == "relay_1" for item in demo_items):
            assert sim.problem.object("relay_1")
            sim.problem.add_goal(
                sim.domain.believe_item_at(sim.domain.get(Item, "relay_1"), sim.domain.get(Location, "in_klt"))
            )
        if any(item.name == "screwdriver_1" for item in demo_items):
            assert sim.problem.object("screwdriver_1")
            sim.problem.add_goal(
                sim.domain.believe_item_at(sim.domain.get(Item, "screwdriver_1"), sim.domain.get(Location, "in_klt"))
            )
        assert sim.problem.object(target_location.name)
        sim.problem.add_goal(
            sim.domain.believe_item_at(sim.domain.get(Item, "klt_1"), sim.domain.objects[target_location.name])
        )
    else:
        if any(item.name == "multimeter_1" for item in demo_items):
            assert sim.problem.object("multimeter_1")
            sim.problem.add_goal(
                sim.domain.believe_item_at(
                    sim.domain.get(Item, "multimeter_1"), sim.domain.objects[target_location.name]
                )
            )
        if any(item.name == "relay_1" for item in demo_items):
            assert sim.problem.object("relay_1")
            sim.problem.add_goal(
                sim.domain.believe_item_at(sim.domain.get(Item, "relay_1"), sim.domain.objects[target_location.name])
            )
        if any(item.name == "screwdriver_1" for item in demo_items):
            assert sim.problem.object("screwdriver_1")
            sim.problem.add_goal(
                sim.domain.believe_item_at(
                    sim.domain.get(Item, "screwdriver_1"), sim.domain.objects[target_location.name]
                )
            )


def test_tables_demo() -> None:
    unified_planning.shortcuts.get_environment().credits_stream = None
    # Define environment values.
    item_locations = {
        Item.get("multimeter_1"): Location.get("table_3"),
        Item.get("klt_1"): Location.get("table_1"),
    }
    # Define goal.
    target_location = Location.get("table_2")

    # Run the Mobipick Tables Demo.
    sim = Simulation(item_locations)
    set_goals(sim, list(item_locations.keys()), target_location)
    print(f"Scenario: Mobipick shall bring the KLT with the multimeter inside to {target_location.name}.")
    sim.run(target_location)


if __name__ == "__main__":
    test_tables_demo()
