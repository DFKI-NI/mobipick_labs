#!/usr/bin/env python3
from geometry_msgs.msg import Pose
import unified_planning
import unified_planning.shortcuts
from tables_demo_planning.refactor.components import Item, Location
from tables_demo_planning.refactor.tables_demo_text import Simulation


def test_pick_n_place_demo() -> None:
    unified_planning.shortcuts.get_environment().credits_stream = None
    # Define environment values.
    item_locations = {
        Item.get("power_drill_with_grip_1"): Location.get("table_2"),
    }

    # Run the Pick & Place Demo.
    sim = Simulation(item_locations)
    sim.problem.add_goal(sim.domain.robot_at(sim.domain.robot, sim.domain.get(Pose, "base_home_pose")))
    sim.problem.add_goal(sim.domain.robot_has(sim.domain.robot, sim.domain.nothing))
    sim.problem.add_goal(sim.domain.item_offered(sim.domain.get(Item, "power_drill_with_grip_1")))

    print("Scenario: Mobipick shall fetch the power drill and hand it over to a person.")
    sim.run()


if __name__ == "__main__":
    test_pick_n_place_demo()
