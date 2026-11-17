#!/usr/bin/env python3
# Copyright (c) 2026, DFKI GmbH and contributors
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the copyright holder nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from geometry_msgs.msg import Pose
import unified_planning
import unified_planning.shortcuts
from tables_demo_planning.components import Item, Location
from tables_demo_planning.tables_demo_text import Simulation


def test_pick_n_place_demo() -> None:
    unified_planning.shortcuts.get_environment().credits_stream = None
    # Define environment values.
    item_locations = {
        Item.get("power_drill_with_grip_1"): Location.get("table_2"),
    }

    # Run the Pick & Place Demo.
    sim = Simulation(item_locations, table_count=3)
    sim.problem.add_goal(sim.domain.robot_at(sim.domain.robot, sim.domain.get(Pose, "base_home_pose")))
    sim.problem.add_goal(sim.domain.robot_has(sim.domain.robot, sim.domain.nothing))
    sim.problem.add_goal(sim.domain.item_offered(sim.domain.get(Item, "power_drill_with_grip_1")))

    print("Scenario: Mobipick shall fetch the power drill and hand it over to a person.")
    sim.run()


if __name__ == "__main__":
    test_pick_n_place_demo()
