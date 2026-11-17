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
