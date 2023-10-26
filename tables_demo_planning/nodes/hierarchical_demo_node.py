#!/usr/bin/env python3

# Software License Agreement (BSD License)
#
#  Copyright (c) 2022, DFKI GmbH
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#   * Neither the name of Willow Garage nor the names of its
#     contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.
#
#
# Authors: Sebastian Stock, Alexander Sung, Marc Vinci

"""
Main execution node of the tables demo using hierarchical planning for execution.
Development in progress.
"""


import sys
import rospy
import unified_planning
from typing import Optional
from tables_demo_planning.mobipick_components import Item, Location
from tables_demo_planning.hierarchical_domain import HierarchicalDomain
from tables_demo_planning.subplan_visualization import SubPlanVisualization


class HierarchicalDemoOrchestrator:
    def __init__(self) -> None:
        self._domain = HierarchicalDomain()
        self.visualization: Optional[SubPlanVisualization] = None
        self.espeak_pub: Optional[rospy.Publisher] = None
        self._trigger_replanning = False  # Temporary solution until it is provided by dispatcher

    def generate_and_execute_plan(self, target_item: Item, target_box: Item, target_location: Location) -> None:
        print(
            f"Scenario: Mobipick shall bring the {self._domain.objects[target_box.name]}"
            f" with the {self._domain.objects[target_item.name]} inside"
            f" to {self._domain.objects[target_location.name]}."
        )

        self._domain.run(
            self._domain.objects[target_item.name],
            self._domain.objects[target_box.name],
            self._domain.objects[target_location.name],
        )


if __name__ == '__main__':
    unified_planning.shortcuts.get_environment().credits_stream = None
    try:
        target_location = Location.table_2
        if len(sys.argv) >= 2:
            parameter = sys.argv[1]
            if parameter in ("1", "table1", "table_1"):
                target_location = Location.table_1
            elif parameter in ("2", "table2", "table_2"):
                target_location = Location.table_2
            elif parameter in ("3", "table3", "table_3"):
                target_location = Location.table_3
            else:
                rospy.logwarn(f"Unknown parameter '{parameter}', using default table.")

        target_item = Item.get("multimeter")
        target_box = Item.get("box")
        HierarchicalDemoOrchestrator().generate_and_execute_plan(target_item, target_box, target_location)
    except rospy.ROSInterruptException:
        pass
