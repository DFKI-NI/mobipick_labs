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
# Authors: Alexander Sung, Sebastian Stock

"""Run the Pick and Place demo with an execution loop using a behavior tree."""


import rospy
from geometry_msgs.msg import Pose
import unified_planning
from tables_demo_planning.components import Item, Location
from tables_demo_planning.tables_demo_api import TablesDemoAPI


def run_demo():
    """Run the handover demo."""

    # Define environment values.
    item_locations = {
        Item.get("power_drill_with_grip_1"): Location.get("table_2"),
    }

    api = TablesDemoAPI(item_locations)
    # Define handover goal.
    api.problem.add_goal(api.domain.robot_at(api.domain.robot, api.domain.get(Pose, "base_home_pose")))
    api.problem.add_goal(api.domain.robot_has(api.domain.robot, api.domain.nothing))
    api.problem.add_goal(api.domain.item_offered(api.domain.get(Item, "power_drill_with_grip_1")))

    print("Scenario: Mobipick shall fetch the power drill and hand it over to a person.")
    api.run()


if __name__ == '__main__':
    unified_planning.shortcuts.get_environment().credits_stream = None
    try:
        run_demo()
    except rospy.ROSInterruptException:
        pass
