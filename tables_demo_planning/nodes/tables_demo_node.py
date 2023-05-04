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
# Authors: Alexander Sung, Oscar Lima, Marc Vinci, Sebastian Stock

"""Main execution node of the tables demo."""


import sys
import rospy
import unified_planning
from tables_demo_planning.mobipick_components import Location
from tables_demo_planning.tables_demo_api import TablesDemoAPIDomain


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
        TablesDemoAPIDomain(target_location).run()
    except rospy.ROSInterruptException:
        pass
