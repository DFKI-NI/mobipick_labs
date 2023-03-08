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


from typing import Set
import rospy
from std_msgs.msg import String
import unified_planning
from unified_planning.model.metrics import MinimizeSequentialPlanLength
from tables_demo_planning.pick_n_place_demo import PickAndPlaceDomain
from tables_demo_planning.subplan_visualization import SubPlanVisualization

"""Running the Pick and Place demo with a simple execution loop"""


def run_demo():
    """Run the pick and place demo."""
    domain = PickAndPlaceDomain()
    espeak_pub = rospy.Publisher("/espeak_node/speak_line", String, queue_size=1)
    visualization = SubPlanVisualization()
    executed_actions: Set[str] = set()
    error_count = 0
    active = True
    while active:
        # Create problem based on current state.
        problem = domain.initialize_problem()
        problem.add_quality_metric(MinimizeSequentialPlanLength())
        domain.set_initial_values(problem)
        domain.set_goals(problem)

        # Plan
        actions = domain.solve(problem)
        if not actions:
            if actions is None:
                print("Execution ended because no plan could be found.")
                espeak_pub.publish("Mission impossible!")
                domain.visualization.add_node("Mission impossible", "red")
            else:
                print("Demo complete.")
                espeak_pub.publish("Demo complete.")
                domain.visualization.add_node("Demo complete", "green")
            active = False
            return

        print("> Plan:")
        print('\n'.join(map(str, actions)))
        action_names = [
            f"{len(executed_actions) + number} {domain.label(action)}" for number, action in enumerate(actions, start=1)
        ]
        visualization.set_actions(action_names, preserve_actions=executed_actions)
        # ... and execute.
        print("> Execution:")
        for action in actions:
            function, parameters = domain.get_executable_action(action)
            action_name = f"{len(executed_actions) + 1} {domain.label(action)}"
            print(action)
            visualization.execute(action_name)
            espeak_pub.publish(domain.label(action))
            result = function(*parameters)
            executed_actions.add(action_name)
            if rospy.is_shutdown():
                return

            if result is None or result:
                visualization.succeed(action_name)
            else:
                print("-- Action failed! Need to replan.")
                error_count += 1
                visualization.fail(action_name)
                espeak_pub.publish("Action failed.")
                if error_count >= 3:
                    print("Execution ended after too many failures.")
                    espeak_pub.publish("Mission impossible!")
                    visualization.add_node("Mission impossible", "red")
                    return

                # Abort execution and loop to planning.
                break
        else:
            active = False
            print("Demo complete.")
            espeak_pub.publish("Demo complete.")
            visualization.add_node("Demo complete", "green")


if __name__ == '__main__':
    unified_planning.shortcuts.get_environment().credits_stream = None
    try:
        run_demo()
    except rospy.ROSInterruptException:
        pass
