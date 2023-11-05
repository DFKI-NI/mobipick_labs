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
# Authors: Sebastian Stock, Marc Vinci


import sys
import rospy
import actionlib
from tables_demo_planning.msg import (
    PlanAndExecuteTaskAction,
    PlanAndExecuteTaskGoal,
)


def main():
    rospy.init_node("task_server_client_node")
    task_server_name = rospy.get_param(
        "~task_server_name",
        default="/mobipick/task_planning",
    )
    client = actionlib.SimpleActionClient(task_server_name, PlanAndExecuteTaskAction)
    client.wait_for_server()

    goal = PlanAndExecuteTaskGoal()
    if len(sys.argv) >= 2:
        goal.task = sys.argv[1]
        goal.parameters = sys.argv[2:]
        client.send_goal_and_wait()
        res = client.get_result()
        if res.success:
            rospy.loginfo("Task execution succeeded!")
        else:
            rospy.loginfo("Task execution failed: %s" % res.message)

    else:
        rospy.logerr("Missing arguments")
        rospy.logerr("Usage: task_server_client task_name [arg1] [arg2] ... [arg_n]")


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
