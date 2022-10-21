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
# Authors: Alexander Sung, DFKI


from typing import List, Sequence
from pydot import Dot, Edge, Node
from std_msgs.msg import String
import rospy

"""
Helper component which maintains the dot graph of a simple sequential plan,
visualized by the dot_graph_visualization repository.
"""


class PlanVisualization:
    def __init__(self, actions: Sequence[object]) -> None:
        self.graph: Dot = None
        self.nodes: List[Node] = []
        self.edges: List[Edge] = []
        self.actions: Sequence[object] = []
        self.plan_pub = rospy.Publisher("/dot_graph_visualization/dot_graph", String, queue_size=1)
        self.set_actions(actions)

    def visualize(self) -> None:
        """Update the visualization by dot code from self.graph."""
        self.plan_pub.publish(self.graph.to_string())

    def set_actions(self, actions: Sequence[object]) -> None:
        """Create and visualize a new graph, built from actions."""
        self.actions = actions
        self.graph = Dot("Plan", graph_type="digraph", bgcolor="white")
        self.nodes = [Node(str(action), style="filled", fillcolor="white") for action in actions]
        for node in self.nodes:
            self.graph.add_node(node)
        self.edges = [Edge(node, self.nodes[number]) for number, node in enumerate(self.nodes[:-1], start=1)]
        for edge in self.edges:
            self.graph.add_edge(edge)
        self.visualize()

    def execute(self, action: object) -> None:
        """Mark action as being executed."""
        index = self.actions.index(action)
        self.nodes[index].set("fillcolor", "yellow")
        if index > 0:
            self.edges[index - 1].set("color", "green")
        self.visualize()

    def succeed(self, action: object) -> None:
        """Mark action as successful."""
        self.nodes[self.actions.index(action)].set("fillcolor", "green")
        self.visualize()

    def fail(self, action: object) -> None:
        """Mark action as failed."""
        self.nodes[self.actions.index(action)].set("fillcolor", "red")
        self.visualize()
