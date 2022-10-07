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


from typing import Dict, List, Optional, Sequence, Set, Tuple
from dataclasses import dataclass
from pydot import Dot, Edge, Node
from std_msgs.msg import String
import rospy

"""
Helper component which maintains the dot graph of a demo specific hierarchical plan,
visualized by the dot_graph_visualization repository.
"""


@dataclass
class VisualizationNode:
    edge: Optional[Edge]  # Note: incoming edge
    node: Node
    action: object


class SubPlanVisualization:
    def __init__(self) -> None:
        self.graph: Dot = None
        self.nodes: Dict[str, VisualizationNode] = {}
        self.plan_pub = rospy.Publisher("/dot_graph_visualization/dot_graph", String, queue_size=1)

    def update(self) -> None:
        """Update visualization by dot code from self.graph."""
        self.plan_pub.publish(self.graph.to_string())

    def set_actions(
        self,
        actions: Sequence[object],
        preserve_actions: Optional[Set[object]] = None,
        predecessor: Optional[object] = None,
    ) -> None:
        """Update graph with actions by adding them to predecessor, or creating a new one."""
        self.graph = Dot("Plan", graph_type="digraph", bgcolor="white")
        nodes: List[Tuple[str, VisualizationNode]] = []
        if predecessor is not None:
            # Keep nodes up to predecessor and append rest after adding new nodes.
            nodes.extend(self.nodes.items())
            while nodes:
                action_name, node = nodes.pop(0)
                # Keep main plan only.
                if action_name.split(' ', maxsplit=1)[0].isdigit():
                    self.graph.add_node(node.node)
                    if node.edge:
                        self.graph.add_edge(node.edge)
                if action_name == predecessor:
                    break
        elif preserve_actions:
            for action_name, node in list(self.nodes.items()):
                if node.action in preserve_actions:
                    self.graph.add_node(node.node)
                    if node.edge:
                        self.graph.add_edge(node.edge)
                    predecessor = node.action
                else:
                    del self.nodes[action_name]
        new_nodes: List[VisualizationNode] = []
        for action in actions:
            graph_node = Node(str(action), style="filled", fillcolor="white")
            self.graph.add_node(graph_node)
            graph_edge: Optional[Edge] = Edge(predecessor, graph_node) if predecessor else None
            if graph_edge:
                self.graph.add_edge(graph_edge)
            new_node = self.nodes[str(action)] = VisualizationNode(graph_edge, graph_node, action)
            new_nodes.append(new_node)
            predecessor = graph_node
        # Reapply nodes kept after predecessor to visualize new nodes to the left of them.
        while nodes:
            action_name, node = nodes.pop(0)
            # Keep remaining part of main plan only.
            if action_name.split(' ', maxsplit=1)[0].isdigit():
                self.graph.add_node(node.node)
                if node.edge:
                    self.graph.add_edge(node.edge)
        self.update()

    def update_fillcolor(self, action: object, value: str) -> None:
        """Set fillcolor of action to value and update visualization."""
        self.nodes[str(action)].node.set("fillcolor", value)
        self.update()

    def execute(self, action: object) -> None:
        """Mark action as being executed."""
        action_node = self.nodes[str(action)]
        if action_node.edge:
            action_node.edge.set("color", "green")
        self.update_fillcolor(action, "yellow")

    def succeed(self, action: object) -> None:
        """Mark action as succeeded."""
        self.update_fillcolor(action, "green")

    def fail(self, action: object) -> None:
        """Mark action as failed."""
        self.update_fillcolor(action, "red")

    def cancel(self, action: object) -> None:
        """Mark cation as canceled."""
        self.update_fillcolor(action, "gray")

    def add_node(self, text: str, fillcolor: str) -> None:
        """Manually add an unconnected node with text and fillcolor into existing graph."""
        if self.graph:
            self.graph.add_node(Node(text, style="filled", fillcolor=fillcolor))
            self.update()
