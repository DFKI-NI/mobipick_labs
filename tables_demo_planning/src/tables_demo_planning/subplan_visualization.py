from typing import Dict, List, Optional, Sequence, Set
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
        if predecessor is None:
            self.graph = Dot("Plan", graph_type="digraph", bgcolor="white")
            if preserve_actions:
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
        self.update()

    def execute(self, action: object) -> None:
        """Mark action as being executed."""
        action_node = self.nodes[str(action)]
        action_node.node.set("fillcolor", "yellow")
        if action_node.edge:
            action_node.edge.set("color", "green")
        self.update()

    def succeed(self, action: object) -> None:
        """Mark action as successful."""
        self.nodes[str(action)].node.set("fillcolor", "green")
        self.update()

    def fail(self, action: object) -> None:
        """Mark action as failed."""
        self.nodes[str(action)].node.set("fillcolor", "red")
        self.update()
