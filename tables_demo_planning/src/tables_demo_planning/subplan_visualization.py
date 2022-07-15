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

    def add_node(self, text: str, fillcolor: str) -> None:
        if self.graph:
            self.graph.add_node(Node(text, style="filled", fillcolor=fillcolor))
            self.update()
