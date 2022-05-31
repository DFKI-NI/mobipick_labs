from typing import Dict, List, Optional, Sequence, Set
from dataclasses import dataclass
from threading import Thread
from xdot import DotWindow
from pydot import Dot, Edge, Node
import gi

gi.require_version('Gtk', '3.0')  # Note: Necessary, else a PyGIWarning appears.
from gi.repository import Gtk


@dataclass
class VisualizationNode:
    edge: Optional[Edge]  # Note: incoming edge
    node: Node
    action: object


class SubPlanVisualization:
    def __init__(self) -> None:
        # Note: Set base_title as workaround because self.window.set_title() has no effect.
        DotWindow.base_title = "Plan Visualization"
        self.window = DotWindow()
        self.window.connect('delete-event', Gtk.main_quit)
        self.graph: Dot = None
        self.nodes: Dict[str, VisualizationNode] = {}
        self.thread = Thread(target=Gtk.main)
        self.thread.start()

    def update(self) -> None:
        """Update visualization by dot code from self.graph."""
        self.window.set_dotcode(str.encode(self.graph.to_string()))

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
