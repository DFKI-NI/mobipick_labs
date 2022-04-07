from typing import List, Sequence
from threading import Thread
from xdot import DotWindow
from pydot import Dot, Edge, Node
import gi

gi.require_version('Gtk', '3.0')  # Note: Necessary, else a PyGIWarning appears.
from gi.repository import Gtk


class PlanVisualization:
    def __init__(self, actions: Sequence[str]) -> None:
        self.window = DotWindow()
        self.window.connect('delete-event', Gtk.main_quit)
        self.graph: Dot = None
        self.nodes: List[Node] = []
        self.edges: List[Edge] = []
        self.actions: Sequence[str] = []
        self.set_actions(actions)
        self.thread = Thread(target=Gtk.main)
        self.thread.start()

    def visualize(self) -> None:
        """Update the visualization by dot code from self.graph."""
        self.window.set_dotcode(str.encode(self.graph.to_string()))

    def set_actions(self, actions: Sequence[str]) -> None:
        """Create and visualize a new graph, built from actions."""
        self.actions = actions
        self.graph = Dot("Plan Visualization", graph_type="digraph", bgcolor="white")
        self.nodes = [Node(action, label=action, style="filled", fillcolor="white") for action in actions]
        for node in self.nodes:
            self.graph.add_node(node)
        self.edges = [Edge(node, self.nodes[index + 1]) for index, node in enumerate(self.nodes[:-1])]
        for edge in self.edges:
            self.graph.add_edge(edge)
        self.visualize()

    def execute(self, index: int) -> None:
        """Mark action at index as being executed."""
        self.nodes[index].set("fillcolor", "yellow")
        if index > 0:
            self.edges[index - 1].set("color", "green")
        self.visualize()

    def succeed(self, index: int) -> None:
        """Mark action at index as success."""
        self.nodes[index].set("fillcolor", "green")
        self.visualize()

    def fail(self, index: int) -> None:
        """Mark action at index as failure."""
        self.nodes[index].set("fillcolor", "red")
        self.visualize()
