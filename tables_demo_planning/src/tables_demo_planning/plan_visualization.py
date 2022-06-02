from typing import List, Sequence
from pydot import Dot, Edge, Node
from std_msgs.msg import String
import rospy


class PlanVisualization:
    def __init__(self, actions: Sequence[object]) -> None:
        self.graph: Dot = None
        self.nodes: List[Node] = []
        self.edges: List[Edge] = []
        self.actions: Sequence[object] = []
        self.plan_pub = rospy.Publisher("/plan_visualization/plan_graph", String, queue_size=1)
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
        self.edges = [Edge(node, self.nodes[index + 1]) for index, node in enumerate(self.nodes[:-1])]
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
