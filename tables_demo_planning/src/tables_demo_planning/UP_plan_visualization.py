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
# Authors: Marc Vinci - DFKI

"""
Helper component which maintains the dot graph of a demo specific hierarchical plan,
visualized by the dot_graph_visualization repository.
"""


from typing import List, Dict, Union, Set, Callable, Sequence, Optional, Tuple
from dataclasses import dataclass
from pydot import Dot, Edge, Node
from unified_planning.plans import Plan, SequentialPlan, TimeTriggeredPlan, HierarchicalPlan, PlanKind, ActionInstance
from unified_planning.plans.hierarchical_plan import Decomposition, MethodInstance
from std_msgs.msg import String
import rospy


@dataclass
class VisualizationNode:
    edge_in: Optional[Edge]  # incoming edge (only one possible)
    edges_out: List[Edge]  # outgoing edges
    node: Node
    action: object
    action_str: str


class UPPlanVisualization:
    def __init__(self) -> None:
        self.graph: Dot = None
        self.plan_pub = rospy.Publisher("/dot_graph_visualization/dot_graph", String, queue_size=1)
        self.nodes: Dict[Union[MethodInstance, ActionInstance], VisualizationNode] = {}
        self.node_name_to_vis_node_map: Dict[str, VisualizationNode] = {}

        self.executed_actions: Set[ActionInstance] = set()

        # Readable labels for parameters
        self.parameter_labels: Dict[str, str] = {
            "base_home_pose": "home",
            "base_handover_pose": "handover",
            "base_pick_pose": "pick",
            "base_place_pose": "place",
            "base_table_1_pose": "table_1",
            "base_table_2_pose": "table_2",
            "base_table_3_pose": "table_3",
            "tool_search_pose": "where tool has been found",
            "klt_search_pose": "where KLT has been found",
        }

        # Readable string for each action to visualize
        self.action_labels: Dict[str, Callable[[Sequence[str]], str]] = {
            "move_base": lambda parameters: f"move_base({parameters[-1]})",
            "move_base_with_item": lambda parameters: f"move_base_with_item({parameters[1]}, {parameters[-1]})",
            "move_arm": lambda parameters: f"move_arm({parameters[-1]})",
            "pick_item": lambda parameters: f"pick_item({parameters[-1]})",
            "place_item": lambda parameters: f"place_item({parameters[-1]}, {parameters[-2]})",
            "store_item": lambda parameters: f"store_item({parameters[-2]}, {parameters[-1]})",
            "hand_over_item": lambda parameters: f"hand_over_item({parameters[-1]})",
            "search_at": lambda parameters: f"search_at({parameters[-1]})",
            "search_tool": lambda parameters: f"search_tool({parameters[-1]})",
            "search_klt": lambda parameters: f"search_klt({parameters[-1]})",
            "conclude_tool_search": lambda parameters: f"conclude_tool_search({parameters[-1]})",
            "conclude_klt_search": lambda parameters: f"conclude_tool_search({parameters[-1]})",
        }

    def label(self, action: ActionInstance) -> str:
        """Return a user-friendly label for visualizing action names and parameters."""
        parameters = [parameter.object() for parameter in action.actual_parameters]
        parameter_labels = [self.parameter_labels.get(parameter.name, str(parameter)) for parameter in parameters]
        return self.action_labels[action.action.name](parameter_labels)

    def visualize(self) -> None:
        """Update the visualization by dot code from self.graph."""
        self.plan_pub.publish(self.graph.to_string())

    def set_plan(self, plan: Union[Plan, SequentialPlan, TimeTriggeredPlan, HierarchicalPlan]) -> None:
        """Update graph with actions from the UP plan."""
        self.executed_actions: Set[ActionInstance] = set()
        if plan.kind == PlanKind.SEQUENTIAL_PLAN:
            self.create_sequential_plan_graph(plan)
        elif plan.kind == PlanKind.TIME_TRIGGERED_PLAN:
            self.create_time_triggered_plan_graph(plan)
        elif plan.kind == PlanKind.HIERARCHICAL_PLAN:
            self.create_hierarchical_plan_graph(plan)
        self.visualize()

    def set_subplan(
        self, plan: Union[Plan, SequentialPlan, TimeTriggeredPlan, HierarchicalPlan], predecessor: ActionInstance
    ) -> None:
        if plan.kind == PlanKind.SEQUENTIAL_PLAN:
            self.add_sequential_subplan(plan, predecessor)
        elif plan.kind == PlanKind.TIME_TRIGGERED_PLAN:
            self.add_time_triggered_subplan(plan, predecessor)
        elif plan.kind == PlanKind.HIERARCHICAL_PLAN:
            self.create_hierarchical_plan_graph(plan, predecessor)
        self.visualize()

    def create_sequential_plan_graph(self, plan: SequentialPlan) -> None:
        self.graph = Dot("Plan", graph_type="digraph", bgcolor="white")

        action_strings = [
            f"{number + len(self.executed_actions)} {self.label(action)}"
            for number, action in enumerate(plan.actions, start=1)
        ]
        predecessor_node = None

        for node in list(self.nodes.values()):
            if node.action in self.executed_actions:
                self.graph.add_node(node.node)
                if node.edge_in:
                    self.graph.add_edge(node.edge_in)  # always only
                predecessor_node = node
            else:
                del self.nodes[node.action]
                del self.node_name_to_vis_node_map[node.action_str]

        for action_str, action in zip(action_strings, plan.actions):
            graph_node = Node(action_str, style="filled", fillcolor="white")
            self.graph.add_node(graph_node)
            graph_edge: Optional[Edge] = Edge(predecessor_node.action_str, graph_node) if predecessor_node else None
            if graph_edge:
                self.graph.add_edge(graph_edge)

            vis_node = VisualizationNode(graph_edge, [], graph_node, action, action_str)
            if predecessor_node:
                predecessor_node.edges_out.append(graph_edge)
            self.nodes[action] = vis_node
            self.node_name_to_vis_node_map[action_str] = vis_node
            predecessor_node = vis_node

    def add_sequential_subplan(self, plan: SequentialPlan, predecessor: ActionInstance) -> None:
        self.graph = Dot("Plan", graph_type="digraph", bgcolor="white")
        nodes: List[Tuple[ActionInstance, VisualizationNode]] = []
        # Keep nodes up to predecessor and append rest after adding new nodes.
        action_strings = [
            f"{len(self.executed_actions)}{chr(number)} {self.label(action)}"
            for number, action in enumerate(plan.actions, start=97)
        ]
        predecessor_node = self.nodes[predecessor]
        nodes.extend(self.nodes.items())
        while nodes:
            action, node = nodes.pop(0)
            # Keep main plan only.
            if node.action_str.split(' ', maxsplit=1)[0].isdigit():
                self.graph.add_node(node.node)
                if node.edge_in:
                    self.graph.add_edge(node.edge_in)
            if action == predecessor:
                break

        for action_str, action in zip(action_strings, plan.actions):
            graph_node = Node(action_str, style="filled", fillcolor="white")
            self.graph.add_node(graph_node)
            graph_edge = Edge(predecessor_node.action_str, graph_node)
            self.graph.add_edge(graph_edge)

            vis_node = VisualizationNode(graph_edge, [], graph_node, action, action_str)
            if predecessor_node:
                predecessor_node.edges_out.append(graph_edge)
            self.nodes[action] = vis_node
            self.node_name_to_vis_node_map[action_str] = vis_node
            predecessor_node = vis_node

        while nodes:
            action, node = nodes.pop(0)
            # Keep remaining part of main plan only.
            if self.nodes[action].action_str.split(' ', maxsplit=1)[0].isdigit():
                self.graph.add_node(node.node)
                if node.edge_in:
                    self.graph.add_edge(node.edge_in)

    def create_time_triggered_plan_graph(self, plan: TimeTriggeredPlan) -> None:
        pass

    def add_time_triggered_subplan(self, plan: TimeTriggeredPlan, predecessor: ActionInstance) -> None:
        pass

    # Recurse through the root UP Decomposition and MethodInstances until the given leaf ActionInstance is found
    # return the path from the root to the leaf as a list
    def UP_decomposition_recursion(self, dec, action):
        if isinstance(dec, Decomposition):
            for _, d in dec.subtasks.items():
                d_end = None
                if isinstance(d, MethodInstance):
                    d_end = self.UP_decomposition_recursion(d.decomposition, action)
                else:
                    if d == action:
                        return [d]
                if d_end is not None:
                    return [d] + d_end

    def create_hierarchical_plan_graph(self, plan: HierarchicalPlan, predecessor: ActionInstance = None) -> None:
        if predecessor is None or self.graph is None:
            self.graph = Dot("Plan", graph_type="digraph", bgcolor="white")

        root_to_leaf_list = []
        # Keep track of nodes and edges already added to graph
        unique_node_list = []
        unique_edge_list = []

        for a in plan.action_plan.actions:
            # find root of each action in the plan
            # UP decomposition does not contain the action ordering
            root_to_leaf_list.append(self.UP_decomposition_recursion(plan.decomposition, a))

        node_id = len(self.nodes) + 1
        for action_path in root_to_leaf_list:
            predecessor_node = self.nodes.get(predecessor, None)
            for method_or_action in action_path:
                if isinstance(method_or_action, MethodInstance):
                    node_label = method_or_action.method.name + str(method_or_action.parameters)
                    node_name = str(node_id) + " " + node_label
                    graph_node = Node(node_name, label=node_label, style="filled", fillcolor="white", shape="box")
                else:
                    node_label = self.label(method_or_action)
                    node_name = str(node_id) + " " + node_label
                    graph_node = Node(node_name, label=node_label, style="filled", fillcolor="white")

                if method_or_action not in unique_node_list:
                    self.graph.add_node(graph_node)
                    unique_node_list.append(method_or_action)
                    node_id += 1
                graph_edge: Optional[Edge] = Edge(predecessor_node.node, graph_node) if predecessor_node else None
                if graph_edge and (predecessor_node.action, method_or_action) not in unique_edge_list:
                    self.graph.add_edge(graph_edge)
                    unique_edge_list.append((predecessor_node.action, method_or_action))
                vis_node = self.nodes.get(method_or_action, None)
                if vis_node is None:
                    vis_node = VisualizationNode(graph_edge, [], graph_node, method_or_action, node_name)
                    if predecessor_node:
                        predecessor_node.edges_out.append(graph_edge)
                    self.nodes[method_or_action] = vis_node
                    self.node_name_to_vis_node_map[node_name] = vis_node
                predecessor_node = vis_node

    def update_fillcolor(self, action: ActionInstance, value: str) -> None:
        """Set fillcolor of action to value and update visualization."""
        self.nodes[action].node.set("fillcolor", value)
        self.visualize()

    def update_node_hierarchy(self, node: VisualizationNode, value: str, finished: bool = False) -> None:
        if node is not None and node.edge_in:
            node.edge_in.set("color", value)
            src_node = self.node_name_to_vis_node_map.get(node.edge_in.get_source()[1:-1], None)
            if src_node is not None and isinstance(src_node.action, MethodInstance):
                edges_finished = [edge.get("color") == value for edge in src_node.edges_out] if finished else [True]
                if all(edges_finished):
                    src_node.node.set("fillcolor", value)
                    self.update_node_hierarchy(src_node, value, finished)

    def execute(self, action: ActionInstance) -> None:
        vis_node = self.nodes.get(action, None)
        if vis_node is not None and vis_node.action_str.split(' ', maxsplit=1)[0].isdigit():
            self.executed_actions.add(action)
        self.update_node_hierarchy(vis_node, "yellow")
        self.update_fillcolor(action, "yellow")

    def succeed(self, action: ActionInstance) -> None:
        """Mark action as succeeded."""
        vis_node = self.nodes.get(action, None)
        self.update_node_hierarchy(vis_node, "green", True)
        self.update_fillcolor(action, "green")

    def fail(self, action: ActionInstance) -> None:
        """Mark action as failed."""
        vis_node = self.nodes.get(action, None)
        self.update_node_hierarchy(vis_node, "red", True)
        self.update_fillcolor(action, "red")

    def cancel(self, action: ActionInstance) -> None:
        """Mark cation as canceled."""
        vis_node = self.nodes.get(action, None)
        self.update_node_hierarchy(vis_node, "gray", True)
        self.update_fillcolor(action, "gray")

    def add_node(self, text: str, fillcolor: str) -> None:
        """Manually add an unconnected node with text and fillcolor into existing graph."""
        if self.graph:
            self.graph.add_node(Node(text, style="filled", fillcolor=fillcolor))
            self.visualize()
