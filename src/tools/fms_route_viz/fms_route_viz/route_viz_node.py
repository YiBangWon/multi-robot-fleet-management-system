"""Visualize the FMS route table as a layered dependency graph."""

from dataclasses import dataclass
from typing import Dict, Iterable, List, Tuple

import matplotlib.pyplot as plt
import networkx as nx
import rclpy
from rclpy.node import Node

from fms_msgs.msg import RouteTable


@dataclass(frozen=True)
class RouteNodeRecord:
    """Compact view of a route-table node used by the visualizer."""

    robot_id: int
    node_end_id: int
    node_start_id: int
    timestep: int
    state: int
    removed: bool
    is_initial_node: bool
    is_goal: bool
    is_waypoint: bool


STATE_STYLE = {
    0: ("staged", "orange"),
    1: ("queued", "orange"),
    2: ("in_progress", "green"),
    3: ("finished", "cyan"),
    5: ("occupied", "yellow"),
}


class RouteTableVisualizer(Node):
    """Subscribe to the shared route table topic and render it with NetworkX."""

    def __init__(self) -> None:
        super().__init__("route_table_visualizer")
        self.create_subscription(
            RouteTable,
            "/fms/route_tables",
            self._handle_route_table,
            20,
        )
        plt.ion()
        self._figure = plt.figure("fms_route_table", figsize=(18, 10))

    def _handle_route_table(self, message: RouteTable) -> None:
        layered_routes = self._extract_layered_routes(message)
        if not layered_routes:
            self.get_logger().warning("Received an empty route table.")
            return

        self._render_graph(layered_routes)
        self.get_logger().info("Updated route-table visualization.")

    def _extract_layered_routes(
        self, message: RouteTable
    ) -> List[List[Tuple[RouteNodeRecord, List[RouteNodeRecord]]]]:
        layered_routes: List[List[Tuple[RouteNodeRecord, List[RouteNodeRecord]]]] = []

        for route_table in message.route_table:
            agent_records: List[Tuple[RouteNodeRecord, List[RouteNodeRecord]]] = []

            for agent_route_table in route_table.agent_route_table:
                route_node = agent_route_table.nodert
                node_record = RouteNodeRecord(
                    robot_id=route_node.robot_id,
                    node_end_id=route_node.id_end,
                    node_start_id=route_node.id_start,
                    timestep=route_node.timestep,
                    state=route_node.state,
                    removed=route_node.remove,
                    is_initial_node=route_node.init_node,
                    is_goal=route_node.is_goal,
                    is_waypoint=route_node.is_waypoint,
                )

                preconditions = [
                    RouteNodeRecord(
                        robot_id=precondition.robot_id,
                        node_end_id=precondition.id_end,
                        node_start_id=precondition.id_start,
                        timestep=precondition.timestep,
                        state=precondition.state,
                        removed=precondition.remove,
                        is_initial_node=precondition.init_node,
                        is_goal=precondition.is_goal,
                        is_waypoint=precondition.is_waypoint,
                    )
                    for precondition in agent_route_table.preconditions
                ]
                agent_records.append((node_record, preconditions))

            layered_routes.append(agent_records)

        return layered_routes

    def _render_graph(
        self, layered_routes: List[List[Tuple[RouteNodeRecord, List[RouteNodeRecord]]]]
    ) -> None:
        graph = nx.DiGraph()
        layers = self._build_layers(layered_routes)
        node_labels: Dict[str, str] = {}
        node_colors: List[str] = []

        for layer_index, agent_records in enumerate(layered_routes):
            for column_index, (node_record, _) in enumerate(agent_records):
                node_name = f"v{layer_index}.{column_index}"
                state_label, color = STATE_STYLE.get(node_record.state, ("dummy", "red"))
                graph.add_node(node_name, layer=layer_index)
                node_labels[node_name] = (
                    f"{node_name}\n"
                    f"start={node_record.node_start_id}\n"
                    f"end={node_record.node_end_id}\n"
                    f"{state_label}\n"
                    f"waypoint={node_record.is_waypoint}"
                )
                node_colors.append(color)

        same_layer_edges = self._build_same_layer_edges(layers)
        precondition_edges = self._build_precondition_edges(layered_routes, layers)
        graph.add_edges_from(same_layer_edges)
        graph.add_edges_from(precondition_edges)

        positions = self._build_positions(layers)
        self._figure.clf()

        nx.draw_networkx_nodes(
            graph,
            positions,
            node_size=3400,
            node_color=node_colors,
            alpha=0.9,
        )
        nx.draw_networkx_edges(
            graph,
            positions,
            edgelist=same_layer_edges,
            edge_color="green",
            arrowsize=10,
            alpha=0.7,
        )
        nx.draw_networkx_edges(
            graph,
            positions,
            edgelist=precondition_edges,
            edge_color="blue",
            arrowsize=10,
            alpha=0.7,
            style="dashed",
        )
        nx.draw_networkx_labels(
            graph,
            positions,
            labels=node_labels,
            font_size=8,
            font_color="black",
            verticalalignment="center",
        )

        plt.axis("off")
        plt.title("Route-Table Dependency Graph")
        plt.tight_layout()
        plt.pause(0.1)

    @staticmethod
    def _build_layers(
        layered_routes: List[List[Tuple[RouteNodeRecord, List[RouteNodeRecord]]]]
    ) -> Dict[int, List[str]]:
        return {
            layer_index: [
                f"v{layer_index}.{column_index}"
                for column_index in range(len(agent_records))
            ]
            for layer_index, agent_records in enumerate(layered_routes)
        }

    @staticmethod
    def _build_same_layer_edges(layers: Dict[int, List[str]]) -> List[Tuple[str, str]]:
        edges: List[Tuple[str, str]] = []
        for nodes in layers.values():
            edges.extend((nodes[index], nodes[index + 1]) for index in range(len(nodes) - 1))
        return edges

    @staticmethod
    def _build_precondition_edges(
        layered_routes: List[List[Tuple[RouteNodeRecord, List[RouteNodeRecord]]]],
        layers: Dict[int, List[str]],
    ) -> List[Tuple[str, str]]:
        edges: List[Tuple[str, str]] = []

        for agent_records in layered_routes:
            for node_record, preconditions in agent_records:
                target_node = layers[node_record.robot_id][node_record.timestep]
                for precondition in preconditions:
                    source_node = layers[precondition.robot_id][precondition.timestep]
                    edges.append((source_node, target_node))

        return edges

    @staticmethod
    def _build_positions(layers: Dict[int, List[str]]) -> Dict[str, Tuple[float, float]]:
        positions: Dict[str, Tuple[float, float]] = {}
        layer_gap = 2.0
        node_gap = 8.0

        for layer_index, nodes in layers.items():
            y_position = layer_index * layer_gap
            for column_index, node_name in enumerate(nodes):
                positions[node_name] = (column_index * node_gap, y_position)

        return positions


def main(args: Iterable[str] | None = None) -> None:
    rclpy.init(args=args)
    node = RouteTableVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
