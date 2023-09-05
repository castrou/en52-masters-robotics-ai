from typing import Literal as L, List, Tuple, Dict
import numpy as np

from queue import PriorityQueue
import math

# --------- Question 1.1 ---------- #


def ucs(
    adj: Dict[str, List[Tuple[str, float]]], start: str, goal: str
) -> Tuple[List[str], float]:
    """
    Finds optimal path in weighted graph using Uniform Cost Search (UCS)

    This method will find the optimal (minimum weight) path to the goal
    node in a weighted graph using UCS. The graph is represented as an
    Adjacency List. The path should start at the `start` node and end
    at the `goal` node, inclusive of `start` and `goal`.

    Notes
    -----
    - We use weight and cost interchangeably. They both refer to the
      weight/cost of traversing an edge in the graph.

    - `start` and `goal` will always be in `adj`, and there is always a
      viable path between them.

    Parameters
    ----------
    adj
        An Adjacency List which describes a weighted graph
    start
        The start node in the graph
    goal
        The goal node to find in the graph

    Returns
    -------
    path
        A list of nodes that describe the path from the start node to the goal
    cost
        The total cost of the path. This is the sum of the weights of each
        edge through `path`
    """

    queue = PriorityQueue()
    queue.put((0, start))
    visited = {start: 0}
    previous = {}

    while not queue.empty():
        current_cost, current_node = queue.get()

        if current_node == goal:
            break

        for neighbor, weight in adj[current_node]:
            cost = current_cost + weight

            if neighbor not in visited or cost < visited[neighbor]:
                visited[neighbor] = cost
                previous[neighbor] = current_node
                queue.put((cost, neighbor))

    # create path
    path = [goal]
    while path[-1] != start:
        path.append(previous[path[-1]])
    path.reverse()

    # Calculate cost
    cost = visited[goal]

    return path, cost


# --------- Question 2.1 ---------- #


def a_star(
    adj: Dict[str, List[Tuple[str, float]]],
    coords: Dict[str, Tuple[float, float, float]],
    start: str,
    goal: str,
) -> Tuple[List[str], float]:
    """
    Finds path in weighted graph using A*

    This method will find the path to the goal node in a weighted
    graph using A*. The graph is represented as an Adjacency List. The
    path should start at the `start` node and end at the `goal` node,
    inclusive of `start` and `goal`.

    Notes
    -----
    - A* search requires a heuristic. Consider the scenario provided to
      you above to calculate an appropriate heuristic that will result
      in A* giving the minimum cost path to the goal node.

    - We use weight and cost interchangeably. They both refer to the
      weight/cost of traversing an edge in the graph.

    - `start` and `goal` will always be in `adj`, and there is always a
      viable path between them.

    Parameters
    ----------
    adj
        An Adjacency List which describes a weighted graph
    coords
        A dictionary of 3D coordinates representing the position of
        each node in the graph
    start
        The start node in the graph
    goal
        The goal node to find in the graph

    Returns
    -------
    path
        A list of nodes that describe the path from the start node to the goal
    cost
        The total cost of the path. This is the sum of the weights of each
        edge through `path`
    """

    # Calculate the heuristic (h_score) using Euclidean distance between nodes and the goal node
    def heuristic(node):
        x1, y1, z1 = coords[node]
        x2, y2, z2 = coords[goal]
        return math.sqrt((x2 - x1)**2 + (y2 - y1)**2 + (z2 - z1)**2)

    queue = PriorityQueue()
    queue.put((0, start))
    previous = {}

    g_scores = {node: math.inf for node in adj}
    f_scores = {node: math.inf for node in adj}

    g_scores[start] = 0
    f_scores[start] = heuristic(start)

    while not queue.empty():
        # Get the node with the lowest f_score from the queue
        current_fscore, current_node = queue.get()

        # Check if we have reached the goal node
        if current_node == goal:
            break

        # Iterate over the neighbors of the current node
        for neighbor, weight in adj[current_node]:
            # Calculate the tentative g_score for the neighbor
            tentative_gscore = g_scores[current_node] + weight

            # Check if the tentative g_score is lower than the current g_score
            if tentative_gscore < g_scores[neighbor]:
                # Update the g_score and f_score for the neighbor
                g_scores[neighbor] = tentative_gscore
                f_scores[neighbor] = tentative_gscore + heuristic(neighbor)

                queue.put((f_scores[neighbor], neighbor))
                previous[neighbor] = current_node

    # create path
    path = []
    current_node = goal
    while current_node != start:
        path.append(current_node)
        current_node = previous[current_node]
    path.append(start)
    path.reverse()

    # Calculate cost
    cost = g_scores[goal]

    return path, cost
