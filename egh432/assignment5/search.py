from typing import Literal as L, List, Tuple, Dict
from collections import deque
import numpy as np


# --------- Question 1.1 ---------- #


def dfs_tree(tree_adj: Dict[str, List[str]], start: str, goal: str) -> List[str]:
    """
    Finds goal node in a tree using depth first search

    This method will find the goal node in a tree using depth first search. The
    tree is represented as a Tree Adjacency List. The path should start at the
    root node and end at the goal node.

    When going down the tree, the algorithm should choose the first child node
    in the adjacency list that has not been visited yet.

    Parameters
    ----------pass
    tree_adj
        A Tree Adjacency List which describes a tree
    start
        The root node of the tree
    goal
        The goal node to find in the tree

    Returns
    -------
    path
        A list of nodes that describe the path from the root node to the goal

    """
    visited = set()
    stack = deque([(start, [start])])

    while stack:
        current_node, path = stack.pop()

        if current_node == goal:
            return path  # Return the path if the end node is reached

        if current_node not in visited:
            visited.add(current_node)

            if current_node in tree_adj:
                neighbors = tree_adj[current_node]
                for neighbor in neighbors[::-1]:
                    stack.append((neighbor, path + [neighbor]))

    return None  # Return None if no path is found

# --------- Question 2.1 ---------- #


def bfs_tree(tree_adj: Dict[str, List[str]], start: str, goal: str) -> List[str]:
    """
    Finds goal node in a tree using breadth first search

    This method will find the goal node in a tree using breadth first search. The
    tree is represented as a Tree Adjacency List. The path should start at the
    root node and end at the goal node.

    When going through the graph, the algorithm should add unvisited nodes to
    the frontier in the order they appear in the adjacency list.

    Parameters
    ----------
    tree_adj
        A Tree Adjacency List which describes a tree
    start
        The root node of the tree
    goal
        The goal node to find in the tree

    Returns
    -------
    path
        A list of nodes that describe the path from the root node to the goal

    """

    visited = set()
    stack = deque([(start, [start])])

    while stack:
        current_node, path = stack.popleft()

        if current_node == goal:
            return path  # Return the path if the end node is reached

        if current_node not in visited:
            visited.add(current_node)

            if current_node in tree_adj:
                neighbors = tree_adj[current_node]
                for neighbor in neighbors[::-1]:
                    stack.append((neighbor, path + [neighbor]))

    return None  # Return None if no path is found



# --------- Question 2.2 ---------- #


def bfs_graph(adj: Dict[str, List[str]], start: str, goal: str) -> List[str]:
    """
    Finds goal node in a graph using bfs first search

    This method will find the goal node in a graph using bfs first search. The
    graph is represented as a Adjacency List. The path should start at the
    `start` node and end at the `goal` node.

    When going through the graph, the algorithm should add unvisited nodes to
    the frontier in the order they appear in the adjacency list.

    Parameters
    ----------
    adj
        An Adjacency List which describes a graph
    start
        The start node in the graph
    goal
        The goal node to find in the graph

    Returns
    -------
    path
        A list of nodes that describe the path from the root node to the goal
    """

    visited = set()
    stack = deque([(start, [start])])

    while stack:
        current_node, path = stack.popleft()

        if current_node == goal:
            return path  # Return the path if the end node is reached

        if current_node not in visited:
            visited.add(current_node)

            if current_node in adj:
                neighbors = adj[current_node]
                for neighbor in neighbors[::-1]:
                    stack.append((neighbor, path + [neighbor]))

    return None  # Return None if no path is found