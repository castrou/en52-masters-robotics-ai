from typing import Literal as L, List, Tuple, Dict
from collections import deque
import numpy as np
from pgraph import *
import json
import random 

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


def tree_from_graph(g: UGraph):
    tree_adj = dict()
    for v in g:
        tree_adj[v.name] = v.neighbors()
    return tree_adj

def path_DFS(g: UGraph, start: str, goal: str):
        """
        Depth-first search
        
        :param S: start vertex
        :type S: Vertex subclass
        :param G: goal vertex
        :type G: Vertex subclass
        :return: list of vertices from S to G inclusive, path length
        :rtype: list of Vertex subclass, float

        Returns a list of vertices that form a path from vertex ``S`` to
        vertex ``G`` if possible, otherwise return None.

        """
        S = g[start]
        G = g[goal]
        
        # we use lists not sets since the order is instructive in verbose
        # mode, really need ordered sets...
        frontier = [S]
        explored = []
        parent = {}
        done = False

        while frontier:
            x = frontier.pop()

            # expand the vertex
            for n in x.neighbours():
                if n is G:
                    parent[n] = x
                    done = True
                    break
                if n not in frontier and n not in explored:
                    # add it to the frontier
                    frontier.append(n)
                    parent[n] = x
            if done:
                break
            explored.append(x)
        else:
            # no path
            return None

        # reconstruct the path from start to goal
        x = G
        path = [x]
        length = 0

        while x is not S:
            p = parent[x]
            length += x.edgeto(p).cost
            path.insert(0, p)
            x = p

        return path, length

def q1():
    with open('q1_pos.json', 'r') as f:
        places = json.loads(f.read())
        
    with open('q1_move.json', 'r') as f:
        routes = json.loads(f.read())
        
    g = UGraph()
    
    for name, coord in places.items():
        g.add_vertex(name=name, coord=[coord["x"], 6 - coord["y"]])
        
    for route in routes:
        g.add_edge(route[0], route[1], cost=route[2])
        
    p = g.path_BFS('S', 'G')
    p = path_DFS(g, 'S', 'G')
    g.highlight_path(p[0])  # overlay the path
    g.plot(block=True) # plot it
    
    
def random_sample_point():
    x = round(random.random() * 10, 1)
    y = round(random.random() * 10, 1)
    name = f'({x}, {y})'
    return name, [x, y]

def is_nearby(node1, node2):
    delta = node1.coord - node2.coord
    dist = np.sqrt(delta[0]**2 + delta[1]**2)
    if dist <= 2:
        print('new-connection')
        return True
    return False
    
def q2():
    with open('q2_vertices.json', 'r') as f:
        corners = json.loads(f.read())
        
    with open('q2_edges.json', 'r') as f:
        edges = json.loads(f.read())
        
    g = UGraph()
    
    for name, coord in corners.items():
        g.add_vertex(name=name, coord=[coord["x"], coord["y"]])
    for edge in edges:
        g.add_edge(edge[0], edge[1])
    
    not_solvable = True
    while not_solvable:
        name, coord = random_sample_point()
        if name not in g:
            g.add_vertex(name=name, coord=coord)
            new_node = g[name]
            for v in g:
                if is_nearby(new_node, v):
                    g.add_edge(new_node, v)
            p = g.path_Astar('S', 'G')
            if p:
                not_solvable = False
                break
            g.plot(block=False) # plot it
                
    p = g.path_Astar('S', 'G')
    g.highlight_path(p[0])  # overlay the path
    g.plot(block=True) # plot it

if __name__ == '__main__':
    # q1()
    q2()