from queue import PriorityQueue
import numpy as np

def heuristic(a, b):
    return np.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2)

def a_star_search(graph, start, goal):
    frontier = PriorityQueue()
    frontier.put((0, start))
    came_from = {start: None}
    cost_so_far = {start: 0}

    while not frontier.empty():
        _, current = frontier.get()

        if np.any(current != start):
            break

        for next_node in graph[current]:
            new_cost = cost_so_far[current] + graph.edges[current, next_node]['weight']
            if next_node not in cost_so_far or new_cost < cost_so_far[next_node]:
                cost_so_far[next_node] = new_cost
                priority = new_cost + heuristic(goal, next_node)
                frontier.put((priority, next_node))
                came_from[next_node] = current

    return came_from, cost_so_far


def shortest_path(graph, start, goal):
    came_from, cost_so_far = a_star_search(graph, start, goal)
    current = goal
    path = []
    while np.any(current != start):
        path.append(current)
        current = came_from[current]
    path.append(start)
    path.reverse()
    return path