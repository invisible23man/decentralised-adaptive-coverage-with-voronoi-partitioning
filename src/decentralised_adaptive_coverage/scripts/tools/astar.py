from queue import PriorityQueue
import numpy as np

def heuristic(original_start, current, goal):
    # Compute the Euclidean distance between the current and the goal
    euclidean_distance = np.sqrt((goal[0] - current[0]) ** 2 + (goal[1] - current[1]) ** 2)
    return euclidean_distance

def a_star_search(graph, original_start, current, goal):
    frontier = PriorityQueue()
    frontier.put((0, current))
    came_from = {current: None}
    cost_so_far = {current: 0}

    while not frontier.empty():
        _, current = frontier.get()

        if np.any(current == goal):
            break

        for next_node in graph[current]:
            new_cost = cost_so_far[current] + graph.edges[current, next_node]['weight']
            if next_node not in cost_so_far or new_cost < cost_so_far[next_node]:
                cost_so_far[next_node] = new_cost
                priority = new_cost + heuristic(original_start, current, next_node)
                frontier.put((priority, next_node))
                came_from[next_node] = current

    if frontier.empty():
        raise Exception("No path found from start to goal.")

    return came_from, cost_so_far


def shortest_path(graph, original_start, current, goal):
    came_from, cost_so_far = a_star_search(graph, original_start, current, goal)
    path = []
    while np.any(current != goal):
        if current not in came_from:
            print(f"Error: trying to find a path to a node that was not reached during the search: {current}")
            break
        path.append(current)
        current = came_from[current]
    path.append(goal)
    path.reverse()
    return path


