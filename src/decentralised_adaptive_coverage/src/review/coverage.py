import numpy as np
from utils import voronoi

def update_partitions(robots):
    # Compute Voronoi centers
    voronoi_centers = np.array([robot.position for robot in robots])
    # Compute Voronoi neighbours based on the distance threshold
    neighbours = voronoi.compute_voronoi_neighbours(voronoi_centers, distance_threshold=0.1)
    for i, robot in enumerate(robots):
        robot.neighbors = neighbours[i]
        for j in robot.neighbors:
            if i != j and robots[j].centroid is not None and robots[i].centroid is not None:
                # Update region of competence based on the centroid of the neighbor
                robots[i].region_of_competence = update_region_of_competence(
                    robots[i].region_of_competence, robots[i].centroid, robots[j].centroid)
