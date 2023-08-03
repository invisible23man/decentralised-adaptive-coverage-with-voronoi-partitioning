from scipy.interpolate import griddata

def sense_field(coordinate, grid_points, weed_distribution, method='nearest'):
    """
    Function to sense the weed density at a given coordinate in the field.
    Args:
    coordinate : Coordinate at which the weed density needs to be sensed.
    grid_points : The grid points of the field.
    weed_distribution : The corresponding weed distribution at each grid point.
    method : The method to be used for interpolation. Default is 'nearest'.
    
    Returns:
    sensed_density : The sensed weed density at the given coordinate.
    """
    # Use the scipy griddata function for interpolation
    sensed_density = griddata(grid_points, weed_distribution, coordinate, method=method)
    
    return sensed_density

if __name__ == '__main__':

    import sys
    sys.path.append(r'/home/invisible23man/Robotics/Simulations/decentralised-adaptive-coverage-with-voronoi-partitioning/src/decentralised_adaptive_coverage/scripts/models')
    from Environment import Field

    # Example usage
    size = 50
    grid_resolution = 1
    drone_count = 16
    weed_centers = [[-15, -15], [15, 15]]
    weed_cov = [[5, 0], [0, 5]]

    field = Field(size, grid_resolution, drone_count, weed_centers, weed_cov)
    field.plot_field()
 
    # Now, let's sense the weed density at a few points
    points_to_sense = [[-12, -12], [0, 0], [10, 10], [-10, -10], [15, 15] , [-15, -15]]
    sensed_densities = [sense_field(point, field.grid_points, field.weed_distribution) for point in points_to_sense]

    print(sensed_densities)
