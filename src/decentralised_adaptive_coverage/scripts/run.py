from models import Environment,UAV
from tqdm import tqdm

if __name__ == "__main__":

    # Example usage
    size = 50
    grid_resolution = 1 
    drone_count = 16
    weed_centers = [[-size/4, -size/4], [size/4, size/4]]
    weed_cov = [[5, 0], [0, 5]]
    iterations = 10

    field = Environment.Field(size, grid_resolution, drone_count, weed_centers, weed_cov)
    field.plot_field()

    drones = [UAV.Drone(pos, field, id) for id,pos in enumerate(field.drone_positions)]

    for iteration in tqdm(range(iterations)):
        print(f"\nIteration {iteration+1}")

        for i, drone in enumerate(drones):
            drone.compute_voronoi(plot=False)
            drone.plan(plot=False)
            # print(f"Drone {i+1} Path Length: {len(drone.lawnmower_path)}")

            drone.sense()
            # print(f"Drone {i+1} Measurements: {drone.measurements[:10]}")

            drone.update_voronoi()
            # print(f"Drone {i+1} Centers: {drone.voronoi_center_tracker}")

            # Update the drone positions in the field
            field.drone_positions[i] = drone.position

        field.plot_field()
