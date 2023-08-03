from models import Environment,UAV
from tqdm import tqdm
import os
import rospy

if __name__ == "__main__":

    # Example usage
    size = 50
    grid_resolution = 1 
    drone_count = 16
    weed_centers = [[-size/4, -size/4], [size/4, size/4]]
    weed_cov = [[5, 0], [0, 5]]
    iterations = 5
    sampling_time = 9

    EXPERIMENT_LOGGING_DIR = '/home/invisible23man/Robotics/Simulations/decentralised-adaptive-coverage-with-voronoi-partitioning/src/decentralised_adaptive_coverage/outputs/experiment_logging'
    EXPERIMENT_TIMESTAMP = ''
    EXPERIMENT_FILENAME = os.path.join(EXPERIMENT_LOGGING_DIR,EXPERIMENT_TIMESTAMP,'data.pkl')
    ANIMATION2D_FILENAME = os.path.join(EXPERIMENT_LOGGING_DIR,EXPERIMENT_TIMESTAMP,'animation2d.gif')
    ANIMATION3D_FILENAME = os.path.join(EXPERIMENT_LOGGING_DIR,EXPERIMENT_TIMESTAMP,'animation3d.gif')

    field = Environment.Field(size, grid_resolution, drone_count, weed_centers, weed_cov, sampling_time)
    field.plot_field()

    drones = [UAV.Drone(pos, field, id) for id,pos in enumerate(field.drone_positions)]

    for iteration in tqdm(range(iterations)):
        print(f"\nIteration {iteration+1}")

        for i, drone in enumerate(drones):
            drone.compute_voronoi(plot=False)
            drone.plan(plot=False)
            # print(f"Drone {i+1} Path Length: {len(drone.lawnmower_path)}")

            if len(drone.lawnmower_path) == 0:
                drone.voronoi_center_tracker.append(drone.position)
                drone.measurements = []
                field.drone_positions[i] = drone.position
                continue   

            drone.sense()
            # print(f"Drone {i+1} Measurements: {drone.measurements[:10]}")

            drone.estimate()

            drone.update_voronoi()
            # print(f"Drone {i+1} Centers: {drone.voronoi_center_tracker}")

            # Drone Posiiton update to be moved ot outer loop for asynchronous update later
            field.drone_positions[i] = drone.position

        # Update the drone positions and path measurements in the field
        field.update_drone_positions([drone.position for drone in drones])
        field.update_path_and_measurements(drones)

        # field.plot_field()

    # Save data
    field.save_data(EXPERIMENT_FILENAME)

    field.animate_field_2d(ANIMATION2D_FILENAME)
    field.animate_field_3d(ANIMATION3D_FILENAME)
