#! /home/invisibleman/anaconda3/envs/drones/bin/python
import rospy
from models import Robots
import configparser
import time
from tqdm import tqdm
import pickle
import os

def main():
    # Initializing ROS node.
    rospy.init_node("drone_controller" + rospy.get_namespace().replace('/', ''), anonymous=True)

    # Load configuration
    config = configparser.ConfigParser()
    config.read(
        '/home/invisibleman/Robotics/adaptive-coverage-with-voronoi/src/decentralised_adaptive_coverage/scripts/config.ini')
    n_iter = config.getint('ITERATIVE_PROCESS', 'n_iterations')

    # Initlialize the Drone
    drone = Robots.Drone(config)

    # Wait until the drone is ready
    time.sleep(3)
    # while not drone.ready and not rospy.is_shutdown():
    #     rospy.loginfo("Waiting for initial messages...")
    #     time.sleep(1)


    # Now start the main loop
    # while not rospy.is_shutdown():
    for i in tqdm(range(n_iter)):

        rospy.loginfo(f"Started movement and sensing:drone{drone.drone_id}, iter:{i}")
        drone.move_and_sense()

        drone.initialize_kalman() # One Time
        
        rospy.loginfo(f"Started Kalman Update:drone{drone.drone_id}, iter:{i}")
        drone.update()

        rospy.loginfo(f"Calculating New Center:drone{drone.drone_id}, iter:{i}")
        drone.calculate_new_voronoi_center()

        # Wait until all drones have updated their centers
        while len(drone.other_centers) != drone.config.getint('INITIAL_SETUP','n_drones') - 1:
            rospy.loginfo(f"Waiting for other drones to update their centers:drone{drone.drone_id}, o_centers:{len(drone.other_centers)},iter:{i}")
            time.sleep(1)  # Sleep for a short time to avoid busy waiting

        rospy.loginfo(f"Calculating Voronoi Partitions:drone{drone.drone_id}, iter:{i}")
        drone.calculate_voronoi_partitions()
        rospy.loginfo(f"Voronoi Centres for drone{drone.drone_id}: {drone.voronoi_center_tracker}")
        
    # Save the run info
    with open(os.path.join(config.get('RESULTS', 'save_directory'), f"{drone.drone_id}_centers.pkl"), "wb") as f:
        pickle.dump(drone.voronoi_center_tracker, f)
    with open(os.path.join(config.get('RESULTS', 'save_directory'), f"{drone.drone_id}_all_centers.pkl"), "wb") as f:
        pickle.dump(drone.all_voronoi_center_tracker, f)

    if drone.enable_physics_simulation:
        drone.drone.land()
    # time.sleep(5)
    # sys.exit(1)

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        exit()
