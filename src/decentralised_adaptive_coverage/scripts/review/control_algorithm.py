#! /usr/bin/python

import rospy
import configparser
import time
import pickle
import os
from tqdm import tqdm
from std_srvs.srv import SetBool
from decentralised_adaptive_coverage.scripts.review import Drones

def main():
    # Initializing ROS node.
    rospy.init_node("drone_controller" + rospy.get_namespace().replace('/', ''), anonymous=True)

    # Load configuration
    config = configparser.ConfigParser()
    config.read(
        '/home/invisible23man/Robotics/Simulations/decentralised-adaptive-coverage-with-voronoi-partitioning/src/decentralised_adaptive_coverage/scripts/config.ini')
    n_iter = config.getint('ITERATIVE_PROCESS', 'n_iterations')
    n_drones = config.getint('INITIAL_SETUP','n_drones') 

    # Initlialize the Drone
    drone = Drones.Drone(config)

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
        
        # rospy.Service(f'/drone{drone.drone_id}/ready', SetBool, drone.handle_ready_service)
        
        # drone.callback_handler.ready_service_callback()

        # Wait until all drones have published their state and covariance
        if config.getboolean('FILTER_SETUP','apply_consensus'):
            drone.publish_state_and_covariance()
            counter = 0 
            while len(drone.other_states) < n_drones- 1 or len(drone.other_covariances) < n_drones- 1:
                rospy.sleep(0.1)  # Sleep for a short duration before checking again
                if counter % 15 == 0:
                    rospy.loginfo(f"Waiting for other drones to send x,P:drone{drone.drone_id}:: \
                                    x,P:{len(drone.other_states)},{len(drone.other_covariances)}")
                counter +=1
                time.sleep(1)  # Sleep for a short time to avoid busy waiting

        rospy.loginfo(f"Calculating New Center:drone{drone.drone_id}, iter:{i}")
        drone.calculate_new_voronoi_center()

        # Wait until all drones have updated their centers
        counter = 0 
        while len(drone.other_centers) != drone.config.getint('INITIAL_SETUP','n_drones') - 1:
            if counter % 15 == 0:
                rospy.loginfo(f"Waiting for other drones to update their centers:drone{drone.drone_id}, o_centers:{len(drone.other_centers)},iter:{i}")
            counter +=1
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
