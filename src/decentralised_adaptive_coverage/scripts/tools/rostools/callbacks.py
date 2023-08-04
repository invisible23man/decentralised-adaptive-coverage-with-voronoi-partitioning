from tools.rostools import msg_handler
from tools.rostools.transformations import model_states_callback
import numpy as np
from std_srvs.srv import SetBool
import rospy

class CallbackHandler:
    def __init__(self, drone):
        self.drone = drone

    def center_callback(self, msg, drone_id):
        self.drone.drone_positions[drone_id] = np.array([msg.x, msg.y, msg.z])

    def callback_voronoi_centers(self, data):
        self.drone.voronoi_centers = msg_handler.decode_data(data)


    def state_callback(self, data, drone_id):
        self.drone.other_states[drone_id] = msg_handler.decode_data(data)

    def covariance_callback(self, data, drone_id):
        self.drone.other_covariances[drone_id] = msg_handler.decode_data(data)

    def mavros_set_home_callback(self, data, drone_number):
        self.drone.drone.transformation_matrix = model_states_callback(data, drone_number+1)

    def ready_service_callback(self):
        for i in range(self.drone.config.getint('INITIAL_SETUP','n_drones')):
            if i != self.drone.drone_id:  # Don't call our own service
                rospy.wait_for_service(f'/drone{i}/ready')
                try:
                    ready_service = rospy.ServiceProxy(f'/drone{i}/ready', SetBool)
                    resp = ready_service()
                    if not resp.success:
                        rospy.logerr(f'Drone {i} is not ready')
                        # Handle the error, e.g., by waiting and trying again
                except rospy.ServiceException as e:
                    rospy.logerr(f'Service call failed: {e}')
                    # Handle the error
