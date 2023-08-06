from tools.rostools import msg_handler
from tools.rostools.transformations import model_states_callback
import numpy as np
from std_srvs.srv import SetBool
import rospy

class CallbackHandler:
    def __init__(self, drone):
        self.drone = drone

    def center_callback(self, msg):
        self.drone.drone_positions[self.drone.drone_id] = np.array([msg.x, msg.y, msg.z])
        # rospy.loginfo(f"Center Callback {self.drone.drone_id}: {np.array([msg.x, msg.y, msg.z])}")

    def other_center_callback(self, msg, other_drone_id):
        self.drone.drone_positions[other_drone_id] = np.array([msg.x, msg.y, msg.z])
        # rospy.loginfo(f"Other Center Callback {other_drone_id+1} from Drone {self.drone.drone_id+1}: {np.array([msg.x, msg.y, msg.z])}")

    def state_callback(self, data, drone_id):
        self.drone.other_states[drone_id] = msg_handler.decode_data(data)

    def covariance_callback(self, data, drone_id):
        self.drone.other_covariances[drone_id] = msg_handler.decode_data(data)

    def mavros_set_home_callback(self, data, drone_number):
        self.drone.drone.transformation_matrix = model_states_callback(data, drone_number+1)