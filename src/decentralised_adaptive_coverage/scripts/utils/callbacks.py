from utils import msg_handler
# from utils.mavros_home import model_states_callback
from utils.transformations import model_states_callback
import numpy as np
class CallbackHandler:
    def __init__(self, drone):
        self.drone = drone

    def callback_all_vertices(self, data):
        self.drone.all_vertices = msg_handler.decode_data(data)
        # rospy.loginfo("Received all_vertices: %s", self.all_vertices)

    def callback_voronoi_centers(self, data):
        self.drone.voronoi_centers = msg_handler.decode_data(data)
        # rospy.loginfo("Received voronoi_centers: %s", self.voronoi_centers)

    def callback_boundary_points(self, data):
        self.drone.boundary_points = msg_handler.decode_data(data)
        # rospy.loginfo("Received boundary_points: %s", self.boundary_points)

    def callback_finite_regions(self, data):
        self.drone.finite_regions = msg_handler.decode_data(data)
        # rospy.loginfo("Received finite_regions: %s", self.finite_regions)

    def callback_xx(self, data):
        self.drone.xx = msg_handler.decode_data(data)
        # rospy.loginfo("Received xx: %s", self.xx)

    def callback_yy(self, data):
        self.drone.yy = data
        # rospy.loginfo("Received yy: %s", self.yy)

    def callback_finite_vertices(self, data):
        self.drone.finite_vertices = msg_handler.decode_data(data)
        # rospy.loginfo("Received finite_vertices: %s", self.finite_vertices)

    def callback_grid_points(self, data):
        self.drone.grid_points = msg_handler.decode_data(data)
        # rospy.loginfo("Received grid_points: %s", self.grid_points)

    def callback_weed_density(self, data):
        self.drone.weed_density = msg_handler.decode_data(data)
        # rospy.loginfo("Received weed_density: %s", self.weed_density)

    def center_callback(self, msg, drone_id):
        # Update the center of the specified drone
        self.drone.other_centers[drone_id] = np.array([msg.x, msg.y])

    def mavros_set_home_callback(self, data, drone_number):
        self.drone.drone.transformation_matrix = model_states_callback(data, drone_number+1)

      