from move import generate_rectangular_spiral_path
import rospy
from std_msgs.msg import String
from PrintColours import *
from utils import msg_encoder

class Drone:
    def __init__(self, config):

        self.ns = rospy.get_namespace()
        self.drone_id = int(self.ns.strip('/').replace('drone', ''))-1

        if self.ns == "/":
            rospy.loginfo(CBLUE2 + "Using default namespace" + CEND)
        else:
            rospy.loginfo(CBLUE2 + "Using {} namespace".format(self.ns) + CEND)

        self.voronoi_center = None
        self.voronoi_region = None
        self.spiral_path = [] 
        self.sensor_values_from_spiral_path = []

        self.all_vertices = String()
        self.voronoi_centers = String()
        self.finite_regions = String()
        self.finite_vertices = String()
        self.boundary_points = String()
        self.xx = String()
        self.yy = String()
        self.grid_points = String()
        self.weed_density = String()

        self.config = config
        self.grid_resolution = config.getfloat('INITIAL_SETUP', 'grid_resolution')
        self.sampling_time = config.getint('SIMULATION_SETUP', 'sampling_time')
        self.time_per_step = config.getfloat('SIMULATION_SETUP', 'time_per_step')
        self.boundary_tolerance=0.02

        self.all_vertices_sub = rospy.Subscriber(
            "/all_vertices", 
            String, 
            self.callback_all_vertices
        )

        self.voronoi_centers_sub = rospy.Subscriber(
            "/voronoi_centers", 
            String, 
            self.callback_voronoi_centers
        )
        self.boundary_points_sub = rospy.Subscriber(
            "/boundary_points", 
            String, 
            self.callback_boundary_points
        )
        self.finite_regions_sub = rospy.Subscriber(
            "/finite_regions", 
            String, 
            self.callback_finite_regions
        )
        self.xx_sub = rospy.Subscriber(
            "/xx", 
            String, 
            self.callback_xx
        )
        self.yy_sub = rospy.Subscriber(
            "/yy", 
            String, 
            self.callback_yy
        )

        self.finite_vertices_sub = rospy.Subscriber(
            "/finite_vertices", 
            String, 
            self.callback_finite_vertices
        )
        self.grid_points_sub = rospy.Subscriber(
            "/grid_points", 
            String, 
            self.callback_grid_points
        )
        self.weed_density_sub = rospy.Subscriber(
            "/weed_density", 
            String, 
            self.callback_weed_density
        )

    @property
    def ready(self):
        return all([self.voronoi_centers.data, self.boundary_points.data, self.finite_regions.data, self.xx.data, self.yy.data])

    def callback_all_vertices(self, data):
        self.all_vertices = msg_encoder.decode_data(data)
        # rospy.loginfo("Received all_vertices: %s", self.all_vertices)

    def callback_voronoi_centers(self, data):
        self.voronoi_centers = msg_encoder.decode_data(data)
        # rospy.loginfo("Received voronoi_centers: %s", self.voronoi_centers)

    def callback_boundary_points(self, data):
        self.boundary_points = msg_encoder.decode_data(data)
        # rospy.loginfo("Received boundary_points: %s", self.boundary_points)

    def callback_finite_regions(self, data):
        self.finite_regions = msg_encoder.decode_data(data)
        # rospy.loginfo("Received finite_regions: %s", self.finite_regions)

    def callback_xx(self, data):
        self.xx = msg_encoder.decode_data(data)
        # rospy.loginfo("Received xx: %s", self.xx)

    def callback_yy(self, data):
        self.yy = data
        # rospy.loginfo("Received yy: %s", self.yy)

    def callback_finite_vertices(self, data):
        self.finite_vertices = msg_encoder.decode_data(data)
        # rospy.loginfo("Received finite_vertices: %s", self.finite_vertices)

    def callback_grid_points(self, data):
        self.grid_points = msg_encoder.decode_data(data)
        # rospy.loginfo("Received grid_points: %s", self.grid_points)

    def callback_weed_density(self, data):
        self.weed_density = msg_encoder.decode_data(data)
        # rospy.loginfo("Received weed_density: %s", self.weed_density)

    def move_and_sense(self):

        self.voronoi_center = self.voronoi_centers[self.drone_id]
        self.voronoi_region = self.finite_regions[self.drone_id]

        self.spiral_path, self.sensor_values_from_spiral_path = generate_rectangular_spiral_path(
            self.voronoi_center, 
            [self.all_vertices[i] for i in self.voronoi_region], 
            self.grid_resolution, 
            self.grid_points, 
            self.weed_density,
            self.sampling_time, 
            self.time_per_step, 
            self.boundary_tolerance)


