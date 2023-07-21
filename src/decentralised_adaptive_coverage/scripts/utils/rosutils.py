import rospy
from std_msgs.msg import String
from PrintColours import *
from utils import msg_encoder

class Drone:
    def __init__(self):

        self.ns = rospy.get_namespace()

        if self.ns == "/":
            rospy.loginfo(CBLUE2 + "Using default namespace" + CEND)
        else:
            rospy.loginfo(CBLUE2 + "Using {} namespace".format(self.ns) + CEND)

        self.all_vertices = String()
        self.voronoi_centers = String()
        self.finite_regions = String()
        self.finite_vertices = String()
        self.boundary_points = String()
        self.xx = String()
        self.yy = String()
        self.grid_points = String()
        self.weed_density = String()

        self.all_vertices = rospy.Subscriber(
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
