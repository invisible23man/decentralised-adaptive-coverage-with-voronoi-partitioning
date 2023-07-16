#! /home/invisibleman/anaconda3/envs/drones/bin/ python

import rospy
from std_msgs.msg import Float64
from robots import Robot
from initialization import initial_setup
import configparser
from utils import plots

def init_voronoi_node():
    # Initialize ROS node
    rospy.init_node('init_voronoi_node', anonymous=True)

    # Load configuration
    config = configparser.ConfigParser()
    config.read('/home/invisibleman/Robotics/adaptive-coverage-with-voronoi/src/decentralised_adaptive_coverage/src/config.ini')

    # Get values from the config file
    n_drones = config.getint('INITIAL_SETUP', 'n_drones')
    r_area = config.getfloat('INITIAL_SETUP', 'r_area')
    grid_resolution = config.getfloat('INITIAL_SETUP', 'grid_resolution')
    filter_type = config.get('INITIAL_SETUP', 'filter_type')
    num_particles = config.getint('INITIAL_SETUP', 'num_particles')

    # Perform initial setup
    vor, finite_vertices, finite_regions, voronoi_centers, xx, yy, \
        grid_points, weed_density, initial_estimates, boundary_points = \
            initial_setup(n=n_drones, r=r_area, filter_type=filter_type,
                          num_particles=num_particles, plots=False)

    # Create ROS publishers for variables
    pub_vor = rospy.Publisher('vor', Float64, queue_size=10)
    pub_finite_vertices = rospy.Publisher('finite_vertices', Float64, queue_size=10)
    pub_finite_regions = rospy.Publisher('finite_regions', Float64, queue_size=10)
    pub_voronoi_centers = rospy.Publisher('voronoi_centers', Float64, queue_size=10)
    pub_xx = rospy.Publisher('xx', Float64, queue_size=10)
    pub_yy = rospy.Publisher('yy', Float64, queue_size=10)
    pub_grid_points = rospy.Publisher('grid_points', Float64, queue_size=10)
    pub_weed_density = rospy.Publisher('weed_density', Float64, queue_size=10)
    pub_initial_estimates = rospy.Publisher('initial_estimates', Float64, queue_size=10)
    pub_boundary_points = rospy.Publisher('boundary_points', Float64, queue_size=10)

    # Publish values to topics
    pub_vor.publish(vor)
    pub_finite_vertices.publish(finite_vertices)
    pub_finite_regions.publish(finite_regions)
    pub_voronoi_centers.publish(voronoi_centers)
    pub_xx.publish(xx)
    pub_yy.publish(yy)
    pub_grid_points.publish(grid_points)
    pub_weed_density.publish(weed_density)
    pub_initial_estimates.publish(initial_estimates)
    pub_boundary_points.publish(boundary_points)

    rospy.spin()

if __name__ == '__main__':
    init_voronoi_node()
