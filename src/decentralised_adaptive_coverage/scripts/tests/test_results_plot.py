#! /usr/bin/python
import sys
sys.path.append(r'/home/invisible23man/Robotics/Simulations/decentralised-adaptive-coverage-with-voronoi-partitioning/src/decentralised_adaptive_coverage/scripts')

from utils import plots
import configparser
import numpy as np
import rospy
from std_msgs.msg import String
from utils import msg_handler

# Load configuration
config = configparser.ConfigParser()
config.read(
    '/home/invisible23man/Robotics/Simulations/decentralised-adaptive-coverage-with-voronoi-partitioning/src/decentralised_adaptive_coverage/scripts/config.ini')

# Global variables to store the latest data from the topics
boundary_points = None
xx = None
yy = None
grid_points = None
weed_density = None

def callback_boundary_points(data):
    global boundary_points
    boundary_points = msg_handler.decode_data(data)

def callback_xx(data):
    global xx
    xx = msg_handler.decode_data(data)

def callback_yy(data):
    global yy
    yy = msg_handler.decode_data(data)

def callback_grid_points(data):
    global grid_points
    grid_points = msg_handler.decode_data(data)

def callback_weed_density(data):
    global weed_density
    weed_density = msg_handler.decode_data(data)

if __name__ == '__main__':

    # Initializing ROS node.
    rospy.init_node("drone_controller" + rospy.get_namespace().replace('/', ''), anonymous=True)

    # Create subscribers for the topics
    rospy.Subscriber("/boundary_points", String, callback_boundary_points)
    rospy.Subscriber("/xx", String, callback_xx)
    rospy.Subscriber("/yy", String, callback_yy)
    rospy.Subscriber("/grid_points", String, callback_grid_points)
    rospy.Subscriber("/weed_density", String, callback_weed_density)

    # Wait for data to be received
    while boundary_points is None or xx is None or yy is None or grid_points is None or weed_density is None:
        pass

    # plots.plot_results_with_voronoi(config, boundary_points, xx, yy, grid_points, weed_density)
    plots.plot_results_3D_overlay(config, boundary_points, xx, yy, grid_points, weed_density)

