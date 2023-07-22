import sys
sys.path.append(r'/home/invisibleman/Robotics/adaptive-coverage-with-voronoi/src/decentralised_adaptive_coverage/scripts')

from utils import plots
import configparser
import numpy as np
import rospy
from std_msgs.msg import String

# Load configuration
config = configparser.ConfigParser()
config.read(
    '/home/invisibleman/Robotics/adaptive-coverage-with-voronoi/src/decentralised_adaptive_coverage/scripts/config.ini')

# Global variable to store the latest weed distribution data
weed_distribution = None

def weed_distribution_callback(data):
    global weed_distribution
    # Assuming the data is published as a numpy array
    weed_distribution = np.array(data.data).reshape((data.info.height, data.info.width))

if __name__ == '__main__':

    # Initializing ROS node.
    rospy.init_node("drone_controller" + rospy.get_namespace().replace('/', ''), anonymous=True)

    # Create a subscriber for the weed distribution topic
    rospy.Subscriber("/weed_density", String, weed_distribution_callback)
    
    plots.plot_results(config, weed_distribution)