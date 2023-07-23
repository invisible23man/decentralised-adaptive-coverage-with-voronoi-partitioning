#! /home/invisibleman/anaconda3/envs/drones/bin/python

import sys
sys.path.append('/home/invisibleman/Robotics/adaptive-coverage-with-voronoi/devel/lib/python3/dist-packages')
import rospy
from std_msgs.msg import String
from initialization import initial_setup
import configparser
from utils import msg_handler


# def create_field_info_msg(data):
#     field_info = FieldInfoMsgs()
#     int_list_array = IntListArray()

#     for sublist in data:
#         int_list = IntList(data=sublist)
#         int_list_array.lists.append(int_list)

#     field_info.data = int_list_array

#     return field_info

def init_voronoi_node():
    # Initialize ROS node
    rospy.init_node('init_voronoi_node', anonymous=True)

    # Load configuration
    config = configparser.ConfigParser()
    config.read('/home/invisibleman/Robotics/adaptive-coverage-with-voronoi/src/decentralised_adaptive_coverage/scripts/config.ini')

    # Perform initial setup
    vor, finite_vertices, finite_regions, voronoi_centers, xx, yy, \
        grid_points, weed_density, initial_estimates, boundary_points, all_vertices= \
            initial_setup(config, plots=False)

    # Create ROS publishers for variables
    pub_all_vertices = rospy.Publisher('all_vertices', String, queue_size=10)
    pub_voronoi_centers = rospy.Publisher('voronoi_centers', String, queue_size=10)
    pub_boundary_points = rospy.Publisher('boundary_points', String, queue_size=10)
    pub_finite_vertices = rospy.Publisher('finite_vertices', String, queue_size=10)
    pub_finite_regions = rospy.Publisher('finite_regions', String, queue_size=10)
    pub_xx = rospy.Publisher('xx', String, queue_size=10)
    pub_yy = rospy.Publisher('yy', String, queue_size=10)
    pub_grid_points = rospy.Publisher('grid_points', String, queue_size=10)
    pub_weed_density = rospy.Publisher('weed_density', String, queue_size=10)

    while not rospy.is_shutdown():
        # Publish values to topics
        pub_all_vertices.publish(msg_handler.encode_data(data=all_vertices.tolist()))
        pub_voronoi_centers.publish(msg_handler.encode_data(data=voronoi_centers.tolist()))
        pub_boundary_points.publish(msg_handler.encode_data(data=boundary_points.tolist()))
        pub_finite_vertices.publish(msg_handler.encode_data(data=str(finite_vertices)))
        pub_finite_regions.publish(msg_handler.encode_data(data=finite_regions))
        pub_xx.publish(msg_handler.encode_data(data=xx.tolist()))
        pub_yy.publish(msg_handler.encode_data(data=yy.tolist()))
        pub_grid_points.publish(msg_handler.encode_data(data=grid_points.tolist()))
        pub_weed_density.publish(msg_handler.encode_data(data=weed_density.tolist()))
       
if __name__ == '__main__':
    init_voronoi_node()
