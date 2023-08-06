from tools.rostools.transformations import apply_transformation_matrix
from iq_gnc.py_gnc_functions_swarm import rospy, gnc_api, Odometry

def navigate_to_destination(x, y, z, gnc_drone:gnc_api):
    """
    Navigate the drone to a specific destination in the Gazebo world.

    Parameters:
        gnc_drone (YourDroneController): An instance of your drone controller class.
        x (float): Gazebo x-coordinate (East) of the destination point in meters.
        y (float): Gazebo y-coordinate (North) of the destination point in meters.
        z (float): Gazebo z-coordinate (Up) of the destination point in meters.

    Returns:
        None

    Description:
        This function sets the drone's target destination using the provided drone controller (gnc_drone).
        The drone will navigate to the specified destination point in the Gazebo world.

    Note:
        - Make sure you have already initialized the ROS node and created the drone object (gnc_drone)
          before calling this function.
        - The function assumes that the drone's orientation (quaternion) is set to (0, 0, 0, 1)
          for simplicity (no rotation).
    """
    # Apply the transformation matrix to convert Gazebo coordinates to MAVROS coordinates
    enu_x, enu_y, enu_z = apply_transformation_matrix(x, y, z, gnc_drone.transformation_matrix)

    # Create the Odometry message with ENU coordinates
    destination = Odometry()

    # Set the position (ENU coordinates)
    destination.pose.pose.position.x = enu_x
    destination.pose.pose.position.y = enu_y
    destination.pose.pose.position.z = enu_z

    # Set the orientation (quaternion - assuming you have orientation information)
    destination.pose.pose.orientation.x = 0.0
    destination.pose.pose.orientation.y = 0.0
    destination.pose.pose.orientation.z = 0.0
    destination.pose.pose.orientation.w = 1.0  # Assuming no rotation for simplicity

    destination_pose_local = gnc_drone.enu_2_local(destination, gnc_drone.local_offset_g)
    gnc_drone.set_destination(
        destination_pose_local.x,
        destination_pose_local.y,
        destination_pose_local.z,
        psi=0,
    )

    gnc_drone.rate.sleep()
    
    return gnc_drone.check_waypoint_reached()
