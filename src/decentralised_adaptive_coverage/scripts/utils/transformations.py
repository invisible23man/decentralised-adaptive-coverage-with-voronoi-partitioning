#! /home/invisibleman/anaconda3/envs/drones/bin/python

import rospy
from gazebo_msgs.msg import ModelStates
from tf.transformations import euler_from_quaternion, quaternion_from_matrix
import numpy as np
import tf2_ros

# Static MAVROS Home position (local ENU)
MAVROS_HOME_X = 0
MAVROS_HOME_Y = 0
MAVROS_HOME_Z = 0

# Static MAVROS Home orientation (yaw in degrees)
MAVROS_HOME_YAW_DEG = 0

# Gazebo Origin (ENU)
GAZEBO_ORIGIN_X = 0.0
GAZEBO_ORIGIN_Y = 0.0
GAZEBO_ORIGIN_Z = 0.0

# Dictionary to store Gazebo pose and orientation for each drone
gazebo_drone_poses = {}

# Create a tf2_ros.Buffer and tf2_ros.TransformListener
tf_buffer = tf2_ros.Buffer()
tf_listener = tf2_ros.TransformListener(tf_buffer)

def model_states_callback(data, drone_number):
    """
    Callback function to process the ModelStates message and extract the pose and orientation of the specified drone.

    :param data: ModelStates message containing the pose and orientation of all models in Gazebo.
    :param drone_number: The drone number for which to extract the pose and orientation.
    """
    # # Unsubscribe after receiving the data once
    # model_states_sub.unregister()

    for name, pose in zip(data.name, data.pose):
        if name == "drone" + str(drone_number):
            drone_pose = {
                'position': {
                    'x': pose.position.x,
                    'y': pose.position.y,
                    'z': pose.position.z
                },
                'orientation': {
                    'roll': 0.0,  # Replace with actual roll
                    'pitch': 0.0,  # Replace with actual pitch
                    'yaw': 0.0  # Replace with actual yaw
                }
            }
            roll, pitch, yaw = euler_from_quaternion([pose.orientation.x,
                                                      pose.orientation.y,
                                                      pose.orientation.z,
                                                      pose.orientation.w])
            drone_pose['orientation']['roll'] = roll
            drone_pose['orientation']['pitch'] = pitch
            drone_pose['orientation']['yaw'] = yaw

            break

    # Now that we have all the Gazebo poses and orientations, compute the transformation matrix
    mavros_home = {
        'position': {
            'x': MAVROS_HOME_X,
            'y': MAVROS_HOME_Y,
            'z': MAVROS_HOME_Z
        },
        'orientation': {
            'yaw': MAVROS_HOME_YAW_DEG
        }
    }
    transformation_matrix = compute_transformation_matrix(drone_pose, mavros_home)

    # Broadcast the transformation matrix
    broadcast_transformation_matrix(transformation_matrix, "world", f"drone{drone_number}")

def compute_transformation_matrix(gazebo_pose, mavros_home):
    """
    Compute the transformation matrix from Gazebo frame to MAVROS frame based on the drone pose and home position.

    :param gazebo_pose: Dictionary containing the position and orientation of the drone in Gazebo frame.
    :param mavros_home: Dictionary containing the MAVROS home position (x, y, z) and orientation (yaw).
    :return: 4x4 transformation matrix.
    """
    # Extract Gazebo drone position
    gazebo_x = gazebo_pose['position']['x']
    gazebo_y = gazebo_pose['position']['y']
    gazebo_z = gazebo_pose['position']['z']

    # Extract MAVROS home position (local ENU)
    mavros_x = mavros_home['position']['x']
    mavros_y = mavros_home['position']['y']
    mavros_z = mavros_home['position']['z']

    # Calculate the translation components (Gazebo origin to MAVROS home)
    x_translation = mavros_x - gazebo_x
    y_translation = mavros_y - gazebo_y
    z_translation = mavros_z - gazebo_z

    # Extract MAVROS home orientation (yaw)
    mavros_yaw = mavros_home['orientation']['yaw']

    # Convert yaw to radians
    yaw_rad = np.radians(mavros_yaw)

    # Compute the rotation matrix (yaw rotation only for simplicity)
    cos_yaw = np.cos(yaw_rad)
    sin_yaw = np.sin(yaw_rad)
    rotation_matrix = np.array([[cos_yaw, -sin_yaw, 0],
                                [sin_yaw, cos_yaw, 0],
                                [0, 0, 1]])

    # Create the 4x4 transformation matrix
    transformation_matrix = np.eye(4)
    transformation_matrix[:3, :3] = rotation_matrix
    transformation_matrix[0, 3] = x_translation
    transformation_matrix[1, 3] = y_translation
    transformation_matrix[2, 3] = z_translation

    return transformation_matrix
   
def broadcast_transformation_matrix(transform_matrix, parent_frame_id, child_frame_id):
    """
    Broadcast the transformation matrix as a TransformStamped message.

    :param transform_matrix: 4x4 transformation matrix.
    :param parent_frame_id: Parent frame ID for the transformation.
    :param child_frame_id: Child frame ID for the transformation.
    """
    # Create a TransformStamped message
    transform_msg = tf2_ros.TransformStamped()
    transform_msg.header.stamp = rospy.Time.now()
    transform_msg.header.frame_id = parent_frame_id
    transform_msg.child_frame_id = child_frame_id

    # Convert the transformation matrix to translation and rotation
    translation = transform_matrix[:3, 3]
    rotation = quaternion_from_matrix(transform_matrix)

    # Set translation
    transform_msg.transform.translation.x = translation[0]
    transform_msg.transform.translation.y = translation[1]
    transform_msg.transform.translation.z = translation[2]

    # Set rotation (as a quaternion)
    transform_msg.transform.rotation.x = rotation[0]
    transform_msg.transform.rotation.y = rotation[1]
    transform_msg.transform.rotation.z = rotation[2]
    transform_msg.transform.rotation.w = rotation[3]

    # Broadcast the transformation matrix using tf2_ros.TransformBroadcaster
    tf_broadcaster = tf2_ros.TransformBroadcaster()
    tf_broadcaster.sendTransform(transform_msg)

def apply_transformation_matrix(x, y, z, drone_number):
    """
    Apply the transformation matrix to convert Gazebo coordinates to MAVROS coordinates.

    :param x: Gazebo X-coordinate.
    :param y: Gazebo Y-coordinate.
    :param z: Gazebo Z-coordinate.
    :param drone_number: The drone number for which to retrieve the transformation matrix.
    :return: Tuple containing the MAVROS ENU coordinates (enu_x, enu_y, enu_z).
    """
    # Wait for the tf2 buffer to become available
    while not rospy.is_shutdown() and not tf_buffer.can_transform(f"world", f"drone{drone_number}", rospy.Time()):
        rospy.sleep(0.1)

    # Create a PointStamped message for the Gazebo coordinates
    point_gazebo = PointStamped()
    point_gazebo.header.stamp = rospy.Time(0)
    point_gazebo.header.frame_id = "world"
    point_gazebo.point.x = x
    point_gazebo.point.y = y
    point_gazebo.point.z = z

    # Transform the Gazebo coordinates to MAVROS coordinates using the tf2 buffer
    try:
        point_mavros = tf_buffer.transform(point_gazebo, f"drone{drone_number}")
        mavros_x = point_mavros.point.x
        mavros_y = point_mavros.point.y
        mavros_z = point_mavros.point.z
        return mavros_x, mavros_y, mavros_z
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        rospy.logwarn(f"Failed to transform coordinates: {e}")
        return None, None, None

if __name__ == "__main__":
    rospy.init_node('get_gazebo_home_and_transform_to_mavros_home', anonymous=True)

    # Replace 'DRONE_NUMBER' with the desired drone number, e.g., 1, 2, 3, etc.
    desired_drone_number = 2

    model_states_sub = rospy.Subscriber('/gazebo/model_states', ModelStates, model_states_callback, desired_drone_number)

    # Create a tf2_ros.TransformBroadcaster
    tf_broadcaster = tf2_ros.TransformBroadcaster()
    rospy.loginfo("Transform Broadcasted")

    rospy.spin()
