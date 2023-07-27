#! /home/invisibleman/anaconda3/envs/drones/bin/python

import rospy
from gazebo_msgs.msg import ModelStates
from mavros_msgs.srv import CommandHome
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion
import utm
import json

def model_states_callback(data, drone_number):
    drone_pose = None
    for name, pose in zip(data.name, data.pose):
        if name == "drone" + str(drone_number):
            drone_pose = pose
            break

    if drone_pose:
        home_position = {
            'x': drone_pose.position.x,
            'y': drone_pose.position.y,
            'z': drone_pose.position.z
        }
        roll, pitch, yaw = euler_from_quaternion([drone_pose.orientation.x,
                                                  drone_pose.orientation.y,
                                                  drone_pose.orientation.z,
                                                  drone_pose.orientation.w])
        home_position['roll'] = roll
        home_position['pitch'] = pitch
        home_position['yaw'] = yaw

        print(f"Position and Orientation of drone {drone_number} in Gazebo (JSON):")
        print(json.dumps(home_position, indent=2))

        # Convert ENU position to LLA
        enu_position = (home_position['x'], home_position['y'], home_position['z'])
        geodetic_position = enu_2_geodetic(enu_position)
        mavros_home = geodetic_to_mavros_home(geodetic_position, home_position['yaw'])

        set_mavros_home(drone_number, mavros_home)
    else:
        print(f"Drone {drone_number} not found in Gazebo.")

    # Unsubscribe after receiving the data once
    model_states_sub.unregister()

def enu_2_geodetic(enu_position):
    # Scaling factor to bring the ENU coordinates within the valid UTM range
    scale_factor = 1e5

    # Replace with the origin of your local ENU frame (in LLA format)
    origin_lat = 0.0
    origin_lon = 0.0

    scaled_enu_position = [pos + scale_factor for pos in enu_position]
    # geodetic_position = utm.to_latlon(scaled_enu_position[0], scaled_enu_position[1], zone_number=1, northern=True)

    geodetic_position = enu_position

    return geodetic_position[0], geodetic_position[1], enu_position[2]


def geodetic_to_mavros_home(geodetic_position, yaw):
    # Create the MAVROS home position format
    mavros_home = CommandHome()
    mavros_home.current_gps = False
    mavros_home.yaw = yaw
    mavros_home.latitude = geodetic_position[0]
    mavros_home.longitude = geodetic_position[1]
    mavros_home.altitude = geodetic_position[2]
    return mavros_home

def set_mavros_home(drone_number, mavros_home:CommandHome):
    rospy.wait_for_service(f'/drone{drone_number}/mavros/cmd/set_home')
    try:
        set_home = rospy.ServiceProxy(f'/drone{drone_number}/mavros/cmd/set_home', CommandHome)
        response = set_home(
            mavros_home.current_gps,
            mavros_home.yaw,
            mavros_home.latitude,
            mavros_home.longitude,
            mavros_home.altitude
        )

        if response.success:
            print(f"Drone {drone_number} MAVROS Home position set successfully.")
        else:
            print(f"Failed to set MAVROS Home position for drone {drone_number}.")

    except rospy.ServiceException as e:
        print("Service call failed:", e)

if __name__ == "__main__":
    rospy.init_node('get_gazebo_home_and_set_mavros_home', anonymous=True)

    # Replace 'DRONE_NUMBER' with the desired drone number, e.g., 1, 2, 3, etc.
    desired_drone_number = 1

    model_states_sub = rospy.Subscriber('/gazebo/model_states', ModelStates, model_states_callback, desired_drone_number)
    rospy.spin()
