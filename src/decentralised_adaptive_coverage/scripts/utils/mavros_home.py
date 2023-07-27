#! /home/invisibleman/anaconda3/envs/drones/bin/python

import rospy
from gazebo_msgs.msg import ModelStates
from mavros_msgs.srv import CommandHome
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion
from geographiclib.geodesic import Geodesic
import utm
import json
import pyproj

# ArduPilot SITL Home Location (LLA)
ARDUPILOT_HOME_LAT = -35.363262
ARDUPILOT_HOME_LON = 149.165237
ARDUPILOT_HOME_ALT = 584.0

# Gazebo Origin (ENU)
GAZEBO_ORIGIN_X = 0.0
GAZEBO_ORIGIN_Y = 0.0
GAZEBO_ORIGIN_Z = 0.0

def enu_to_lla(enu_x, enu_y, enu_z, lib = 'pyproj'):
    if lib == 'geodesic':
        # Convert ENU to LLA using GeographicLib
        geod = Geodesic.WGS84
        d = geod.Direct(ARDUPILOT_HOME_LAT, ARDUPILOT_HOME_LON, 0.0, enu_x, enu_y, enu_z)
        lla_lat = d['lat2']
        lla_lon = d['lon2']
        lla_alt = ARDUPILOT_HOME_ALT + enu_z
    else:
        enu_to_lla_proj = pyproj.Proj(proj='latlong', ellps='WGS84', datum='WGS84')
        lla_lon, lla_lat, lla_alt = enu_to_lla_proj(enu_x, enu_y, enu_z, inverse=True)
    return lla_lat, lla_lon, lla_alt

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

        rospy.loginfo(f"Position and Orientation of drone {drone_number} in Gazebo (JSON):")
        rospy.loginfo(json.dumps(home_position, indent=2))

        # Convert ENU position to LLA
        enu_position = (home_position['x'], home_position['y'], home_position['z'])
        geodetic_position = enu_2_geodetic(enu_position)
        mavros_home = geodetic_to_mavros_home(geodetic_position, home_position['yaw'])

        set_mavros_home(drone_number, mavros_home)
    else:
        rospy.loginfo(f"Drone {drone_number} not found in Gazebo.")

    # Unsubscribe after receiving the data once
    # model_states_sub.unregister()

def enu_2_geodetic(enu_position):
    # Scaling factor to bring the ENU coordinates within the valid UTM range
    scale_factor = 1e5

    # Replace with the origin of your local ENU frame (in LLA format)
    origin_lat = 0.0
    origin_lon = 0.0

    scaled_enu_position = [pos + scale_factor for pos in enu_position]
    # geodetic_position = utm.to_latlon(scaled_enu_position[0], scaled_enu_position[1], zone_number=1, northern=True)

    # geodetic_position = enu_position
    geodetic_position = enu_to_lla(enu_position[0],enu_position[1],enu_position[2])

    return geodetic_position[0], geodetic_position[1], geodetic_position[2]


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
            rospy.loginfo(f"Drone {drone_number} MAVROS Home position set successfully.")
        else:
            rospy.loginfo(f"Failed to set MAVROS Home position for drone {drone_number}.")

    except rospy.ServiceException as e:
        rospy.loginfo("Service call failed:", e)

if __name__ == "__main__":
    rospy.init_node('get_gazebo_home_and_set_mavros_home', anonymous=True)

    # Replace 'DRONE_NUMBER' with the desired drone number, e.g., 1, 2, 3, etc.
    desired_drone_number = 1

    model_states_sub = rospy.Subscriber('/gazebo/model_states', ModelStates, model_states_callback, desired_drone_number)
    rospy.spin()
