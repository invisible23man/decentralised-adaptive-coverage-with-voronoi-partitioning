#! /home/invisibleman/anaconda3/envs/drones/bin/python
import rospy
from gazebo_msgs.msg import ModelStates
import json

def model_states_callback(data, drone_number):
    drone_position = None
    drone_orientation = None

    for name, pose in zip(data.name, data.pose):
        if name == "drone" + str(drone_number):
            drone_position = pose.position
            drone_orientation = pose.orientation
            break

    if drone_position and drone_orientation:
        json_data = json.dumps({
            'x': drone_position.x,
            'y': drone_position.y,
            'z': drone_position.z,
            'orientation': {
                'x': drone_orientation.x,
                'y': drone_orientation.y,
                'z': drone_orientation.z,
                'w': drone_orientation.w
            }
        }, indent=2)
        print(f"Position and Orientation of drone {drone_number} in Gazebo (JSON):")
        print(json_data)
    else:
        print(f"Drone {drone_number} not found in Gazebo.")

    # Unsubscribe after receiving the data once
    model_states_sub.unregister()
    
    return json_data

if __name__ == "__main__":
    rospy.init_node('get_drone_position', anonymous=True)

    # Replace 'DRONE_NUMBER' with the desired drone number, e.g., 1, 2, 3, etc.
    desired_drone_number = 3

    model_states_sub = rospy.Subscriber('/gazebo/model_states', ModelStates, model_states_callback, desired_drone_number)
    rospy.spin()

