from tools.rostools.actions import OtherDroneMonitor
import rospy
from geometry_msgs.msg import Point as rosMsgPoint
from gazebo_msgs.msg import ModelStates

class Subscriber:
    def __init__(self, drone):
        self.drone = drone

    def setup_subscribers(self):
        # Setup Subscribers: all voronoi centers, measurements, variances
        rospy.Subscriber(
                f'/drone{self.drone.drone_id+1}/center',
                rosMsgPoint,
                self.drone.callback_handler.center_callback,
        )
        rospy.sleep(1)

        # Initialize subscribers for the other drones
        for i in range(self.drone.drone_count):
            if i != self.drone.drone_id:  # Don't subscribe to our own topic
                rospy.Subscriber(
                        f'/drone{i+1}/center',
                        rosMsgPoint,
                        self.drone.callback_handler.other_center_callback,
                        callback_args = i
                )
        rospy.sleep(1)

    def setup_gnc_drone_subscribers(self):
        # Subscriber for coordinate transformations
        self.model_states_sub = rospy.Subscriber(
            '/gazebo/model_states', 
            ModelStates, 
            self.drone.callback_handler.mavros_set_home_callback, 
            self.drone.drone_id
        )

        # Wait for the transformation matrix to become available
        while self.drone.gnc_drone.transformation_matrix is None:
            rospy.loginfo(f"Waiting for transformation matrix for {self.drone.drone_id}")
            rospy.sleep(0.1)

        # Unsubscribe after receiving the data once
        self.model_states_sub.unregister() 
        rospy.loginfo(f"Transform Broadcasted for {self.drone.drone_id}")