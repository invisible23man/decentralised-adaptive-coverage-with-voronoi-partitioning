from tools.rostools.actions import OtherDroneMonitor
import rospy
from geometry_msgs.msg import Point as rosMsgPoint

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