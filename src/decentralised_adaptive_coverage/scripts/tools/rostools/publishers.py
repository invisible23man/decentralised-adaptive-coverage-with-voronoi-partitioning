import rospy
from geometry_msgs.msg import Point as rosMsgPoint

class Publisher:
    def __init__(self, drone):
        self.drone = drone

    def setup_publishers(self):
        # Setup Publishers: voronoi center, measurements, variances
        self.pub_center = rospy.Publisher(
            f'/drone{self.drone.drone_id+1}/center', rosMsgPoint, queue_size=10, latch=True)
        self.drone.voronoi_center = self.drone.drone_positions[self.drone.drone_id]
        self.pub_center.publish(rosMsgPoint(
            self.drone.voronoi_center[0], self.drone.voronoi_center[1], self.drone.voronoi_center[2]))
        rospy.sleep(1)

    def publish_payload(self, voronoi_center):
        # Setup Publishers: voronoi centers, measurements, variances
        self.drone.update_voronoi_completed = False
        center_msg = rosMsgPoint(
            voronoi_center[0], voronoi_center[1], voronoi_center[2])
        self.pub_center.publish(center_msg)
        rospy.sleep(1)
        self.drone.update_voronoi_completed = True
