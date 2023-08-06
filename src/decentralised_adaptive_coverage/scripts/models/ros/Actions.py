import actionlib
import rospy
from decentralised_adaptive_coverage.msg import VoronoiUpdateAction, VoronoiUpdateGoal

class OtherDroneMonitor(object):
    def __init__(self, other_drone_name):
        self.client = actionlib.SimpleActionClient(other_drone_name, VoronoiUpdateAction)
        self.client.wait_for_server()

    def start_monitoring(self):
        # You don't need to send a goal if you're just monitoring the other drone
        self.client.send_goal(VoronoiUpdateGoal(), done_cb=self.done_cb, feedback_cb=self.feedback_cb)

    def done_cb(self, status, result):
        rospy.loginfo('Other drone finished updating Voronoi center')

    def feedback_cb(self, feedback):
        rospy.loginfo('Other drone is %f percent complete', feedback.percent_complete)
