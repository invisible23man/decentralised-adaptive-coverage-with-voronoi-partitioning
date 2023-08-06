import actionlib
import rospy
from decentralised_adaptive_coverage.msg import (VoronoiUpdateAction, VoronoiUpdateGoal,
                                                 VoronoiUpdateFeedback, VoronoiUpdateResult)

class Action:
    def __init__(self, drone):
        self.drone = drone

    def setup_actions(self):
        self.server = actionlib.SimpleActionServer(f'/drone{self.drone.drone_id+1}/update_voronoi_action', 
                                            VoronoiUpdateAction, 
                                            execute_cb=self.drone_update_voronoi_action, 
                                            auto_start=False)
        self.server.start()

        # self.drone.other_drones = {}
        # for i in range(self.drone.drone_count):
        #     if i != self.drone.drone_id:  # Don't create a client for our own drone
        #         self.drone.other_drones[i] = OtherDroneMonitor(f'/drone/{i+1}')

    def drone_update_voronoi_action(self, goal):
        # This function will be called when the action is requested.
        # It should perform the action and send feedback about its progress.
        # For this example, we'll just simulate a long-running action.
        for i in range(100):
            if self.server.is_preempt_requested():
                self.server.set_preempted()
                return
            rospy.sleep(0.1)
            self.server.publish_feedback(VoronoiUpdateFeedback(percent_complete=i/100.0))
        self.drone.update_voronoi_completed = True
        self.server.set_succeeded(VoronoiUpdateResult(success=True))

    def wait_for_all_drones(self):
        # Create an action client for each drone
        action_clients = [actionlib.SimpleActionClient(f'/drone{i+1}/update_voronoi_action', 
                                                    VoronoiUpdateAction) 
                        for i in range(self.drone.drone_count) if i != self.drone.drone_id]

        # Wait for all drones to finish their tasks
        while not all(client.wait_for_result(timeout=rospy.Duration(0.1)) for client in action_clients):
            rospy.loginfo(f"Drone {self.drone.drone_id+1} waiting for other drones")
            rospy.sleep(1.0)  # Adjust the sleep duration as needed


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
