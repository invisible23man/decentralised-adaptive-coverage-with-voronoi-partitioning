#! /home/invisible23man/anaconda3/envs/drones/bin/ python
import rospy
from std_msgs.msg import Float64

def callback(data):
    rospy.loginfo("Received time %s", data.data)

def listener():
    rospy.init_node('time_collector')
    rospy.Subscriber("time_taken", Float64, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
