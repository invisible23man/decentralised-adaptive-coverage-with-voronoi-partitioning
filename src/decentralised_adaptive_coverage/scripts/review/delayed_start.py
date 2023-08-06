#! /usr/bin/python
import rospy
import roslaunch
import time

def delayed_start():
    rospy.init_node('delayed_start_node', anonymous=True)

    # Get parameters
    start_delay = rospy.get_param('start_delay')
    node_pkg = rospy.get_param('~node_pkg')
    node_type = rospy.get_param('~node_type')
    node_name = rospy.get_param('~node_name')

    # Delay start with countdown
    for i in range(int(start_delay), 0, -1):
        rospy.loginfo(f"Starting control node in {i} seconds...")
        time.sleep(1)

    # Start node
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    launch = roslaunch.parent.ROSLaunchParent(uuid, [roslaunch.rlutil.resolve_launch_arguments([node_pkg, node_type])[0]])
    launch.start()

if __name__ == '__main__':
    try:
        delayed_start()
    except rospy.ROSInterruptException:
        pass
