#! /home/invisible23man/anaconda3/envs/drones/bin/ python
import rospy
from std_msgs.msg import Float64
import turtle

def plot_circle(radius):
    start_time = rospy.get_time()
  
    # initialise the turtle instance
    animation = turtle.Turtle()
    
    #creating animation
    # changes speed of turtle
    animation.speed(0)
    
    # hiding turtle 
    animation.hideturtle()
    
    # changing background color
    animation.getscreen().bgcolor("black")
    
    # color of the animation
    animation.color("red")
  
    for i in range(100):
        
        # drawing circle using circle function 
        # by passing radius i
        animation.circle(radius)
    
        # changing turtle face by 5 degree from it's
        # previous position after drawing a circle
        animation._rotate(5)

    end_time = rospy.get_time()
    time_taken = end_time - start_time

    # Publish the time taken
    pub = rospy.Publisher('time_taken', Float64, queue_size=10)
    pub.publish(time_taken)

if __name__ == "__main__":
    rospy.init_node('circle_drawer')
    radius = rospy.get_param('~radius')  # Get the radius from the ROS parameter server
    plot_circle(radius)
