# plot_circle.py
import matplotlib.pyplot as plt
import numpy as np
import sys
import time
import socket
import struct
import turtle

def plot_circle(radius):
    start_time = time.time()
  
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

    end_time = time.time()
    time_taken = end_time - start_time

    # Broadcast the time taken using socket communication
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        message = struct.pack('!d', time_taken)  # Pack the time taken into a binary format
        sock.sendto(message, ('0.0.0.0', 1000+int(radius)))  # Replace with your broadcast address

if __name__ == "__main__":
    radius = float(sys.argv[1])  # Get the radius from the command-line arguments
    plot_circle(radius)
