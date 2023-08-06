#!/bin/bash
cd ~/Robotics/Simulations/ardupilot/ArduCopter/ && \ 
gnome-terminal \
 --tab -e "python sim_vehicle.py -v ArduCopter -f gazebo-drone1 -I0" \
 --tab -e "python sim_vehicle.py -v ArduCopter -f gazebo-drone2 -I1" 