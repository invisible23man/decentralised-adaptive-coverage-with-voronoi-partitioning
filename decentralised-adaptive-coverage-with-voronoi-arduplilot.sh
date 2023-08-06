#!/bin/bash
cd ~/Robotics/Simulations/ardupilot/ArduCopter/ && \ 
gnome-terminal \
 --tab -e "sim_vehicle.py -v ArduCopter -f gazebo-drone1 -I0" \
 --tab -e "sim_vehicle.py -v ArduCopter -f gazebo-drone2 -I1" \
 --tab -e "sim_vehicle.py -v ArduCopter -f gazebo-drone3 -I2" \
 --tab -e "sim_vehicle.py -v ArduCopter -f gazebo-drone4 -I3" \
 --tab -e "sim_vehicle.py -v ArduCopter -f gazebo-drone5 -I4" \
 --tab -e "sim_vehicle.py -v ArduCopter -f gazebo-drone6 -I5" 