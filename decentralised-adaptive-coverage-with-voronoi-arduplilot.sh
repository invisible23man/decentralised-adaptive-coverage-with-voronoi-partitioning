#!/bin/bash
cd ~/Robotics/Simulations/ardupilot/Tools/autotest && \ 
gnome-terminal \
 --tab -e "python sim_vehicle.py -v ArduCopter -f gazebo-drone1 -I0" \
 --tab -e "python sim_vehicle.py -v ArduCopter -f gazebo-drone2 -I1" \
 --tab -e "python sim_vehicle.py -v ArduCopter -f gazebo-drone3 -I2" \
 --tab -e "python sim_vehicle.py -v ArduCopter -f gazebo-drone4 -I3" \
 --tab -e "python sim_vehicle.py -v ArduCopter -f gazebo-drone5 -I4" 