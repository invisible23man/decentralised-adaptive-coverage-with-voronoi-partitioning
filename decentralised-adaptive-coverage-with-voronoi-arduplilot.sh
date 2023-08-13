#!/bin/bash
cd ~/Robotics/Simulations/ardupilot/ArduCopter/ && \ 
gnome-terminal \
 --tab -e "sim_vehicle.py -v ArduCopter -f gazebo-drone1 -I0" \
 --tab -e "sim_vehicle.py -v ArduCopter -f gazebo-drone2 -I1" \
 --tab -e "sim_vehicle.py -v ArduCopter -f gazebo-drone3 -I2" \
 --tab -e "sim_vehicle.py -v ArduCopter -f gazebo-drone4 -I3" \
 --tab -e "sim_vehicle.py -v ArduCopter -f gazebo-drone5 -I4" \
 --tab -e "sim_vehicle.py -v ArduCopter -f gazebo-drone6 -I5" \
 --tab -e "sim_vehicle.py -v ArduCopter -f gazebo-drone7 -I6" \
 --tab -e "sim_vehicle.py -v ArduCopter -f gazebo-drone8 -I7" \
 --tab -e "sim_vehicle.py -v ArduCopter -f gazebo-drone9 -I8" \
 --tab -e "sim_vehicle.py -v ArduCopter -f gazebo-drone10 -I9" \
 --tab -e "sim_vehicle.py -v ArduCopter -f gazebo-drone11 -I10" \
 --tab -e "sim_vehicle.py -v ArduCopter -f gazebo-drone12 -I11" \
 --tab -e "sim_vehicle.py -v ArduCopter -f gazebo-drone13 -I12" \
 --tab -e "sim_vehicle.py -v ArduCopter -f gazebo-drone14 -I13" \
 --tab -e "sim_vehicle.py -v ArduCopter -f gazebo-drone15 -I14" \
 --tab -e "sim_vehicle.py -v ArduCopter -f gazebo-drone16 -I15" 