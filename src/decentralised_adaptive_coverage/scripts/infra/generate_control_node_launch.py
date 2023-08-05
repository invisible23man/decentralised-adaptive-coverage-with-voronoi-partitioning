import sys
sys.path.append('/home/invisible23man/Robotics/Simulations/ardupilot/Tools/autotest/pysim/')

import configparser
import os
import json
from vehicleinfo import VehicleInfo
from generate_world_files import create_world_file


def generate_node_launch_files(num_drones, LAUNCHFILE):
    """
    Generate launch files for control_algorithm_node for each drone.

    Args:_BUILD
        num_drones (int): Number of drones.
        LAUNCHFILE (str): Path to the output launch file.

    Returns:
        None
    """
    launch_file_template = \
    """
    <launch>
        <!-- Include init_voronoi.launch -->
        <!-- <include file="$(find decentralised_adaptive_coverage)/launch/init_voronoi.launch"/> -->

        <!-- Delay start of control_algorithm_node for each drone -->
        <!-- <param name="start_delay" value="5.0" type="double" /> -->

        <!-- Launch control_algorithm_node for each drone -->
        {drone_groups}
    </launch>
    """

    drone_group_template = \
    """
    <group ns="drone{drone_id}">
        <node pkg="decentralised_adaptive_coverage" type="drone_node.py" name="control_algorithm_node_{drone_id_minus_1}" output="screen" respawn="false" respawn_delay="5"/>
            <param name="namespace" value="/drone{drone_id}"/>
            <param name="use_sim_time"  value="true" />
    </group>
    """

    drone_groups = "\n".join(drone_group_template.format(drone_id=i, drone_id_minus_1=i-1) for i in range(1, num_drones + 1))

    with open(LAUNCHFILE, "w") as f:
        f.write(launch_file_template.format(drone_groups=drone_groups))

if __name__ == '__main__':

    num_drones = 8

    NODELAUNCHFILE = '/home/invisible23man/Robotics/Simulations/decentralised-adaptive-coverage-with-voronoi-partitioning/src/decentralised_adaptive_coverage/launch/control_node.launch'

    generate_node_launch_files(num_drones, NODELAUNCHFILE)
