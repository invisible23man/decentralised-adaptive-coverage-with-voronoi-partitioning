import configparser
import os

def generate_launch_file(num_drones, LAUNCHFILE):
    launch_file_template = """
<launch>
    <!-- Include init_voronoi.launch -->
    <!-- <include file="$(find decentralised_adaptive_coverage)/launch/init_voronoi.launch"/> -->

    <!-- Delay start of control_algorithm_node for each drone -->
    <!-- <param name="start_delay" value="5.0" type="double" /> -->

    <!-- Launch control_algorithm_node for each drone -->
    {drone_groups}
</launch>
    """

    drone_group_template = """
<group ns="drone{drone_id}">
    <node pkg="decentralised_adaptive_coverage" type="control_algorithm.py" name="control_algorithm_node_{drone_id_minus_1}" output="screen" respawn="true" respawn_delay="5"/>
        <param name="namespace" value="/drone{drone_id}"/>
        <param name="use_sim_time"  value="true" />
</group>
    """

    drone_groups = "\n".join(drone_group_template.format(drone_id=i, drone_id_minus_1=i-1) for i in range(1, num_drones + 1))

    with open(LAUNCHFILE, "w") as f:
        f.write(launch_file_template.format(drone_groups=drone_groups))

if __name__ == '__main__':

    CONFIGFILE = '/home/invisibleman/Robotics/adaptive-coverage-with-voronoi/src/decentralised_adaptive_coverage/scripts/config.ini'
    LAUNCHFILE = '/home/invisibleman/Robotics/adaptive-coverage-with-voronoi/src/decentralised_adaptive_coverage/launch/control.launch'

    # Read the number of drones from the config file
    config = configparser.ConfigParser()
    config.read(CONFIGFILE)
    num_drones = config.getint('INITIAL_SETUP', 'n_drones')

    generate_launch_file(num_drones, LAUNCHFILE)
