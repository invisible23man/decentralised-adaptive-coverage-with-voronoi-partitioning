import configparser
import os

def generate_node_launch_files(num_drones, LAUNCHFILE):
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
    <node pkg="decentralised_adaptive_coverage" type="control_algorithm.py" name="control_algorithm_node_{drone_id_minus_1}" output="screen" respawn="false" respawn_delay="5"/>
        <param name="namespace" value="/drone{drone_id}"/>
        <param name="use_sim_time"  value="true" />
</group>
    """

    drone_groups = "\n".join(drone_group_template.format(drone_id=i, drone_id_minus_1=i-1) for i in range(1, num_drones + 1))

    with open(LAUNCHFILE, "w") as f:
        f.write(launch_file_template.format(drone_groups=drone_groups))

def generate_mavros_launch_file(num_of_drones, file_path, start_port = 14550):
    launch_content = "<launch>\n"

    for i in range(num_of_drones):
        fcu_port_in = start_port + i * 10
        fcu_port_out = (start_port+5) + i * 10

        launch_content += f"\t<node pkg=\"mavros\" type=\"mavros_node\" name=\"mavros\" required=\"false\" clear_params=\"true\" output=\"screen\" respawn=\"true\" ns=\"/drone{i+1}\">\n"
        launch_content += f"\t\t<param name=\"fcu_url\" value=\"udp://127.0.0.1:{fcu_port_in}@{fcu_port_out}\" />\n"
        launch_content += "\t\t<param name=\"gcs_url\" value=\"\" />\n"
        launch_content += f"\t\t<param name=\"target_system_id\" value=\"{i+1}\" />\n"
        launch_content += "\t\t<param name=\"target_component_id\" value=\"1\" />\n"
        launch_content += "\t\t<param name=\"fcu_protocol\" value=\"v2.0\" />\n"
        launch_content += "\n"
        launch_content += "\t\t<!-- load blacklist, config -->\n"
        launch_content += f"\t\t<rosparam command=\"load\" file=\"$(find mavros)/launch/apm_pluginlists.yaml\" />\n"
        launch_content += f"\t\t<rosparam command=\"load\" file=\"$(find mavros)/launch/apm_config.yaml\" />\n"
        launch_content += "\t</node>\n\n"

    launch_content += "</launch>"

    with open(file_path, 'w') as file:
        file.write(launch_content)

if __name__ == '__main__':

    CONFIGFILE = '/home/invisibleman/Robotics/adaptive-coverage-with-voronoi/src/decentralised_adaptive_coverage/scripts/config.ini'
    LAUNCHFILE = '/home/invisibleman/Robotics/adaptive-coverage-with-voronoi/src/decentralised_adaptive_coverage/launch/control.launch'
    MAVROSLAUNCHFILE = '/home/invisibleman/Robotics/adaptive-coverage-with-voronoi/src/decentralised_adaptive_coverage/launch/multi-apm.launch'
    
    # ArduPilot Global
    PARM_DIR = '/home/invisibleman/Robotics/Simulations/ardupilot/Tools/autotest/default_params'
    VEHICLE_INFO_FILE = '/home/invisibleman/Robotics/Simulations/ardupilot/Tools/autotest/pysim/vehicleinfo.py'
    ARDUPILOT_FILE = '~/multi-ardupilot-world.sh'



    # Read the number of drones from the config file
    config = configparser.ConfigParser()
    config.read(CONFIGFILE)
    num_drones = config.getint('INITIAL_SETUP', 'n_drones')

    generate_node_launch_files(num_drones, LAUNCHFILE)
    generate_mavros_launch_file(num_drones, MAVROSLAUNCHFILE)

