import json
import sys

sys.path.append('/home/invisibleman/Robotics/Simulations/ardupilot/Tools/autotest/pysim/')

import configparser
import os
from vehicleinfo import VehicleInfo


def generate_node_launch_files(num_drones, LAUNCHFILE):
    """
    Generate launch files for control_algorithm_node for each drone.

    Args:
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
        <node pkg="decentralised_adaptive_coverage" type="control_algorithm.py" name="control_algorithm_node_{drone_id_minus_1}" output="screen" respawn="false" respawn_delay="5"/>
            <param name="namespace" value="/drone{drone_id}"/>
            <param name="use_sim_time"  value="true" />
    </group>
    """

    drone_groups = "\n".join(drone_group_template.format(drone_id=i, drone_id_minus_1=i-1) for i in range(1, num_drones + 1))

    with open(LAUNCHFILE, "w") as f:
        f.write(launch_file_template.format(drone_groups=drone_groups))

def generate_mavros_launch_file(num_of_drones, file_path, start_port = 14550):
    """
    Generate MAVROS launch file for each drone.

    Args:
        num_of_drones (int): Number of drones.
        file_path (str): Path to the output launch file.
        start_port (int, optional): Starting UDP port for each drone. Defaults to 14550.

    Returns:
        None
    """
    
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

def add_drones(self, num_drones):
    """
    Add drone definitions to the VehicleInfo class.

    Args:
        num_drones (int): Number of drones.

    Returns:
        None
    """
    
    for i in range(1, num_drones + 1):
        drone_name = f"gazebo-drone{i}"
        self.options["ArduCopter"]["frames"][drone_name] = {
            "waf_target": "bin/arducopter",
            "default_params_filename": ["default_params/copter.parm",
                                        f"default_params/{drone_name}.parm"],
        }

def replace_in_file(filename, text_to_search, replacement_text):
    """
    Replace text in a file.

    Args:
        filename (str): Path to the file.
        text_to_search (str): Text to search for.
        replacement_text (str): Replacement text.

    Returns:
        None
    """
    with open(filename, 'r') as file:
        filedata = file.read()

    # Replace the target string
    filedata = filedata.replace(text_to_search, replacement_text)

    # Write the file out again
    with open(filename, 'w') as file:
        file.write(filedata)

def save_class_to_file(class_instance, file_path):
    """
    Save a class instance to a Python file.

    Args:
        class_instance (object): Class instance to save.
        file_path (str): Path to the output Python file.

    Returns:
        None
    """
    class_name = class_instance.__class__.__name__
    class_dict = class_instance.__dict__

    with open(file_path, 'w') as file:
        file.write(f"class {class_name}(object):\n")
        file.write("\n    def __init__(self):\n")
        file.write('        """\n')
        file.write('        waf_target: option passed to wafs --target to create binary\n')
        file.write('        default_params_filename: filename of default parameters file.  Taken to be relative to autotest dir.\n')
        file.write('        extra_mavlink_cmds: extra parameters that will be passed to mavproxy\n')
        file.write('        """\n')

        for key, value in class_dict.items():
            if isinstance(value, dict):
                value = json.dumps(value, indent=4)  # pretty-print JSON
                # value = value.replace("{", "{ ").replace(": ", ": ").replace(", ", ", ").replace("}", " }").replace("\n", "")
            file.write(f"        self.{key} = {value}\n")

        # Write methods to file
        # for name, method in inspect.getmembers(class_instance, predicate=inspect.ismethod):
        #     if name.startswith("__"):  # Ignore built-in methods
        #         continue
        #     source = inspect.getsource(method)
        #     file.write(f"\n    {source}\n")
            file.write("""
    def default_frame(self, vehicle):
        return self.options[vehicle]["default_frame"]

    def default_waf_target(self, vehicle):
        default_frame = self.default_frame(vehicle)
        return self.options[vehicle]["frames"][default_frame]["waf_target"]

    def options_for_frame(self, frame, vehicle, opts):
        ret = None
        frames = self.options[vehicle]["frames"]
        if frame in frames:
            ret = self.options[vehicle]["frames"][frame]
        else:
            for p in ["octa", "tri", "y6", "firefly", "heli", "gazebo", "last_letter", "jsbsim", "quadplane", "plane-elevon", "plane-vtail", "plane", "airsim"]:
                if frame.startswith(p):
                    ret = self.options[vehicle]["frames"][p]
                    break
        if ret is None:
            if frame.endswith("-heli"):
                ret = self.options[vehicle]["frames"]["heli"]
        if ret is None:
            print("WARNING: no config for frame (%s)" % frame)
            ret = {}

        if "model" not in ret:
            ret["model"] = frame

        if "sitl-port" not in ret:
            ret["sitl-port"] = True

        if opts.model is not None:
            ret["model"] = opts.model

        if (ret["model"].find("xplane") != -1 or ret["model"].find("flightaxis") != -1):
            ret["sitl-port"] = False

        if "waf_target" not in ret:
            ret["waf_target"] = self.default_waf_target(vehicle)

        if opts.build_target is not None:
            ret["waf_target"] = opts.build_target

        return ret
        """)

    replace_in_file(file_path, '"external": true', '"external": True')

def create_ardupilot_vehicle_parameter_file(num_drones, file_path):
    """
    Create ArduPilot vehicle parameter files for each drone.

    Args:
        num_drones (int): Number of drones.
        file_path (str): Path to the output directory.

    Returns:
        None
    """
    
    for drone_id in range(1, num_drones + 1):
        with open(os.path.join(file_path,f"gazebo-drone{drone_id}.parm"), "w") as file:
            file.write("# Iris is X frame\n")
            file.write("FRAME_CLASS 1\n")
            file.write("FRAME_TYPE  1\n")
            file.write("# IRLOCK FEATURE\n")
            file.write("RC8_OPTION 39\n")
            file.write("PLND_ENABLED    1\n")
            file.write("PLND_TYPE       3\n")
            file.write("# SONAR FOR IRLOCK\n")
            file.write("SIM_SONAR_SCALE 10\n")
            file.write("RNGFND1_TYPE 1\n")
            file.write("RNGFND1_SCALING 10\n")
            file.write("RNGFND1_PIN 0\n")
            file.write("RNGFND1_MAX_CM 5000\n")
            file.write(f"SYSID_THISMAV {drone_id}\n")

def add_vehicle_info(num_drones, file_path):
    """
    Add drone information to the VehicleInfo class.

    Args:
        num_drones (int): Number of drones.
        file_path (str): Path to the output Python file.

    Returns:
        None
    """

    # Assuming you've imported the class as VehicleInfo
    VehicleInfo.add_drones = add_drones
    vehicleInfo = VehicleInfo()
    vehicleInfo.add_drones(num_drones)
    save_class_to_file(vehicleInfo, file_path)

def generate_ardupilot_launch_script(num_drones, file_path):
    """
    Generate a bash script to launch ArduPilot for each drone.

    Args:
        num_drones (int): Number of drones.
        file_path (str): Path to the output bash script file.

    Returns:
        str: The generated bash script.
    """
    
    script = "#!/bin/bash\n"
    script += "cd ~/ardupilot/ArduCopter/ && \\\ngnome-terminal \\\n"

    for i in range(num_drones):
        script += f" --tab -e \"sim_vehicle.py -v ArduCopter -f gazebo-drone{i+1} -I{i}\" \\\n"

    # Remove the trailing backslash and newline character
    script = script.rstrip("\\\n")

    with open(os.path.expanduser(file_path), 'w') as file:
        file.write(script)


    return script


if __name__ == '__main__':

    CONFIGFILE = '/home/invisibleman/Robotics/adaptive-coverage-with-voronoi/src/decentralised_adaptive_coverage/scripts/config.ini'
    LAUNCHFILE = '/home/invisibleman/Robotics/adaptive-coverage-with-voronoi/src/decentralised_adaptive_coverage/launch/control.launch'
    MAVROSLAUNCHFILE = '/home/invisibleman/Robotics/adaptive-coverage-with-voronoi/src/decentralised_adaptive_coverage/launch/multi-apm.launch'
    
    # ArduPilot Global
    VEHICLEPARAMETERDIR = '/home/invisibleman/Robotics/Simulations/ardupilot/Tools/autotest/default_params'
    VEHICLEINFOFILE = '/home/invisibleman/Robotics/Simulations/ardupilot/Tools/autotest/pysim/vehicleinfo.py'
    ARDUPILOTLAUNCHFILE = '~/decentralised-adaptive-coverage-with-voronoi-arduplilot.sh'



    # Read the number of drones from the config file
    config = configparser.ConfigParser()
    config.read(CONFIGFILE)
    num_drones = config.getint('INITIAL_SETUP', 'n_drones')

    generate_node_launch_files(num_drones, LAUNCHFILE)
    generate_mavros_launch_file(num_drones, MAVROSLAUNCHFILE)

    create_ardupilot_vehicle_parameter_file(num_drones, VEHICLEPARAMETERDIR)
    add_vehicle_info(num_drones, VEHICLEINFOFILE)
    generate_ardupilot_launch_script(num_drones, ARDUPILOTLAUNCHFILE)


