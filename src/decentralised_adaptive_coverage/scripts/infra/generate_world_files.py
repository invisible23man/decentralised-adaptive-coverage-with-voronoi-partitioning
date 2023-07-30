import os
import argparse

def generate_physics():
    physics = """
        <physics type='ode'>
            <ode>
                <solver>
                    <type>quick</type>
                    <iters>100</iters>
                    <sor>1</sor>
                    <use_dynamic_moi_rescaling>0</use_dynamic_moi_rescaling>
                </solver>
                <constraints>
                    <cfm>0</cfm>
                    <erp>0.9</erp>
                    <contact_max_correcting_vel>0.1</contact_max_correcting_vel>
                    <contact_surface_layer>0</contact_surface_layer>
                </constraints>
            </ode>
            <real_time_update_rate>-1</real_time_update_rate>
            <max_step_size>0.001</max_step_size>
            <real_time_factor>1</real_time_factor>
        </physics>
    """
    return physics

def generate_ground_plane():
    ground_plane = """
        <model name='ground_plane'>
            <static>1</static>
            <link name='link'>
                <collision name='collision'>
                    <geometry>
                        <plane>
                            <normal>0 0 1</normal>
                            <size>5000 5000</size>
                        </plane>
                    </geometry>
                    <surface>
                        <friction>
                            <ode>
                                <mu>1</mu>
                                <mu2>1</mu2>
                            </ode>
                            <torsional>
                                <ode/>
                            </torsional>
                        </friction>
                        <contact>
                            <ode/>
                        </contact>
                        <bounce/>
                    </surface>
                    <max_contacts>10</max_contacts>
                </collision>
                <visual name='runway'>
                    <pose>0 0 0.005 0 -0 0</pose>
                    <cast_shadows>0</cast_shadows>
                    <geometry>
                        <plane>
                            <normal>0 0 1</normal>
                            <size>1829 45</size>
                        </plane>
                    </geometry>
                    <material>
                        <script>
                            <uri>file://media/materials/scripts/gazebo.material</uri>
                            <name>Gazebo/Runway</name>
                        </script>
                    </material>
                </visual>
                <visual name='grass'>
                    <pose>0 0 -0.1 0 -0 0</pose>
                    <cast_shadows>0</cast_shadows>
                    <geometry>
                        <plane>
                            <normal>0 0 1</normal>
                            <size>5000 5000</size>
                        </plane>
                    </geometry>
                    <material>
                        <script>
                            <uri>file://media/materials/scripts/gazebo.material</uri>
                            <name>Gazebo/Grass</name>
                        </script>
                    </material>
                </visual>
                <self_collide>0</self_collide>
                <enable_wind>0</enable_wind>
                <kinematic>0</kinematic>
            </link>
        </model>
    """
    return ground_plane

def generate_light():
    light = """
        <light name='sun' type='directional'>
            <cast_shadows>1</cast_shadows>
            <pose>0 0 10 0 -0 0</pose>
            <diffuse>0.8 0.8 0.8 1</diffuse>
            <specular>0.2 0.2 0.2 1</specular>
            <attenuation>
                <range>1000</range>
                <constant>0.9</constant>
                <linear>0.01</linear>
                <quadratic>0.001</quadratic>
            </attenuation>
            <direction>-0.5 0.1 -0.9</direction>
            <spot>
                <inner_angle>0</inner_angle>
                <outer_angle>0</outer_angle>
                <falloff>0</falloff>
            </spot>
        </light>
    """
    return light

def generate_gravity():
    gravity = """
        <gravity>0 0 -9.8</gravity>
    """
    return gravity

def generate_magnetic_field():
    magnetic_field = """
        <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>
    """
    return magnetic_field

def generate_atmosphere():
    atmosphere = """
        <atmosphere type='adiabatic'/>
    """
    return atmosphere

def generate_scene():
    scene = """
        <scene>
            <ambient>0.4 0.4 0.4 1</ambient>
            <background>0.7 0.7 0.7 1</background>
            <shadows>0</shadows>
        </scene>
    """
    return scene

def generate_wind():
    wind = """
        <wind/>
    """
    return wind

def generate_spherical_coordinates():
    spherical_coordinates = """
        <spherical_coordinates>
            <surface_model>EARTH_WGS84</surface_model>
            <latitude_deg>0</latitude_deg>
            <longitude_deg>0</longitude_deg>
            <elevation>0</elevation>
            <heading_deg>0</heading_deg>
        </spherical_coordinates>
    """
    return spherical_coordinates

def generate_state():
    state = """
        <state world_name='default'>
            <sim_time>21 920000000</sim_time>
            <real_time>47 126797538</real_time>
            <wall_time>1688653734 580412540</wall_time>
            <iterations>15582</iterations>
        </state>
    """
    return state

def generate_gui():
    gui = """
        <gui fullscreen='0'>
            <camera name='user_camera'>
                <pose>53.6425 -71.5855 22.2667 0 0.275643 2.35619</pose>
                <view_controller>orbit</view_controller>
                <projection_type>perspective</projection_type>
            </camera>
        </gui>
    """
    return gui

def generate_drone(drone_id, pose):

    fdm_port_in = 9002 + (drone_id - 1) * 10
    fdm_port_out = fdm_port_in + 1

    drone = f"""
        <model name="drone{drone_id}">
            <pose>{pose}</pose>
            <frame name='iris_demo::__model__' attached_to='iris_demo::iris::base_link'>
                <pose relative_to='__model__'>0 0 0 0 -0 0</pose>
            </frame>
            <frame name='iris_demo::iris::__model__' attached_to='iris_demo::iris::base_link'>
                <pose relative_to='iris_demo::__model__'>0 0 0.194923 0 -0 0</pose>
            </frame>
            <link name='iris_demo::iris::base_link'>
                <velocity_decay>
                <linear>0</linear>
                <angular>0</angular>
                </velocity_decay>
                <inertial>
                <pose>0 0 0 0 -0 0</pose>
                <mass>1.5</mass>
                <inertia>
                    <ixx>0.008</ixx>
                    <ixy>0</ixy>
                    <ixz>0</ixz>
                    <iyy>0.015</iyy>
                    <iyz>0</iyz>
                    <izz>0.017</izz>
                </inertia>
                </inertial>
                <collision name='base_link_collision_fl'>
                <pose>0.13 0.22 -0.08 0 -0 0</pose>
                <geometry>
                    <box>
                    <size>0.05 0.05 0.18</size>
                    </box>
                </geometry>
                <surface>
                    <contact>
                    <ode>
                        <max_vel>100</max_vel>
                        <min_depth>0.001</min_depth>
                    </ode>
                    </contact>
                    <friction>
                    <ode>
                        <mu>1</mu>
                        <mu2>1</mu2>
                    </ode>
                    <torsional>
                        <ode/>
                    </torsional>
                    </friction>
                    <bounce/>
                </surface>
                <max_contacts>10</max_contacts>
                </collision>
                <collision name='base_link_collision_fr'>
                <pose>0.13 -0.22 -0.08 0 -0 0</pose>
                <geometry>
                    <box>
                    <size>0.05 0.05 0.18</size>
                    </box>
                </geometry>
                <surface>
                    <contact>
                    <ode>
                        <max_vel>100</max_vel>
                        <min_depth>0.001</min_depth>
                    </ode>
                    </contact>
                    <friction>
                    <ode>
                        <mu>1</mu>
                        <mu2>1</mu2>
                    </ode>
                    <torsional>
                        <ode/>
                    </torsional>
                    </friction>
                    <bounce/>
                </surface>
                <max_contacts>10</max_contacts>
                </collision>
                <collision name='base_link_collision_br'>
                <pose>-0.13 -0.22 -0.08 0 -0 0</pose>
                <geometry>
                    <box>
                    <size>0.05 0.05 0.18</size>
                    </box>
                </geometry>
                <surface>
                    <contact>
                    <ode>
                        <max_vel>100</max_vel>
                        <min_depth>0.001</min_depth>
                    </ode>
                    </contact>
                    <friction>
                    <ode>
                        <mu>1</mu>
                        <mu2>1</mu2>
                    </ode>
                    <torsional>
                        <ode/>
                    </torsional>
                    </friction>
                    <bounce/>
                </surface>
                <max_contacts>10</max_contacts>
                </collision>
                <collision name='base_link_collision_bl'>
                <pose>-0.13 0.22 -0.08 0 -0 0</pose>
                <geometry>
                    <box>
                    <size>0.05 0.05 0.18</size>
                    </box>
                </geometry>
                <surface>
                    <contact>
                    <ode>
                        <max_vel>100</max_vel>
                        <min_depth>0.001</min_depth>
                    </ode>
                    </contact>
                    <friction>
                    <ode>
                        <mu>1</mu>
                        <mu2>1</mu2>
                    </ode>
                    <torsional>
                        <ode/>
                    </torsional>
                    </friction>
                    <bounce/>
                </surface>
                <max_contacts>10</max_contacts>
                </collision>
                <visual name='base_link_visual'>
                <geometry>
                    <mesh>
                    <uri>model://iris_base/meshes/iris.dae</uri>
                    </mesh>
                </geometry>
                <material>
                    <script>
                    <name>Gazebo/DarkGrey</name>
                    <uri>__default__</uri>
                    </script>
                </material>
                </visual>
                <visual name='front_left_leg_visual'>
                <pose>0.123 0.22 -0.11 0 -0 0</pose>
                <geometry>
                    <cylinder>
                    <radius>0.005</radius>
                    <length>0.17</length>
                    </cylinder>
                </geometry>
                <material>
                    <script>
                    <name>Gazebo/FlatBlack</name>
                    <uri>file://media/materials/scripts/gazebo.material</uri>
                    </script>
                </material>
                </visual>
                <visual name='front_right_leg_visual'>
                <pose>0.123 -0.22 -0.11 0 -0 0</pose>
                <geometry>
                    <cylinder>
                    <radius>0.005</radius>
                    <length>0.17</length>
                    </cylinder>
                </geometry>
                <material>
                    <script>
                    <name>Gazebo/FlatBlack</name>
                    <uri>file://media/materials/scripts/gazebo.material</uri>
                    </script>
                </material>
                </visual>
                <visual name='rear_left_leg_visual'>
                <pose>-0.14 0.21 -0.11 0 -0 0</pose>
                <geometry>
                    <cylinder>
                    <radius>0.005</radius>
                    <length>0.17</length>
                    </cylinder>
                </geometry>
                <material>
                    <script>
                    <name>Gazebo/FlatBlack</name>
                    <uri>file://media/materials/scripts/gazebo.material</uri>
                    </script>
                </material>
                </visual>
                <visual name='rear_right_leg_visual'>
                <pose>-0.14 -0.21 -0.11 0 -0 0</pose>
                <geometry>
                    <cylinder>
                    <radius>0.005</radius>
                    <length>0.17</length>
                    </cylinder>
                </geometry>
                <material>
                    <script>
                    <name>Gazebo/FlatBlack</name>
                    <uri>file://media/materials/scripts/gazebo.material</uri>
                    </script>
                </material>
                </visual>
                <pose relative_to='iris_demo::iris::__model__'>0 0 0 0 -0 0</pose>
                <self_collide>0</self_collide>
                <enable_wind>0</enable_wind>
                <kinematic>0</kinematic>
            </link>
            <link name='iris_demo::iris::iris/ground_truth/odometry_sensorgt_link'>
                <pose relative_to='iris_demo::iris::__model__'>0 0 0 0 -0 0</pose>
                <inertial>
                <pose>0 0 0 0 -0 0</pose>
                <mass>0.15</mass>
                <inertia>
                    <ixx>0.0001</ixx>
                    <ixy>0</ixy>
                    <ixz>0</ixz>
                    <iyy>0.0002</iyy>
                    <iyz>0</iyz>
                    <izz>0.0002</izz>
                </inertia>
                </inertial>
                <self_collide>0</self_collide>
                <enable_wind>0</enable_wind>
                <kinematic>0</kinematic>
            </link>
            <joint name='iris_demo::iris::iris/ground_truth/odometry_sensorgt_joint' type='revolute'>
                <child>iris_demo::iris::iris/ground_truth/odometry_sensorgt_link</child>
                <parent>iris_demo::iris::base_link</parent>
                <axis>
                <xyz expressed_in='iris_demo::iris::__model__'>0 0 1</xyz>
                <limit>
                    <lower>0</lower>
                    <upper>0</upper>
                    <effort>0</effort>
                    <velocity>0</velocity>
                </limit>
                <dynamics>
                    <damping>1</damping>
                    <spring_reference>0</spring_reference>
                    <spring_stiffness>0</spring_stiffness>
                </dynamics>
                </axis>
                <physics>
                <ode>
                    <implicit_spring_damper>1</implicit_spring_damper>
                    <limit>
                    <cfm>0</cfm>
                    <erp>0.2</erp>
                    </limit>
                </ode>
                </physics>
            </joint>
            <link name='iris_demo::iris::iris/imu_link'>
                <inertial>
                <pose>0 0 0 0 -0 0</pose>
                <mass>0.15</mass>
                <inertia>
                    <ixx>1e-05</ixx>
                    <ixy>0</ixy>
                    <ixz>0</ixz>
                    <iyy>2e-05</iyy>
                    <iyz>0</iyz>
                    <izz>2e-05</izz>
                </inertia>
                </inertial>
                <sensor name='imu_sensor' type='imu'>
                <pose>0 0 0 -3.14159 -0 0</pose>
                <always_on>1</always_on>
                <update_rate>1000</update_rate>
                <imu/>
                </sensor>
                <pose relative_to='iris_demo::iris::__model__'>0 0 0 0 -0 0</pose>
                <self_collide>0</self_collide>
                <enable_wind>0</enable_wind>
                <kinematic>0</kinematic>
            </link>
            <joint name='iris_demo::iris::iris/imu_joint' type='revolute'>
                <child>iris_demo::iris::iris/imu_link</child>
                <parent>iris_demo::iris::base_link</parent>
                <axis>
                <xyz expressed_in='iris_demo::iris::__model__'>0 0 1</xyz>
                <limit>
                    <lower>0</lower>
                    <upper>0</upper>
                    <effort>0</effort>
                    <velocity>0</velocity>
                </limit>
                <dynamics>
                    <damping>1</damping>
                    <spring_reference>0</spring_reference>
                    <spring_stiffness>0</spring_stiffness>
                </dynamics>
                </axis>
                <physics>
                <ode>
                    <implicit_spring_damper>1</implicit_spring_damper>
                    <limit>
                    <cfm>0</cfm>
                    <erp>0.2</erp>
                    </limit>
                </ode>
                </physics>
            </joint>
            <link name='iris_demo::iris::rotor_0'>
                <pose relative_to='iris_demo::iris::__model__'>0.13 -0.22 0.023 0 -0 0</pose>
                <inertial>
                <pose>0 0 0 0 -0 0</pose>
                <mass>0.025</mass>
                <inertia>
                    <ixx>9.75e-06</ixx>
                    <ixy>0</ixy>
                    <ixz>0</ixz>
                    <iyy>0.000166704</iyy>
                    <iyz>0</iyz>
                    <izz>0.000167604</izz>
                </inertia>
                </inertial>
                <collision name='rotor_0_collision'>
                <pose>0 0 0 0 -0 0</pose>
                <geometry>
                    <cylinder>
                    <length>0.005</length>
                    <radius>0.1</radius>
                    </cylinder>
                </geometry>
                <surface>
                    <contact>
                    <ode/>
                    </contact>
                    <friction>
                    <ode/>
                    <torsional>
                        <ode/>
                    </torsional>
                    </friction>
                    <bounce/>
                </surface>
                <max_contacts>10</max_contacts>
                </collision>
                <visual name='rotor_0_visual'>
                <pose>0 0 0 0 -0 0</pose>
                <geometry>
                    <mesh>
                    <scale>1 1 1</scale>
                    <uri>model://iris_base/meshes/iris_prop_ccw.dae</uri>
                    </mesh>
                </geometry>
                <material>
                    <script>
                    <name>Gazebo/Blue</name>
                    <uri>__default__</uri>
                    </script>
                </material>
                </visual>
                <gravity>1</gravity>
                <velocity_decay/>
                <self_collide>0</self_collide>
                <enable_wind>0</enable_wind>
                <kinematic>0</kinematic>
            </link>
            <joint name='iris_demo::iris::rotor_0_joint' type='revolute'>
                <child>iris_demo::iris::rotor_0</child>
                <parent>iris_demo::iris::base_link</parent>
                <axis>
                <xyz expressed_in='iris_demo::iris::__model__'>0 0 1</xyz>
                <limit>
                    <lower>-1e+16</lower>
                    <upper>1e+16</upper>
                </limit>
                <dynamics>
                    <damping>0.004</damping>
                    <spring_reference>0</spring_reference>
                    <spring_stiffness>0</spring_stiffness>
                </dynamics>
                </axis>
                <physics>
                <ode>
                    <implicit_spring_damper>1</implicit_spring_damper>
                    <limit>
                    <cfm>0</cfm>
                    <erp>0.2</erp>
                    </limit>
                </ode>
                </physics>
            </joint>
            <link name='iris_demo::iris::rotor_1'>
                <pose relative_to='iris_demo::iris::__model__'>-0.13 0.2 0.023 0 -0 0</pose>
                <inertial>
                <pose>0 0 0 0 -0 0</pose>
                <mass>0.025</mass>
                <inertia>
                    <ixx>9.75e-06</ixx>
                    <ixy>0</ixy>
                    <ixz>0</ixz>
                    <iyy>0.000166704</iyy>
                    <iyz>0</iyz>
                    <izz>0.000167604</izz>
                </inertia>
                </inertial>
                <collision name='rotor_1_collision'>
                <pose>0 0 0 0 -0 0</pose>
                <geometry>
                    <cylinder>
                    <length>0.005</length>
                    <radius>0.1</radius>
                    </cylinder>
                </geometry>
                <surface>
                    <contact>
                    <ode/>
                    </contact>
                    <friction>
                    <ode/>
                    <torsional>
                        <ode/>
                    </torsional>
                    </friction>
                    <bounce/>
                </surface>
                <max_contacts>10</max_contacts>
                </collision>
                <visual name='rotor_1_visual'>
                <pose>0 0 0 0 -0 0</pose>
                <geometry>
                    <mesh>
                    <scale>1 1 1</scale>
                    <uri>model://iris_base/meshes/iris_prop_ccw.dae</uri>
                    </mesh>
                </geometry>
                <material>
                    <script>
                    <name>Gazebo/DarkGrey</name>
                    <uri>__default__</uri>
                    </script>
                </material>
                </visual>
                <gravity>1</gravity>
                <velocity_decay/>
                <self_collide>0</self_collide>
                <enable_wind>0</enable_wind>
                <kinematic>0</kinematic>
            </link>
            <joint name='iris_demo::iris::rotor_1_joint' type='revolute'>
                <child>iris_demo::iris::rotor_1</child>
                <parent>iris_demo::iris::base_link</parent>
                <axis>
                <xyz expressed_in='iris_demo::iris::__model__'>0 0 1</xyz>
                <limit>
                    <lower>-1e+16</lower>
                    <upper>1e+16</upper>
                </limit>
                <dynamics>
                    <damping>0.004</damping>
                    <spring_reference>0</spring_reference>
                    <spring_stiffness>0</spring_stiffness>
                </dynamics>
                </axis>
                <physics>
                <ode>
                    <implicit_spring_damper>1</implicit_spring_damper>
                    <limit>
                    <cfm>0</cfm>
                    <erp>0.2</erp>
                    </limit>
                </ode>
                </physics>
            </joint>
            <link name='iris_demo::iris::rotor_2'>
                <pose relative_to='iris_demo::iris::__model__'>0.13 0.22 0.023 0 -0 0</pose>
                <inertial>
                <pose>0 0 0 0 -0 0</pose>
                <mass>0.025</mass>
                <inertia>
                    <ixx>9.75e-06</ixx>
                    <ixy>0</ixy>
                    <ixz>0</ixz>
                    <iyy>0.000166704</iyy>
                    <iyz>0</iyz>
                    <izz>0.000167604</izz>
                </inertia>
                </inertial>
                <collision name='rotor_2_collision'>
                <pose>0 0 0 0 -0 0</pose>
                <geometry>
                    <cylinder>
                    <length>0.005</length>
                    <radius>0.1</radius>
                    </cylinder>
                </geometry>
                <surface>
                    <contact>
                    <ode/>
                    </contact>
                    <friction>
                    <ode/>
                    <torsional>
                        <ode/>
                    </torsional>
                    </friction>
                    <bounce/>
                </surface>
                <max_contacts>10</max_contacts>
                </collision>
                <visual name='rotor_2_visual'>
                <pose>0 0 0 0 -0 0</pose>
                <geometry>
                    <mesh>
                    <scale>1 1 1</scale>
                    <uri>model://iris_base/meshes/iris_prop_cw.dae</uri>
                    </mesh>
                </geometry>
                <material>
                    <script>
                    <name>Gazebo/Blue</name>
                    <uri>__default__</uri>
                    </script>
                </material>
                </visual>
                <gravity>1</gravity>
                <velocity_decay/>
                <self_collide>0</self_collide>
                <enable_wind>0</enable_wind>
                <kinematic>0</kinematic>
            </link>
            <joint name='iris_demo::iris::rotor_2_joint' type='revolute'>
                <child>iris_demo::iris::rotor_2</child>
                <parent>iris_demo::iris::base_link</parent>
                <axis>
                <xyz expressed_in='iris_demo::iris::__model__'>0 0 1</xyz>
                <limit>
                    <lower>-1e+16</lower>
                    <upper>1e+16</upper>
                </limit>
                <dynamics>
                    <damping>0.004</damping>
                    <spring_reference>0</spring_reference>
                    <spring_stiffness>0</spring_stiffness>
                </dynamics>
                </axis>
                <physics>
                <ode>
                    <implicit_spring_damper>1</implicit_spring_damper>
                    <limit>
                    <cfm>0</cfm>
                    <erp>0.2</erp>
                    </limit>
                </ode>
                </physics>
            </joint>
            <link name='iris_demo::iris::rotor_3'>
                <pose relative_to='iris_demo::iris::__model__'>-0.13 -0.2 0.023 0 -0 0</pose>
                <inertial>
                <pose>0 0 0 0 -0 0</pose>
                <mass>0.025</mass>
                <inertia>
                    <ixx>9.75e-06</ixx>
                    <ixy>0</ixy>
                    <ixz>0</ixz>
                    <iyy>0.000166704</iyy>
                    <iyz>0</iyz>
                    <izz>0.000167604</izz>
                </inertia>
                </inertial>
                <collision name='rotor_3_collision'>
                <pose>0 0 0 0 -0 0</pose>
                <geometry>
                    <cylinder>
                    <length>0.005</length>
                    <radius>0.1</radius>
                    </cylinder>
                </geometry>
                <surface>
                    <contact>
                    <ode/>
                    </contact>
                    <friction>
                    <ode/>
                    <torsional>
                        <ode/>
                    </torsional>
                    </friction>
                    <bounce/>
                </surface>
                <max_contacts>10</max_contacts>
                </collision>
                <visual name='rotor_3_visual'>
                <pose>0 0 0 0 -0 0</pose>
                <geometry>
                    <mesh>
                    <scale>1 1 1</scale>
                    <uri>model://iris_base/meshes/iris_prop_cw.dae</uri>
                    </mesh>
                </geometry>
                <material>
                    <script>
                    <name>Gazebo/DarkGrey</name>
                    <uri>__default__</uri>
                    </script>
                </material>
                </visual>
                <gravity>1</gravity>
                <velocity_decay/>
                <self_collide>0</self_collide>
                <enable_wind>0</enable_wind>
                <kinematic>0</kinematic>
            </link>
            <joint name='iris_demo::iris::rotor_3_joint' type='revolute'>
                <child>iris_demo::iris::rotor_3</child>
                <parent>iris_demo::iris::base_link</parent>
                <axis>
                <xyz expressed_in='iris_demo::iris::__model__'>0 0 1</xyz>
                <limit>
                    <lower>-1e+16</lower>
                    <upper>1e+16</upper>
                </limit>
                <dynamics>
                    <damping>0.004</damping>
                    <spring_reference>0</spring_reference>
                    <spring_stiffness>0</spring_stiffness>
                </dynamics>
                </axis>
                <physics>
                <ode>
                    <implicit_spring_damper>1</implicit_spring_damper>
                    <limit>
                    <cfm>0</cfm>
                    <erp>0.2</erp>
                    </limit>
                </ode>
                </physics>
            </joint>
            <static>0</static>
            <plugin name='rotor_0_blade_1' filename='libLiftDragPlugin.so'>
                <a0>0.3</a0>
                <alpha_stall>1.4</alpha_stall>
                <cla>4.2500</cla>
                <cda>0.10</cda>
                <cma>0.00</cma>
                <cla_stall>-0.025</cla_stall>
                <cda_stall>0.0</cda_stall>
                <cma_stall>0.0</cma_stall>
                <area>0.002</area>
                <air_density>1.2041</air_density>
                <cp>0.084 0 0</cp>
                <forward>0 1 0</forward>
                <upward>0 0 1</upward>
                <link_name>iris_demo::iris::rotor_0</link_name>
            </plugin>
            <plugin name='rotor_0_blade_2' filename='libLiftDragPlugin.so'>
                <a0>0.3</a0>
                <alpha_stall>1.4</alpha_stall>
                <cla>4.2500</cla>
                <cda>0.10</cda>
                <cma>0.00</cma>
                <cla_stall>-0.025</cla_stall>
                <cda_stall>0.0</cda_stall>
                <cma_stall>0.0</cma_stall>
                <area>0.002</area>
                <air_density>1.2041</air_density>
                <cp>-0.084 0 0</cp>
                <forward>0 -1 0</forward>
                <upward>0 0 1</upward>
                <link_name>iris_demo::iris::rotor_0</link_name>
            </plugin>
            <plugin name='rotor_1_blade_1' filename='libLiftDragPlugin.so'>
                <a0>0.3</a0>
                <alpha_stall>1.4</alpha_stall>
                <cla>4.2500</cla>
                <cda>0.10</cda>
                <cma>0.00</cma>
                <cla_stall>-0.025</cla_stall>
                <cda_stall>0.0</cda_stall>
                <cma_stall>0.0</cma_stall>
                <area>0.002</area>
                <air_density>1.2041</air_density>
                <cp>0.084 0 0</cp>
                <forward>0 1 0</forward>
                <upward>0 0 1</upward>
                <link_name>iris_demo::iris::rotor_1</link_name>
            </plugin>
            <plugin name='rotor_1_blade_2' filename='libLiftDragPlugin.so'>
                <a0>0.3</a0>
                <alpha_stall>1.4</alpha_stall>
                <cla>4.2500</cla>
                <cda>0.10</cda>
                <cma>0.00</cma>
                <cla_stall>-0.025</cla_stall>
                <cda_stall>0.0</cda_stall>
                <cma_stall>0.0</cma_stall>
                <area>0.002</area>
                <air_density>1.2041</air_density>
                <cp>-0.084 0 0</cp>
                <forward>0 -1 0</forward>
                <upward>0 0 1</upward>
                <link_name>iris_demo::iris::rotor_1</link_name>
            </plugin>
            <plugin name='rotor_2_blade_1' filename='libLiftDragPlugin.so'>
                <a0>0.3</a0>
                <alpha_stall>1.4</alpha_stall>
                <cla>4.2500</cla>
                <cda>0.10</cda>
                <cma>0.00</cma>
                <cla_stall>-0.025</cla_stall>
                <cda_stall>0.0</cda_stall>
                <cma_stall>0.0</cma_stall>
                <area>0.002</area>
                <air_density>1.2041</air_density>
                <cp>0.084 0 0</cp>
                <forward>0 -1 0</forward>
                <upward>0 0 1</upward>
                <link_name>iris_demo::iris::rotor_2</link_name>
            </plugin>
            <plugin name='rotor_2_blade_2' filename='libLiftDragPlugin.so'>
                <a0>0.3</a0>
                <alpha_stall>1.4</alpha_stall>
                <cla>4.2500</cla>
                <cda>0.10</cda>
                <cma>0.00</cma>
                <cla_stall>-0.025</cla_stall>
                <cda_stall>0.0</cda_stall>
                <cma_stall>0.0</cma_stall>
                <area>0.002</area>
                <air_density>1.2041</air_density>
                <cp>-0.084 0 0</cp>
                <forward>0 1 0</forward>
                <upward>0 0 1</upward>
                <link_name>iris_demo::iris::rotor_2</link_name>
            </plugin>
            <plugin name='rotor_3_blade_1' filename='libLiftDragPlugin.so'>
                <a0>0.3</a0>
                <alpha_stall>1.4</alpha_stall>
                <cla>4.2500</cla>
                <cda>0.10</cda>
                <cma>0.00</cma>
                <cla_stall>-0.025</cla_stall>
                <cda_stall>0.0</cda_stall>
                <cma_stall>0.0</cma_stall>
                <area>0.002</area>
                <air_density>1.2041</air_density>
                <cp>0.084 0 0</cp>
                <forward>0 -1 0</forward>
                <upward>0 0 1</upward>
                <link_name>iris_demo::iris::rotor_3</link_name>
            </plugin>
            <plugin name='rotor_3_blade_2' filename='libLiftDragPlugin.so'>
                <a0>0.3</a0>
                <alpha_stall>1.4</alpha_stall>
                <cla>4.2500</cla>
                <cda>0.10</cda>
                <cma>0.00</cma>
                <cla_stall>-0.025</cla_stall>
                <cda_stall>0.0</cda_stall>
                <cma_stall>0.0</cma_stall>
                <area>0.002</area>
                <air_density>1.2041</air_density>
                <cp>-0.084 0 0</cp>
                <forward>0 1 0</forward>
                <upward>0 0 1</upward>
                <link_name>iris_demo::iris::rotor_3</link_name>
            </plugin>
            <plugin name='arducopter_plugin' filename='libArduPilotPlugin.so'>
                <fdm_addr>127.0.0.1</fdm_addr>
                <fdm_port_in>{fdm_port_in}</fdm_port_in>
                <fdm_port_out>{fdm_port_out}</fdm_port_out>
                <modelXYZToAirplaneXForwardZDown>0 0 0 3.141593 0 0</modelXYZToAirplaneXForwardZDown>
                <gazeboXYZToNED>0 0 0 3.141593 0 0</gazeboXYZToNED>
                <imuName>iris_demo::iris::iris/imu_link::imu_sensor</imuName>
                <connectionTimeoutMaxCount>5</connectionTimeoutMaxCount>
                <control channel='0'>
                <type>VELOCITY</type>
                <offset>0</offset>
                <p_gain>0.20</p_gain>
                <i_gain>0</i_gain>
                <d_gain>0</d_gain>
                <i_max>0</i_max>
                <i_min>0</i_min>
                <cmd_max>2.5</cmd_max>
                <cmd_min>-2.5</cmd_min>
                <jointName>iris_demo::iris::rotor_0_joint</jointName>
                <multiplier>838</multiplier>
                <controlVelocitySlowdownSim>1</controlVelocitySlowdownSim>
                </control>
                <control channel='1'>
                <type>VELOCITY</type>
                <offset>0</offset>
                <p_gain>0.20</p_gain>
                <i_gain>0</i_gain>
                <d_gain>0</d_gain>
                <i_max>0</i_max>
                <i_min>0</i_min>
                <cmd_max>2.5</cmd_max>
                <cmd_min>-2.5</cmd_min>
                <jointName>iris_demo::iris::rotor_1_joint</jointName>
                <multiplier>838</multiplier>
                <controlVelocitySlowdownSim>1</controlVelocitySlowdownSim>
                </control>
                <control channel='2'>
                <type>VELOCITY</type>
                <offset>0</offset>
                <p_gain>0.20</p_gain>
                <i_gain>0</i_gain>
                <d_gain>0</d_gain>
                <i_max>0</i_max>
                <i_min>0</i_min>
                <cmd_max>2.5</cmd_max>
                <cmd_min>-2.5</cmd_min>
                <jointName>iris_demo::iris::rotor_2_joint</jointName>
                <multiplier>-838</multiplier>
                <controlVelocitySlowdownSim>1</controlVelocitySlowdownSim>
                </control>
                <control channel='3'>
                <type>VELOCITY</type>
                <offset>0</offset>
                <p_gain>0.20</p_gain>
                <i_gain>0</i_gain>
                <d_gain>0</d_gain>
                <i_max>0</i_max>
                <i_min>0</i_min>
                <cmd_max>2.5</cmd_max>
                <cmd_min>-2.5</cmd_min>
                <jointName>iris_demo::iris::rotor_3_joint</jointName>
                <multiplier>-838</multiplier>
                <controlVelocitySlowdownSim>1</controlVelocitySlowdownSim>
                </control>
            </plugin>
        </model>
    """

    return drone

def generate_drones_in_grid(area_size, num_drones, distance=3, pose=[0,1.57,0]):
    drones = []

    # Calculate the number of drones in each row and column
    num_rows = int(area_size / distance)
    num_columns = int(area_size / distance)

    for i in range(num_drones):
        row = i % num_rows
        column = (i // num_columns) % num_columns

        x = row * distance
        y = column * distance
        pose = f"{x} {y} 0 0 -0 {pose[2]}"  # Replace with the desired initial pose values

        drone = generate_drone(i + 1, pose)
        drones.append(drone)

    return drones

def generate_coverage_area(coverage_radii):
    coverage_area = f"""
        <model name="circular_border">
        <pose>5 5 10 0 0 0</pose>
        <link name="link">
            <visual name="visual">
            <geometry>
                <cylinder>
                <radius>{coverage_radii}</radius>
                <length>1</length>
                </cylinder>
            </geometry>
            <material>
                <ambient>1 0 0 1</ambient>
                <diffuse>1 0 0 1</diffuse>
                <specular>1 1 1 1</specular>
                <emissive>0 0 0 0</emissive>
                <shader type="vertex" name="default">
                <normal_map>__default__</normal_map>
                </shader>
            </material>
            </visual>
        </link>
        </model>
    """
    return coverage_area

def create_world_file(num_of_drones, home_area, coverage_radii, file_path):
    world_file = open(file_path, "w")

    world_file.write("<sdf version='1.7'>\n")
    world_file.write("<world name='default'>\n")
    world_file.write(generate_physics())
    world_file.write(generate_ground_plane())
    world_file.write(generate_light())

    # world_file.write(generate_coverage_area(coverage_radii))
    for drone in generate_drones_in_grid(home_area, num_of_drones):
        world_file.write(drone)

    world_file.write(generate_gravity())
    world_file.write(generate_magnetic_field())
    world_file.write(generate_atmosphere())
    world_file.write(generate_scene())
    world_file.write(generate_wind())
    world_file.write(generate_spherical_coordinates())
    world_file.write(generate_state())
    world_file.write(generate_gui())

    # Add more elements here if needed

    world_file.write("</world>\n")
    world_file.write("</sdf>\n")

    world_file.close()

if __name__ == '__main__':
    # Create the parser
    parser = argparse.ArgumentParser(description='Get Arguments for the run')

    # Add the arguments
    parser.add_argument('num_of_drones', metavar='N', type=int, help='The number of drones')
    parser.add_argument('drone_placement_area', metavar='N', type=int, help='The area to arrange drones')

    # Parse the arguments
    args = parser.parse_args()

    # Call the function with a specific num of drones
    NUM_OF_DRONES = args.num_of_drones
    AREA_SIZE = args.drone_placement_area


    WORLD_FILE_DIR = r'/home/invisible23man/catkin_ws/src/iq_sim/worlds'
    WORLD_FILE_NAME = 'multi_drone_world.world'

    # Call the function
    create_world_file(NUM_OF_DRONES, AREA_SIZE, os.path.join(WORLD_FILE_DIR,WORLD_FILE_NAME))

