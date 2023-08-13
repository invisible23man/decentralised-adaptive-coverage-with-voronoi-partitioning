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
            <sky>
                <clouds>
                    <speed>12</speed>
                </clouds>
            </sky>
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

def generate_drones_at_home(num_drones, x_sep=10, y_ext=6, drone_sep=3):
    drones = []

    drone_in_a_row = 2*y_ext//drone_sep + 1
    x_positions = [x_sep + (drone_sep * (i // drone_in_a_row)) for i in range(num_drones)]
    y_positions = [-y_ext + (drone_sep * (i % drone_in_a_row)) \
                   if i % (drone_in_a_row * 2) < drone_in_a_row \
                    else (i % (drone_in_a_row * 2) - drone_in_a_row) * drone_sep \
                      for i in range(num_drones)]
    for x, y in zip(x_positions, y_positions):
        pose = f"{x} {y} 0 0 -0 0"  # Replace with the desired initial pose values
        print(pose)
        drone = generate_drone(len(drones) + 1, pose)
        drones.append(drone)

    print(len(x_positions), len(y_positions))

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

def generate_heightmap():
    heightmap ="""
    <model name="heightmap">
      <static>true</static>
      <link name="link">
        <visual name="visual_abcedf">
          <geometry>
            <heightmap>
              <use_terrain_paging>false</use_terrain_paging>
              <texture>
                <diffuse>file://materials/textures/grass_color.jpg</diffuse>
                <normal>file://materials/textures/grass_normal.jpg</normal>
                <size>4</size>
              </texture>
              <texture>
                <diffuse>file://materials/textures/dirt_diffusespecular.png</diffuse>
                <normal>file://materials/textures/flat_normal.png</normal>
                <size>1</size>
              </texture>
              <!-- <texture>
                <diffuse>file://materials/textures/grass_color.jpg</diffuse>
                <normal>file://materials/textures/grass_normal.jpg</normal>
                <size>4</size>
              </texture> -->
              <!-- Without this unused 3. texture the blend between the first 2 doesn't work -->
              <texture>
                <diffuse>file://materials/textures/fungus_diffusespecular.png</diffuse>
                <normal>file://materials/textures/flat_normal.png</normal>
                <size>1</size>
              </texture>
              <blend>
                <min_height>0.05</min_height>
                <fade_dist>0.05</fade_dist>
              </blend>
              <!-- <blend>
                <min_height>0.3</min_height>
                <fade_dist>0.05</fade_dist>
              </blend> -->
              <!-- Without this unused 3. texture the blend between the first 2 doesn't work -->
              <blend>
                <min_height>4</min_height>
                <fade_dist>5</fade_dist>
              </blend>
              <uri>file://virtual_maize_field_heightmap.png</uri>
              <size>70.19999999999999 70.19999999999999 0.325</size>
              <pos>0 0 0</pos>
            </heightmap>
          </geometry>
        </visual>
        <collision name="collision">
          <geometry>
            <heightmap>
              <uri>file://virtual_maize_field_heightmap.png</uri>
              <size>70.19999999999999 70.19999999999999 0.325</size>
              <pos>0 0 0</pos>
            </heightmap>
          </geometry>
        </collision>
      </link>
    </model>
    """

    return heightmap

def generate_crops_weeds():
    crops_weeds = """
    <include>
      <uri>model://maize_02</uri>
      <pose>-3.4254440085902034 -5.112269776325877 0.29114602237360276 0.0 0.0 4.189228512427175</pose>
      <name>maize_02_0000</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>-3.4195125859749633 -4.9435613044554305 0.29114602237360276 0.0 0.0 4.622520843543169</pose>
      <name>maize_02_0001</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>-3.4477525448304123 -4.77856715559359 0.29114602237360276 0.0 0.0 0.8406081520459314</pose>
      <name>maize_02_0002</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>-3.4355126263620006 -4.591575281470537 0.29114602237360276 0.0 0.0 5.89125211034871</pose>
      <name>maize_02_0003</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>-3.446650229581284 -4.40434162933074 0.29114602237360276 0.0 0.0 3.3632240180258623</pose>
      <name>maize_02_0004</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>-3.441680806734887 -4.259683530738143 0.29114602237360276 0.0 0.0 3.4155436769742846</pose>
      <name>maize_02_0005</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>-3.4431144721086815 -4.071138566038772 0.29114602237360276 0.0 0.0 0.45244207715678775</pose>
      <name>maize_02_0006</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>-3.4222796187572255 -3.903547499399078 0.29114602237360276 0.0 0.0 4.964239017161655</pose>
      <name>maize_02_0007</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>-3.447252553036442 -3.7303187640337363 0.29114602237360276 0.0 0.0 0.4369635486807079</pose>
      <name>maize_02_0008</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>-3.433464941092455 -3.5722264819841385 0.29114602237360276 0.0 0.0 5.356701053296287</pose>
      <name>maize_02_0009</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>-3.4342778464943517 -3.4229918685244227 0.29114602237360276 0.0 0.0 4.215639836688481</pose>
      <name>maize_02_0010</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>-3.429126566224097 -3.27065858974373 0.29114602237360276 0.0 0.0 1.4318226854807625</pose>
      <name>maize_01_0011</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>-3.4327299210531677 -3.0843559339974793 0.29114602237360276 0.0 0.0 1.2118715441304933</pose>
      <name>maize_01_0012</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>-3.4292772798236504 -2.8910948587460763 0.29114602237360276 0.0 0.0 0.5170837342371754</pose>
      <name>maize_02_0013</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>-3.4531968537824316 -2.7456653363730843 0.29114602237360276 0.0 0.0 3.1378123403316716</pose>
      <name>maize_01_0014</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>-3.4288446519693307 -2.551135534979884 0.29114602237360276 0.0 0.0 2.1239591118787557</pose>
      <name>maize_01_0015</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>-3.425721156866524 -2.4168076133033685 0.29114602237360276 0.0 0.0 5.351850451760419</pose>
      <name>maize_01_0016</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>-3.4315249268168513 -2.282564516769656 0.29114602237360276 0.0 0.0 5.614107499804609</pose>
      <name>maize_01_0017</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>-3.4337563789960965 -2.1248847886742 0.29114602237360276 0.0 0.0 6.00643487400797</pose>
      <name>maize_02_0018</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>-3.4329117305641246 -1.9597778376679686 0.29114602237360276 0.0 0.0 4.406267287089308</pose>
      <name>maize_02_0019</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>-3.450846395817471 -1.7716113553991812 0.29114602237360276 0.0 0.0 5.520816743039335</pose>
      <name>maize_01_0020</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>-3.4469503824209347 -1.6231937917547983 0.29114602237360276 0.0 0.0 5.571353019748994</pose>
      <name>maize_02_0021</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>-3.452342507126994 -1.432109518039864 0.29114602237360276 0.0 0.0 4.261706783235389</pose>
      <name>maize_01_0022</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>-3.4173508387425824 -1.301909620284114 0.29114602237360276 0.0 0.0 0.4572452395177467</pose>
      <name>maize_01_0023</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>-3.4471463125128388 -1.1271120608674643 0.29114602237360276 0.0 0.0 1.0177824604492534</pose>
      <name>maize_02_0024</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>-3.419095089269575 -0.971047418439805 0.29114602237360276 0.0 0.0 0.3567117829917271</pose>
      <name>maize_02_0025</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>-3.415170426010835 -0.8055288173377937 0.29114602237360276 0.0 0.0 2.837668153762604</pose>
      <name>maize_01_0026</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>-3.435055321496364 -0.645196721379782 0.29114602237360276 0.0 0.0 4.725128411591607</pose>
      <name>maize_01_0027</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>-3.424731712762916 -0.4823040060023436 0.29114602237360276 0.0 0.0 4.3225770120880584</pose>
      <name>maize_01_0028</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>-3.4504755648654712 -0.3120315991908935 0.29114602237360276 0.0 0.0 0.15739609126557166</pose>
      <name>maize_02_0029</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>-3.416258277041176 -0.15586004719375968 0.29114602237360276 0.0 0.0 4.500833942782054</pose>
      <name>maize_02_0030</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>-3.4317756243828583 -0.012191562430152736 0.29114602237360276 0.0 0.0 3.168207535268002</pose>
      <name>maize_01_0031</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>-3.4246451710264076 0.1207660618550479 0.29114602237360276 0.0 0.0 1.3988757655451873</pose>
      <name>maize_02_0032</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>-3.445001082938014 0.2658797687618186 0.29114602237360276 0.0 0.0 2.9213929635368765</pose>
      <name>maize_02_0033</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>-3.432323470437327 0.4506967834488558 0.29114602237360276 0.0 0.0 3.0899716154008297</pose>
      <name>maize_02_0034</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>-3.429608143904851 0.6435403512921312 0.29114602237360276 0.0 0.0 4.059580355818467</pose>
      <name>maize_02_0035</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>-3.421075903960848 0.7721218296531278 0.29114602237360276 0.0 0.0 2.622682695246801</pose>
      <name>maize_01_0036</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>-3.4302147716578437 0.9128436734522571 0.29114602237360276 0.0 0.0 3.392964274386464</pose>
      <name>maize_01_0037</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>-3.4443940746428847 1.0454258697458343 0.29114602237360276 0.0 0.0 5.88222867594634</pose>
      <name>maize_02_0038</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>-3.432033904171113 1.1920896888980694 0.29114602237360276 0.0 0.0 5.697811369998041</pose>
      <name>maize_01_0039</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>-3.4219812397410307 1.3814272237671261 0.29114602237360276 0.0 0.0 4.19083554746248</pose>
      <name>maize_01_0040</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>-3.430036392504887 1.5124418544034066 0.29114602237360276 0.0 0.0 1.0675179373393058</pose>
      <name>maize_01_0041</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>-3.450837524553273 1.6903363811906065 0.29114602237360276 0.0 0.0 6.069295648670322</pose>
      <name>maize_02_0042</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>-3.428466193906621 1.8691558319705415 0.29114602237360276 0.0 0.0 0.15080573777202005</pose>
      <name>maize_01_0043</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>-3.4182981191189685 2.0132654997985693 0.29114602237360276 0.0 0.0 5.335287410227084</pose>
      <name>maize_01_0044</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>-3.4347389047517396 2.1712452556937 0.29114602237360276 0.0 0.0 1.7270543910172589</pose>
      <name>maize_02_0045</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>-3.439370925664352 2.290319527519255 0.29114602237360276 0.0 0.0 3.391542689247149</pose>
      <name>maize_01_0046</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>-3.441997843468314 2.4600420979704456 0.29114602237360276 0.0 0.0 0.3763771780382184</pose>
      <name>maize_01_0047</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>-3.4463924086654845 2.5924936507666576 0.29114602237360276 0.0 0.0 0.9889440817129096</pose>
      <name>maize_01_0048</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>-3.4285459685875543 2.7429610076103295 0.29114602237360276 0.0 0.0 4.745432800129673</pose>
      <name>maize_01_0049</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>-3.435825260347039 2.8887529862539134 0.29114602237360276 0.0 0.0 2.7153876791491722</pose>
      <name>maize_02_0050</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>-3.4412462058140068 3.0450807318300397 0.29114602237360276 0.0 0.0 3.311902734799959</pose>
      <name>maize_02_0051</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>-3.430054819580302 3.2098609170854155 0.29114602237360276 0.0 0.0 5.666390336336104</pose>
      <name>maize_01_0052</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>-3.4510161943854967 3.3532009889532697 0.29114602237360276 0.0 0.0 0.21579933407967425</pose>
      <name>maize_02_0053</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>-3.41381079079604 3.5743850578212806 0.29114602237360276 0.0 0.0 1.7157278661535644</pose>
      <name>maize_01_0054</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>-3.4395738648083576 3.763390964988422 0.29114602237360276 0.0 0.0 1.5471259899027723</pose>
      <name>maize_01_0055</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>-3.429878934759434 3.886984178521728 0.29114602237360276 0.0 0.0 0.6471235398245595</pose>
      <name>maize_02_0056</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>-3.416828516043822 4.099662143227818 0.29114602237360276 0.0 0.0 5.435842640902399</pose>
      <name>maize_01_0057</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>-3.452206642545608 4.2537837497747715 0.29114602237360276 0.0 0.0 4.123160271508832</pose>
      <name>maize_01_0058</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>-3.42364667826404 4.433992195633508 0.29114602237360276 0.0 0.0 3.5567299341225915</pose>
      <name>maize_01_0059</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>-3.414357924469981 4.596422662133418 0.29114602237360276 0.0 0.0 0.7128783301954479</pose>
      <name>maize_02_0060</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>-3.4457669623897447 4.730646620310224 0.29114602237360276 0.0 0.0 3.2616899848589394</pose>
      <name>maize_02_0061</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>-3.445414988953393 4.886247484636514 0.29114602237360276 0.0 0.0 0.4856341696735002</pose>
      <name>maize_02_0062</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>-3.4267771181285855 5.037600948578164 0.29114602237360276 0.0 0.0 3.4702020337714203</pose>
      <name>maize_01_0063</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>-2.683971512058474 -5.106261925477736 0.2907688925864437 0.0 0.0 5.166477639918837</pose>
      <name>maize_02_0064</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>-2.6863868386849172 -4.949024547957412 0.2907688925864437 0.0 0.0 5.208634502805326</pose>
      <name>maize_02_0065</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>-2.6910689992097225 -4.7821893016826245 0.2907688925864437 0.0 0.0 1.8545852396439158</pose>
      <name>maize_02_0066</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>-2.691992769836883 -4.597230547985172 0.2907688925864437 0.0 0.0 4.91109589506402</pose>
      <name>maize_02_0067</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>-2.6853185507073274 -4.448990913001069 0.2907688925864437 0.0 0.0 3.392232194528641</pose>
      <name>maize_01_0068</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>-2.701526441157828 -4.2694344681505525 0.2907688925864437 0.0 0.0 2.911065173773778</pose>
      <name>maize_01_0069</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>-2.6653406373704507 -4.104661242158717 0.2907688925864437 0.0 0.0 5.040312055299821</pose>
      <name>maize_02_0070</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>-2.6874142217012067 -3.9871669626277337 0.2907688925864437 0.0 0.0 2.4687556141505933</pose>
      <name>maize_02_0071</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>-2.6924599329030676 -3.8330563892566154 0.2907688925864437 0.0 0.0 5.305486962627388</pose>
      <name>maize_02_0072</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>-2.6861873542766492 -3.6531569100639913 0.2907688925864437 0.0 0.0 2.9950496303552345</pose>
      <name>maize_02_0073</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>-2.679743831019111 -3.5233403806704384 0.2907688925864437 0.0 0.0 6.007516574035589</pose>
      <name>maize_01_0074</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>-2.689617700214349 -3.364636528907879 0.2907688925864437 0.0 0.0 2.9257003700343884</pose>
      <name>maize_02_0075</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>-2.6752972661849195 -3.223615654932426 0.2907688925864437 0.0 0.0 1.191116994213174</pose>
      <name>maize_01_0076</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>-2.683556489400629 -3.059888217922572 0.2907688925864437 0.0 0.0 3.9917948913972263</pose>
      <name>maize_02_0077</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>-2.687345935823335 -2.902507341033866 0.2907688925864437 0.0 0.0 2.7718282817389452</pose>
      <name>maize_02_0078</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>-2.6847605614680186 -2.7386629788280565 0.2907688925864437 0.0 0.0 2.8944055818955987</pose>
      <name>maize_02_0079</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>-2.674352836467894 -2.529463330890467 0.2907688925864437 0.0 0.0 4.39889965990327</pose>
      <name>maize_02_0080</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>-2.6698677731477214 -2.37008284747981 0.2907688925864437 0.0 0.0 3.149248445996983</pose>
      <name>maize_02_0081</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>-2.6901036762363226 -2.199142835178229 0.2907688925864437 0.0 0.0 4.92171660188916</pose>
      <name>maize_01_0082</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>-2.6959200981373144 -2.0529486864738558 0.2907688925864437 0.0 0.0 0.1309103709829023</pose>
      <name>maize_01_0083</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>-2.6709172764483866 -1.897170908781054 0.2907688925864437 0.0 0.0 4.3384570415771195</pose>
      <name>maize_01_0084</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>-2.678684644772937 -1.7828011709525144 0.2907688925864437 0.0 0.0 2.5326631312460606</pose>
      <name>maize_02_0085</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>-2.6775106364192656 -1.6364464856730647 0.2907688925864437 0.0 0.0 5.988221198760665</pose>
      <name>maize_01_0086</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>-2.6736354044583006 -1.439782784743294 0.2907688925864437 0.0 0.0 0.820055471873943</pose>
      <name>maize_02_0087</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>-2.6843479710084934 -1.26347546748632 0.2907688925864437 0.0 0.0 2.4834847711482637</pose>
      <name>maize_02_0088</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>-2.681519756906996 -1.13906875095034 0.2907688925864437 0.0 0.0 4.113164646531646</pose>
      <name>maize_02_0089</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>-2.6932458790733587 -0.9618734340955797 0.2907688925864437 0.0 0.0 0.297487103309708</pose>
      <name>maize_02_0090</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>-2.6834283100112906 -0.7873570745254774 0.2907688925864437 0.0 0.0 0.14081043411224642</pose>
      <name>maize_02_0091</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>-2.685497723719615 -0.6340289382295587 0.2907688925864437 0.0 0.0 2.0483535517885882</pose>
      <name>maize_01_0092</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>-2.6875904945440277 -0.5066469419945898 0.2907688925864437 0.0 0.0 2.484303081406399</pose>
      <name>maize_01_0093</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>-2.6834440338565155 -0.3248173978128541 0.2907688925864437 0.0 0.0 1.1340770287686448</pose>
      <name>maize_01_0094</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>-2.6971125418474764 -0.15061551069915424 0.2907688925864437 0.0 0.0 4.962062707213124</pose>
      <name>maize_02_0095</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>-2.664836615856087 0.6557228429399418 0.2928365955487579 0.0 0.0 5.973153071848792</pose>
      <name>maize_01_0096</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>-2.680955499750845 0.8351811749648927 0.2928365955487579 0.0 0.0 3.4456898773095173</pose>
      <name>maize_02_0097</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>-2.6705290982965284 0.9894626607155494 0.2928365955487579 0.0 0.0 6.21019327699511</pose>
      <name>maize_01_0098</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>-2.6921750721718722 1.1817531719250347 0.2928365955487579 0.0 0.0 2.2808474975810813</pose>
      <name>maize_02_0099</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>-2.6925965803147704 1.3223153346411838 0.2928365955487579 0.0 0.0 6.132039960740185</pose>
      <name>maize_01_0100</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>-2.6786858978138577 1.4635396261711122 0.2928365955487579 0.0 0.0 4.223603932023898</pose>
      <name>maize_01_0101</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>-2.6732049073302973 1.6384960112920144 0.2928365955487579 0.0 0.0 4.162129248775341</pose>
      <name>maize_01_0102</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>-2.6857057790347554 1.8018099891712893 0.2928365955487579 0.0 0.0 3.942527518500527</pose>
      <name>maize_01_0103</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>-2.6694793331451008 1.97203967691211 0.2928365955487579 0.0 0.0 1.7270066262810224</pose>
      <name>maize_02_0104</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>-2.69273701186682 2.1111405919698445 0.2928365955487579 0.0 0.0 3.5122812664748086</pose>
      <name>maize_02_0105</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>-2.6717469751294987 2.225850554814036 0.2928365955487579 0.0 0.0 0.8949223266250003</pose>
      <name>maize_02_0106</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>-2.679455516348051 2.4075700886483746 0.2928365955487579 0.0 0.0 1.1158307077568974</pose>
      <name>maize_02_0107</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>-2.6945508300714986 2.544958241028896 0.2928365955487579 0.0 0.0 3.4450501481729527</pose>
      <name>maize_01_0108</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>-2.66969465478028 2.7316684567204437 0.2928365955487579 0.0 0.0 0.9420955606025773</pose>
      <name>maize_01_0109</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>-2.691860498320991 2.8499451516942935 0.2928365955487579 0.0 0.0 0.022872343523549395</pose>
      <name>maize_01_0110</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>-2.6918892774858394 2.988056572230705 0.2928365955487579 0.0 0.0 0.4829087276957363</pose>
      <name>maize_01_0111</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>-2.6907897956788185 3.1845372398820215 0.2928365955487579 0.0 0.0 4.042526420251413</pose>
      <name>maize_01_0112</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>-2.6998261655523343 3.369103900498824 0.2928365955487579 0.0 0.0 4.341203317995775</pose>
      <name>maize_02_0113</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>-2.6920304665308805 3.5096125721236104 0.2928365955487579 0.0 0.0 0.06902845898446912</pose>
      <name>maize_02_0114</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>-2.6729817322974783 3.674149821919733 0.2928365955487579 0.0 0.0 0.3447715230067694</pose>
      <name>maize_02_0115</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>-2.692762757423986 3.8574686532445215 0.2928365955487579 0.0 0.0 3.873017110590971</pose>
      <name>maize_02_0116</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>-2.6838097159093017 4.001628313303461 0.2928365955487579 0.0 0.0 2.66044598238817</pose>
      <name>maize_02_0117</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>-2.676082058066836 4.1424267868235 0.2928365955487579 0.0 0.0 5.325154988087079</pose>
      <name>maize_01_0118</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>-2.6829127542809146 4.337503193233521 0.2928365955487579 0.0 0.0 0.16235613515340697</pose>
      <name>maize_02_0119</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>-2.686999296531224 4.533063596417679 0.2928365955487579 0.0 0.0 5.882049769311074</pose>
      <name>maize_01_0120</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>-2.672556207480571 4.683676110342199 0.2928365955487579 0.0 0.0 3.0461314689927064</pose>
      <name>maize_01_0121</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>-2.674256757472532 4.858033438210412 0.2928365955487579 0.0 0.0 0.985273586224128</pose>
      <name>maize_01_0122</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>-2.684366554515764 5.02724886300211 0.2928365955487579 0.0 0.0 1.042426063360034</pose>
      <name>maize_01_0123</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>-1.9390879112035126 -5.136004134798475 0.28723981134455145 0.0 0.0 5.307820775853577</pose>
      <name>maize_02_0124</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>-1.9410194966360714 -4.968466356874598 0.28723981134455145 0.0 0.0 0.9458771068739309</pose>
      <name>maize_02_0125</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>-1.913928717729262 -4.82825002581289 0.28723981134455145 0.0 0.0 4.235666940225235</pose>
      <name>maize_02_0126</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>-1.942939557973517 -4.6318636826006445 0.28723981134455145 0.0 0.0 1.6996591477167484</pose>
      <name>maize_01_0127</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>-1.9452364882708113 -4.472812228718531 0.28723981134455145 0.0 0.0 5.814355240853099</pose>
      <name>maize_02_0128</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>-1.9260098623199795 -4.294900546565216 0.28723981134455145 0.0 0.0 1.7240389517364805</pose>
      <name>maize_01_0129</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>-1.9392476138649666 -4.11766203718456 0.28723981134455145 0.0 0.0 5.869950345336705</pose>
      <name>maize_01_0130</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>-1.9206716185577188 -3.9717780555186426 0.28723981134455145 0.0 0.0 6.0287713493348285</pose>
      <name>maize_01_0131</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>-1.9212303905592534 -3.8219671020501123 0.28723981134455145 0.0 0.0 4.077133353786977</pose>
      <name>maize_01_0132</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>-1.9302176490124066 -3.656029018756259 0.28723981134455145 0.0 0.0 3.0805326986426045</pose>
      <name>maize_02_0133</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>-1.933041265258463 -3.4813484836885515 0.28723981134455145 0.0 0.0 2.588050340801529</pose>
      <name>maize_01_0134</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>-1.9469265967578755 -3.3271055068039717 0.28723981134455145 0.0 0.0 2.037313972078032</pose>
      <name>maize_02_0135</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>-1.9396286219329102 -3.187564455715193 0.28723981134455145 0.0 0.0 4.221150591290892</pose>
      <name>maize_01_0136</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>-1.9469725899022494 -3.0158724855431776 0.28723981134455145 0.0 0.0 1.9821536566005904</pose>
      <name>maize_02_0137</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>-1.9479640746117854 -2.8638953735806787 0.28723981134455145 0.0 0.0 2.262401137846337</pose>
      <name>maize_01_0138</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>-1.930368688231572 -2.730616988253917 0.28723981134455145 0.0 0.0 4.968609197143389</pose>
      <name>maize_02_0139</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>-1.9163886555191643 -2.586365313338693 0.28723981134455145 0.0 0.0 4.168058345955526</pose>
      <name>maize_01_0140</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>-1.9472253958476822 -2.2376295204765335 0.29805819867314265 0.0 0.0 5.087676526152143</pose>
      <name>maize_02_0141</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>-1.9462489968429928 -2.066893367333111 0.29805819867314265 0.0 0.0 3.2335737073991413</pose>
      <name>maize_02_0142</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>-1.9206022070160866 -1.889432223953658 0.29805819867314265 0.0 0.0 0.5209851139881868</pose>
      <name>maize_01_0143</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>-1.945702991428711 -1.7101037380392077 0.29805819867314265 0.0 0.0 4.57731777740399</pose>
      <name>maize_01_0144</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>-1.9212765740903954 -1.5513617810021567 0.29805819867314265 0.0 0.0 1.6343943131450105</pose>
      <name>maize_02_0145</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>-1.9393521088005479 -1.4021761779898374 0.29805819867314265 0.0 0.0 5.9571637256987735</pose>
      <name>maize_01_0146</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>-1.9517735779479854 -1.2542432258227492 0.29805819867314265 0.0 0.0 0.32630817544605606</pose>
      <name>maize_02_0147</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>-1.9256557742808866 -1.0874722902304672 0.29805819867314265 0.0 0.0 1.50302749945533</pose>
      <name>maize_01_0148</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>-1.9196831977629407 -0.9202556775780293 0.29805819867314265 0.0 0.0 1.0500357150786508</pose>
      <name>maize_01_0149</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>-1.9338671922158064 -0.7376211224682541 0.29805819867314265 0.0 0.0 1.1122744658454946</pose>
      <name>maize_02_0150</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>-1.9481990486109473 -0.5996419162683928 0.29805819867314265 0.0 0.0 4.575373811283382</pose>
      <name>maize_02_0151</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>-1.9159507996954561 -0.4589617485630395 0.29805819867314265 0.0 0.0 4.096623440279471</pose>
      <name>maize_01_0152</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>-1.9202318609937816 0.1761011630791245 0.2933576391843192 0.0 0.0 4.207361134711418</pose>
      <name>maize_01_0153</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>-1.9411267213559318 0.3856133950340883 0.2933576391843192 0.0 0.0 1.6606162794017654</pose>
      <name>maize_02_0154</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>-1.9365463301445986 0.5246139253975075 0.2933576391843192 0.0 0.0 1.3617570013301457</pose>
      <name>maize_02_0155</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>-1.9499064192664146 0.6622944930583392 0.2933576391843192 0.0 0.0 1.210425391109182</pose>
      <name>maize_01_0156</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>-1.9352790792518761 0.8436357292186845 0.2933576391843192 0.0 0.0 3.7592565593793563</pose>
      <name>maize_02_0157</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>-1.9334860788204349 0.9932587760359732 0.2933576391843192 0.0 0.0 4.562714565864099</pose>
      <name>maize_02_0158</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>-1.9433483798079774 1.1707738358665356 0.2933576391843192 0.0 0.0 2.474124555738976</pose>
      <name>maize_01_0159</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>-1.932491116939473 1.3379139617582068 0.2933576391843192 0.0 0.0 0.6309674901766729</pose>
      <name>maize_02_0160</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>-1.9371952472588547 1.4997690757855686 0.2933576391843192 0.0 0.0 3.7928974934191086</pose>
      <name>maize_02_0161</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>-1.9503093054024396 1.6982388434397162 0.2933576391843192 0.0 0.0 4.612830418254849</pose>
      <name>maize_02_0162</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>-1.927214029324751 1.8135337965168121 0.2933576391843192 0.0 0.0 0.256491432826332</pose>
      <name>maize_01_0163</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>-1.935408788641456 2.612682847657087 0.29165811128206565 0.0 0.0 1.3395662578424776</pose>
      <name>maize_02_0164</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>-1.9375489195713165 2.7621466235859264 0.29165811128206565 0.0 0.0 4.620434394321687</pose>
      <name>maize_01_0165</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>-1.9440873647140402 2.9607892700716265 0.29165811128206565 0.0 0.0 3.397458150058163</pose>
      <name>maize_01_0166</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>-1.9439874029770983 3.1179612702910395 0.29165811128206565 0.0 0.0 5.769698116128082</pose>
      <name>maize_01_0167</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>-1.9330074248388676 3.262852895128405 0.29165811128206565 0.0 0.0 3.3361498327097783</pose>
      <name>maize_02_0168</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>-1.9376744796072976 3.420293011647119 0.29165811128206565 0.0 0.0 1.5063370472116147</pose>
      <name>maize_01_0169</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>-1.9355231759542886 3.575221362951985 0.29165811128206565 0.0 0.0 0.1609949088246878</pose>
      <name>maize_01_0170</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>-1.9407400032110032 3.7825477465070696 0.29165811128206565 0.0 0.0 1.399545348496909</pose>
      <name>maize_02_0171</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>-1.9525410903541693 3.9356169483099146 0.29165811128206565 0.0 0.0 1.369305156822726</pose>
      <name>maize_01_0172</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>-1.9322777565193945 4.097111205051303 0.29165811128206565 0.0 0.0 2.1265754589071366</pose>
      <name>maize_02_0173</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>-1.9169752325536977 4.287852257545027 0.29165811128206565 0.0 0.0 4.547985674749507</pose>
      <name>maize_01_0174</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>-1.9330017745774342 4.458608472996253 0.29165811128206565 0.0 0.0 2.55085717694152</pose>
      <name>maize_02_0175</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>-1.9322241172585322 4.6243353982775774 0.29165811128206565 0.0 0.0 4.6846823973301435</pose>
      <name>maize_01_0176</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>-1.944542358961069 4.7944877075474635 0.29165811128206565 0.0 0.0 3.0187997882368287</pose>
      <name>maize_02_0177</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>-1.952584800993338 4.932459106275847 0.29165811128206565 0.0 0.0 1.0781779819419857</pose>
      <name>maize_02_0178</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>-1.1741202229457235 -5.082792930878854 0.2903592975735342 0.0 0.0 2.5051897817366693</pose>
      <name>maize_01_0179</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>-1.1931503895184634 -4.936254211718262 0.2903592975735342 0.0 0.0 2.5765852012726636</pose>
      <name>maize_01_0180</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>-1.1747218888102777 -4.812418997389959 0.2903592975735342 0.0 0.0 3.3313467309206723</pose>
      <name>maize_02_0181</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>-1.1991316312067992 -4.6471167225950705 0.2903592975735342 0.0 0.0 0.8054795468284451</pose>
      <name>maize_02_0182</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>-1.1987447813047765 -4.522877761435936 0.2903592975735342 0.0 0.0 3.5853734524413814</pose>
      <name>maize_01_0183</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>-1.199707293232231 -4.374549027356943 0.2903592975735342 0.0 0.0 2.6283761755533246</pose>
      <name>maize_02_0184</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>-1.1791446185286993 -4.202197282925405 0.2903592975735342 0.0 0.0 1.130016583679095</pose>
      <name>maize_01_0185</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>-1.182669136106088 -4.046307555285337 0.2903592975735342 0.0 0.0 4.410622126330697</pose>
      <name>maize_01_0186</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>-1.1950577654254206 -3.884974568162968 0.2903592975735342 0.0 0.0 3.475327683846249</pose>
      <name>maize_01_0187</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>-1.2029301457804613 -3.733170398027257 0.2903592975735342 0.0 0.0 1.6234287420142717</pose>
      <name>maize_02_0188</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>-1.1837039781023009 -3.5272854326721212 0.2903592975735342 0.0 0.0 0.5688374906391139</pose>
      <name>maize_02_0189</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>-1.1808554137666074 -3.374677016414051 0.2903592975735342 0.0 0.0 1.5179146962859766</pose>
      <name>maize_02_0190</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>-1.189233825194885 -2.2500729427295254 0.2992226334220228 0.0 0.0 0.4364667210518579</pose>
      <name>maize_02_0191</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>-1.1984389303815526 -2.0714915220071393 0.2992226334220228 0.0 0.0 2.395496655249187</pose>
      <name>maize_01_0192</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>-1.1792068930215605 -1.956959992382937 0.2992226334220228 0.0 0.0 5.313686991155943</pose>
      <name>maize_01_0193</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>-1.179474452298117 -1.8117156184339138 0.2992226334220228 0.0 0.0 0.9173877971680676</pose>
      <name>maize_01_0194</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>-1.1822611282614464 -1.6357195125704465 0.2992226334220228 0.0 0.0 0.9438233846206625</pose>
      <name>maize_01_0195</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>-1.1783853308405718 -1.487040183267161 0.2992226334220228 0.0 0.0 1.0486694180708311</pose>
      <name>maize_01_0196</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>-1.1822319102954921 -1.3241949374030817 0.2992226334220228 0.0 0.0 0.6878298888506588</pose>
      <name>maize_02_0197</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>-1.1917731710818638 -1.142903215212244 0.2992226334220228 0.0 0.0 0.782317930761463</pose>
      <name>maize_01_0198</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>-1.1873195098996048 -0.9472506198966979 0.2992226334220228 0.0 0.0 0.16935544827441645</pose>
      <name>maize_01_0199</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>-1.1782421796763853 -0.8340796561700126 0.2992226334220228 0.0 0.0 6.208045357758416</pose>
      <name>maize_01_0200</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>-1.1709521100250782 -0.6569278977825945 0.2992226334220228 0.0 0.0 0.4258153883934576</pose>
      <name>maize_01_0201</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>-1.1699486502185428 -0.04690040598103273 0.2940487636827868 0.0 0.0 0.40260061546935494</pose>
      <name>maize_02_0202</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>-1.188562068173578 0.08684550564820714 0.2940487636827868 0.0 0.0 4.3501779729536025</pose>
      <name>maize_02_0203</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>-1.1905199910293276 0.24760002921157742 0.2940487636827868 0.0 0.0 0.37519356850266145</pose>
      <name>maize_01_0204</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>-1.1754120264972938 0.41952846871398997 0.2940487636827868 0.0 0.0 0.15306152459795871</pose>
      <name>maize_01_0205</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>-1.1977974286993622 0.8327310225978302 0.2901008141825368 0.0 0.0 1.4150886193968226</pose>
      <name>maize_01_0206</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>-1.1847820212421563 0.9992287010231351 0.2901008141825368 0.0 0.0 4.5777214113366425</pose>
      <name>maize_01_0207</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>-1.172460254306535 1.1928410588138192 0.2901008141825368 0.0 0.0 5.5106777024404145</pose>
      <name>maize_02_0208</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>-1.1779131249859325 1.3641997679919005 0.2901008141825368 0.0 0.0 0.8367193968376264</pose>
      <name>maize_01_0209</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>-1.1889109647959684 1.5211090592648162 0.2901008141825368 0.0 0.0 1.5971741272332791</pose>
      <name>maize_01_0210</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>-1.19440097449958 1.6871955558224583 0.2901008141825368 0.0 0.0 2.543546072507386</pose>
      <name>maize_02_0211</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>-1.1878889717309584 1.8319249444549603 0.2901008141825368 0.0 0.0 1.0201555529333954</pose>
      <name>maize_01_0212</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>-1.1944165818036767 1.9859885084712898 0.2901008141825368 0.0 0.0 0.7509997548415828</pose>
      <name>maize_01_0213</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>-1.192124203058345 2.15384354260177 0.2901008141825368 0.0 0.0 3.4350649429112967</pose>
      <name>maize_02_0214</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>-1.194980547056566 2.3539504747889524 0.2901008141825368 0.0 0.0 1.984413722395875</pose>
      <name>maize_01_0215</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>-1.182548713740236 2.541245340788712 0.2901008141825368 0.0 0.0 1.5551136865880777</pose>
      <name>maize_01_0216</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>-1.177739581306008 2.7008160251048343 0.2901008141825368 0.0 0.0 4.277079998725106</pose>
      <name>maize_01_0217</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>-1.1872998541259911 2.862659423323274 0.2901008141825368 0.0 0.0 3.133599789926177</pose>
      <name>maize_01_0218</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>-1.1974948663611187 3.015401046095964 0.2901008141825368 0.0 0.0 0.9553497369207188</pose>
      <name>maize_02_0219</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>-1.1983583419416526 3.2078408122573157 0.2901008141825368 0.0 0.0 5.978650920882396</pose>
      <name>maize_02_0220</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>-1.2027530130166735 3.3367718307369154 0.2901008141825368 0.0 0.0 1.3634935626489695</pose>
      <name>maize_02_0221</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>-1.1859569633985876 3.4835053723498453 0.2901008141825368 0.0 0.0 2.9443003433621526</pose>
      <name>maize_02_0222</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>-1.1680327296571358 3.64044600368645 0.2901008141825368 0.0 0.0 4.341456852025142</pose>
      <name>maize_01_0223</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>-1.1937066847204796 3.7985624982408615 0.2901008141825368 0.0 0.0 5.893780615768862</pose>
      <name>maize_02_0224</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>-1.1978389991006693 3.9666366537660034 0.2901008141825368 0.0 0.0 1.5055929175416134</pose>
      <name>maize_01_0225</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>-1.182651193740642 4.111012578915026 0.2901008141825368 0.0 0.0 1.4106173182470123</pose>
      <name>maize_01_0226</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>-1.1793622990639627 4.260172970471799 0.2901008141825368 0.0 0.0 4.400613010261971</pose>
      <name>maize_01_0227</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>-1.1700311356528124 4.407781865379051 0.2901008141825368 0.0 0.0 2.413190407182906</pose>
      <name>maize_02_0228</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>-1.1746941385670402 4.585356818726253 0.2901008141825368 0.0 0.0 2.4469560725596016</pose>
      <name>maize_02_0229</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>-1.2004490355622992 4.74766379236227 0.2901008141825368 0.0 0.0 4.385564524009326</pose>
      <name>maize_02_0230</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>-1.1772501595983367 4.893494936232545 0.2901008141825368 0.0 0.0 3.720424553903173</pose>
      <name>maize_01_0231</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>-0.43363664468068874 -5.078475455609083 0.2934386638625485 0.0 0.0 4.997066134215646</pose>
      <name>maize_01_0232</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>-0.4314884172890072 -4.6170422082597975 0.29493320788289934 0.0 0.0 3.9630683559101625</pose>
      <name>maize_01_0233</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>-0.4481802881698136 -4.452176926183648 0.29493320788289934 0.0 0.0 1.961154379189926</pose>
      <name>maize_01_0234</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>-0.4384687944221626 -4.297329549305216 0.29493320788289934 0.0 0.0 4.687499133438443</pose>
      <name>maize_02_0235</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>-0.42065771655406525 -4.140153018486144 0.29493320788289934 0.0 0.0 3.121387550913212</pose>
      <name>maize_02_0236</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>-0.42962515167928306 -3.9995627173113593 0.29493320788289934 0.0 0.0 6.253921190737469</pose>
      <name>maize_01_0237</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>-0.4407837716041847 -3.842222427552966 0.29493320788289934 0.0 0.0 3.5783102251986882</pose>
      <name>maize_02_0238</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>-0.4284374725652236 -3.673948383391386 0.29493320788289934 0.0 0.0 4.233654705410236</pose>
      <name>maize_02_0239</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>-0.44299628766547583 -3.492061923785766 0.29493320788289934 0.0 0.0 3.462833807693597</pose>
      <name>maize_02_0240</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>-0.4451842557803589 -3.3788500484997166 0.29493320788289934 0.0 0.0 2.015026118758488</pose>
      <name>maize_02_0241</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>-0.4305101604720396 -3.1997576619367356 0.29493320788289934 0.0 0.0 5.529352670901123</pose>
      <name>maize_01_0242</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>-0.4496443434292692 -3.05858732035916 0.29493320788289934 0.0 0.0 2.7067928321134103</pose>
      <name>maize_02_0243</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>-0.4293105024726267 -2.91390275826964 0.29493320788289934 0.0 0.0 6.109988608346328</pose>
      <name>maize_01_0244</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>-0.43433019479159407 -2.768754092579854 0.29493320788289934 0.0 0.0 0.9064895794273405</pose>
      <name>maize_01_0245</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>-0.43832910285520077 -2.5702889337891324 0.29493320788289934 0.0 0.0 0.08395329143205493</pose>
      <name>maize_01_0246</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>-0.4310624928607787 -2.4243622125044517 0.29493320788289934 0.0 0.0 2.788293718599852</pose>
      <name>maize_02_0247</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>-0.4364605113989839 -2.2617600979941614 0.29493320788289934 0.0 0.0 1.5452152970301194</pose>
      <name>maize_01_0248</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>-0.45244054342612916 -2.129891302124515 0.29493320788289934 0.0 0.0 0.7333823306057744</pose>
      <name>maize_02_0249</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>-0.43898718923458846 -1.9645959530075205 0.29493320788289934 0.0 0.0 0.9470274755482151</pose>
      <name>maize_01_0250</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>-0.4267279107477093 -1.7962876271663264 0.29493320788289934 0.0 0.0 3.196183337176009</pose>
      <name>maize_01_0251</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>-0.43521596622720704 -1.6122425815289305 0.29493320788289934 0.0 0.0 1.8981893190320724</pose>
      <name>maize_01_0252</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>-0.4492357344947635 -1.486653432855329 0.29493320788289934 0.0 0.0 6.121931847298333</pose>
      <name>maize_02_0253</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>-0.42543236736073364 -1.322696167019028 0.29493320788289934 0.0 0.0 3.5354558976898747</pose>
      <name>maize_02_0254</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>-0.4518983060106434 -1.193039459034889 0.29493320788289934 0.0 0.0 3.0277550377586784</pose>
      <name>maize_01_0255</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>-0.44500377380620293 -0.9954520837153682 0.29493320788289934 0.0 0.0 2.3275253195328105</pose>
      <name>maize_01_0256</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>-0.43450164435499516 -0.8491395375964244 0.29493320788289934 0.0 0.0 5.257664261876355</pose>
      <name>maize_01_0257</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>-0.42366764376399635 -0.7186585255122004 0.29493320788289934 0.0 0.0 2.07762010588029</pose>
      <name>maize_02_0258</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>-0.44718711786123544 -0.5411455362577735 0.29493320788289934 0.0 0.0 0.40577576507973373</pose>
      <name>maize_01_0259</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>-0.44744955964371025 -0.38959708632253864 0.29493320788289934 0.0 0.0 0.3358856992258773</pose>
      <name>maize_01_0260</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>-0.42357731303842927 -0.23126276172791194 0.29493320788289934 0.0 0.0 4.597810140840779</pose>
      <name>maize_01_0261</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>-0.4438415960005657 1.0117140588821494 0.29005462546632915 0.0 0.0 5.24394946792421</pose>
      <name>maize_01_0262</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>-0.4291883906138745 1.152777072231677 0.29005462546632915 0.0 0.0 2.1188710732377425</pose>
      <name>maize_01_0263</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>-0.432983943699083 1.3253516251387598 0.29005462546632915 0.0 0.0 1.780012040250816</pose>
      <name>maize_01_0264</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>-0.43489163003317977 1.4663581602274647 0.29005462546632915 0.0 0.0 5.586291507498421</pose>
      <name>maize_01_0265</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>-0.43519232347030545 1.6463790923003776 0.29005462546632915 0.0 0.0 2.186567746777054</pose>
      <name>maize_02_0266</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>-0.4353423686137008 1.7740616516338 0.29005462546632915 0.0 0.0 1.5705273284510988</pose>
      <name>maize_02_0267</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>-0.4487395640079046 1.9433661898730286 0.29005462546632915 0.0 0.0 5.925574618825958</pose>
      <name>maize_02_0268</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>-0.4474339142520325 2.0916131815810193 0.29005462546632915 0.0 0.0 4.577045330823885</pose>
      <name>maize_01_0269</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>-0.433567867758188 2.2276460162102882 0.29005462546632915 0.0 0.0 2.5933138891650667</pose>
      <name>maize_02_0270</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>-0.4416489200800502 2.4463472378947575 0.29005462546632915 0.0 0.0 5.401895942305134</pose>
      <name>maize_02_0271</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>-0.43160344533995687 2.599900989703852 0.29005462546632915 0.0 0.0 3.2983902590313585</pose>
      <name>maize_02_0272</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>-0.4535141521000292 2.741664918263533 0.29005462546632915 0.0 0.0 3.605154076467036</pose>
      <name>maize_02_0273</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>-0.4532896946697407 2.8710751885088905 0.29005462546632915 0.0 0.0 5.597780142891883</pose>
      <name>maize_01_0274</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>-0.42761079892537435 3.069557709146772 0.29005462546632915 0.0 0.0 3.5351648918419767</pose>
      <name>maize_01_0275</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>-0.44915459726114504 3.208406062752556 0.29005462546632915 0.0 0.0 4.823531363526403</pose>
      <name>maize_02_0276</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>-0.44205691514214207 3.397633512516818 0.29005462546632915 0.0 0.0 2.12828856822771</pose>
      <name>maize_01_0277</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>-0.4290482168880385 3.5469385470307264 0.29005462546632915 0.0 0.0 3.8859279101186766</pose>
      <name>maize_02_0278</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>-0.4243471415544633 4.709670052442298 0.2946326584383497 0.0 0.0 5.064973370937764</pose>
      <name>maize_02_0279</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>-0.45003792692028544 4.859859076045269 0.2946326584383497 0.0 0.0 1.2501570290952777</pose>
      <name>maize_02_0280</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>-0.439268670471324 5.038701289418479 0.2946326584383497 0.0 0.0 3.79836215842102</pose>
      <name>maize_02_0281</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>0.32556192602770606 -5.104735923380147 0.2928726102947117 0.0 0.0 4.997170492110676</pose>
      <name>maize_02_0282</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>0.3150543294566539 -4.95471751168405 0.2928726102947117 0.0 0.0 5.340751955241068</pose>
      <name>maize_01_0283</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>0.308331755851615 -4.76514349248586 0.2928726102947117 0.0 0.0 0.5266107259301388</pose>
      <name>maize_01_0284</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>0.31170540741505226 -4.586290484245454 0.2928726102947117 0.0 0.0 2.1230388779013283</pose>
      <name>maize_02_0285</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>0.2993275714505512 -4.4452630461361835 0.2928726102947117 0.0 0.0 1.2103005010909507</pose>
      <name>maize_02_0286</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>0.3058163689830895 -4.275969912348431 0.2928726102947117 0.0 0.0 4.503496349552127</pose>
      <name>maize_01_0287</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>0.3161111262152976 -4.135868888443419 0.2928726102947117 0.0 0.0 2.3315880134059594</pose>
      <name>maize_01_0288</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>0.3241444062896437 -3.947637165977187 0.2928726102947117 0.0 0.0 2.9486837030537925</pose>
      <name>maize_01_0289</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>0.31082252359637286 -3.8063255452451425 0.2928726102947117 0.0 0.0 3.5319085225629423</pose>
      <name>maize_01_0290</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>0.33099511933522185 -3.6336121903688845 0.2928726102947117 0.0 0.0 0.6127784265137791</pose>
      <name>maize_02_0291</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>0.3295183172696179 -3.4958936061616486 0.2928726102947117 0.0 0.0 2.889930345567569</pose>
      <name>maize_01_0292</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>0.30275736270402476 -3.322249870678565 0.2928726102947117 0.0 0.0 1.430697342500538</pose>
      <name>maize_01_0293</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>0.3253577733646047 -3.193258460628443 0.2928726102947117 0.0 0.0 0.8987434534866431</pose>
      <name>maize_02_0294</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>0.3299588313961568 -3.0263288647843964 0.2928726102947117 0.0 0.0 3.4653096998030737</pose>
      <name>maize_01_0295</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>0.3304591936060808 -2.8385975410976965 0.2928726102947117 0.0 0.0 1.1714260658929543</pose>
      <name>maize_01_0296</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>0.3069125922621385 -2.7077892981845837 0.2928726102947117 0.0 0.0 5.097726029763674</pose>
      <name>maize_01_0297</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>0.3346580941122288 -2.58052500404363 0.2928726102947117 0.0 0.0 5.834619905982987</pose>
      <name>maize_02_0298</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>0.333466106069793 -2.3921532596610966 0.2928726102947117 0.0 0.0 2.4423614400338947</pose>
      <name>maize_01_0299</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>0.3182384564182601 -2.2324459295966186 0.2928726102947117 0.0 0.0 5.478615967105552</pose>
      <name>maize_02_0300</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>0.3140559758661339 -2.0274728109315032 0.2928726102947117 0.0 0.0 2.5052281806033476</pose>
      <name>maize_01_0301</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>0.31382881243465643 -1.8775855016331668 0.2928726102947117 0.0 0.0 4.058092188596861</pose>
      <name>maize_02_0302</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>0.33248201655856047 -1.6958758685024886 0.2928726102947117 0.0 0.0 5.279333921135141</pose>
      <name>maize_02_0303</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>0.3189009516514578 -1.571865901462996 0.2928726102947117 0.0 0.0 5.751069560695198</pose>
      <name>maize_01_0304</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>0.29639045531828057 -1.4037076654536143 0.2928726102947117 0.0 0.0 4.7812727122542515</pose>
      <name>maize_02_0305</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>0.327606931808031 -1.2429920491985187 0.2928726102947117 0.0 0.0 3.2801338091372974</pose>
      <name>maize_02_0306</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>0.3121762287354781 -1.0845289307340638 0.2928726102947117 0.0 0.0 3.523262929656006</pose>
      <name>maize_02_0307</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>0.3180082212982316 -0.9018026379701949 0.2928726102947117 0.0 0.0 4.011246154100022</pose>
      <name>maize_02_0308</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>0.3166609872233672 -0.7879671519517535 0.2928726102947117 0.0 0.0 5.928398421428213</pose>
      <name>maize_01_0309</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>0.32974423872935166 -0.6110516796101946 0.2928726102947117 0.0 0.0 2.250564024722139</pose>
      <name>maize_01_0310</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>0.32355003356342094 -0.46558271984371125 0.2928726102947117 0.0 0.0 0.4374411796815129</pose>
      <name>maize_02_0311</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>0.3305101691436536 -0.3022086584831145 0.2928726102947117 0.0 0.0 3.74222302838097</pose>
      <name>maize_02_0312</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>0.2974321826584858 -0.14415428581634693 0.2928726102947117 0.0 0.0 3.0980663594969386</pose>
      <name>maize_02_0313</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>0.3159248577290601 -0.010334025934239932 0.2928726102947117 0.0 0.0 1.1976631380986793</pose>
      <name>maize_01_0314</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>0.33108548852308495 0.1451944021279825 0.2928726102947117 0.0 0.0 0.324720861490631</pose>
      <name>maize_01_0315</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>0.3215617513059241 0.28890192277398263 0.2928726102947117 0.0 0.0 1.5317786379058889</pose>
      <name>maize_02_0316</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>0.335124945279063 0.4513002898126768 0.2928726102947117 0.0 0.0 5.203741401248203</pose>
      <name>maize_01_0317</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>0.3015049518055499 0.6201759531402589 0.2928726102947117 0.0 0.0 3.719578827370138</pose>
      <name>maize_01_0318</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>0.33561089882604467 0.8012657367453873 0.2928726102947117 0.0 0.0 4.091884967982439</pose>
      <name>maize_01_0319</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>0.29924572521663073 0.9794499936493359 0.2928726102947117 0.0 0.0 6.040809826050248</pose>
      <name>maize_02_0320</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>0.3302646680946122 1.1822380547106164 0.2928726102947117 0.0 0.0 2.1169383469163865</pose>
      <name>maize_02_0321</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>0.31328226161309747 1.350555629012657 0.2928726102947117 0.0 0.0 3.388401723399162</pose>
      <name>maize_02_0322</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>0.33239536714021556 1.4993527003256224 0.2928726102947117 0.0 0.0 1.9357190893581624</pose>
      <name>maize_01_0323</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>0.3255171137618236 1.6754292133517463 0.2928726102947117 0.0 0.0 2.203955643600239</pose>
      <name>maize_02_0324</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>0.3140447216214768 1.8716162832019858 0.2928726102947117 0.0 0.0 5.46611297207772</pose>
      <name>maize_01_0325</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>0.3016219495268122 2.011938439890687 0.2928726102947117 0.0 0.0 4.685001731281291</pose>
      <name>maize_01_0326</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>0.3291459972701163 2.1827089002997573 0.2928726102947117 0.0 0.0 2.613896171082865</pose>
      <name>maize_01_0327</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>0.3228935617908131 2.283137831063325 0.2928726102947117 0.0 0.0 4.149956852638339</pose>
      <name>maize_01_0328</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>0.3102926148567886 2.4844847916183586 0.2928726102947117 0.0 0.0 5.282496275488914</pose>
      <name>maize_01_0329</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>0.32229503426025197 2.626540353130748 0.2928726102947117 0.0 0.0 5.442157435387965</pose>
      <name>maize_01_0330</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>0.3216280133705949 2.799887911100977 0.2928726102947117 0.0 0.0 1.4735770660369742</pose>
      <name>maize_02_0331</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>0.30693972887524934 2.9777477545578774 0.2928726102947117 0.0 0.0 1.0276042452129825</pose>
      <name>maize_01_0332</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>0.32703421422906986 3.1264481206118644 0.2928726102947117 0.0 0.0 4.0491256427941416</pose>
      <name>maize_01_0333</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>0.3343188722477106 3.310808776397212 0.2928726102947117 0.0 0.0 4.331628568790747</pose>
      <name>maize_02_0334</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>0.29666858248819716 3.4276570385513496 0.2928726102947117 0.0 0.0 2.995504991330039</pose>
      <name>maize_01_0335</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>0.3089020463741954 3.640618113351949 0.2928726102947117 0.0 0.0 0.2683734350496738</pose>
      <name>maize_01_0336</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>0.3214505323631802 3.7805836378764974 0.2928726102947117 0.0 0.0 1.6016198751158055</pose>
      <name>maize_01_0337</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>0.31886720285756676 3.965579619371959 0.2928726102947117 0.0 0.0 0.6895106021218087</pose>
      <name>maize_01_0338</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>0.3097183587413386 4.107331636782857 0.2928726102947117 0.0 0.0 1.5621849727068846</pose>
      <name>maize_02_0339</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>0.3332546690735967 4.231053327146609 0.2928726102947117 0.0 0.0 4.193311620968604</pose>
      <name>maize_01_0340</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>0.33320660303413474 4.412746577618143 0.2928726102947117 0.0 0.0 5.755738257262761</pose>
      <name>maize_01_0341</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>0.33210295615577623 4.578068556047167 0.2928726102947117 0.0 0.0 2.133496561774821</pose>
      <name>maize_02_0342</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>0.3117315774350744 4.714374994774897 0.2928726102947117 0.0 0.0 3.2713817321373475</pose>
      <name>maize_01_0343</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>0.3191158805502514 4.889347494971635 0.2928726102947117 0.0 0.0 2.9337595930316285</pose>
      <name>maize_02_0344</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>1.0528739558835554 -5.0859383808827126 0.2919503043211369 0.0 0.0 6.1054342397024515</pose>
      <name>maize_01_0345</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>1.0580464122690536 -4.928145278004712 0.2919503043211369 0.0 0.0 1.5653756881414078</pose>
      <name>maize_02_0346</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>1.046846163528513 -4.712630419648847 0.2919503043211369 0.0 0.0 1.1135790528711131</pose>
      <name>maize_01_0347</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>1.0486864805433171 -4.527766080222515 0.2919503043211369 0.0 0.0 3.8284221383860233</pose>
      <name>maize_02_0348</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>1.0484891160655643 -4.383928201628717 0.2919503043211369 0.0 0.0 1.5925849086131587</pose>
      <name>maize_01_0349</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>1.068267481186087 -4.228529961653501 0.2919503043211369 0.0 0.0 0.08303108640542939</pose>
      <name>maize_01_0350</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>1.0653549979155121 -4.079399807207284 0.2919503043211369 0.0 0.0 5.80384540228637</pose>
      <name>maize_01_0351</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>1.0744632174634958 -3.8964067098769326 0.2919503043211369 0.0 0.0 1.708660612971579</pose>
      <name>maize_01_0352</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>1.0780498271271517 -3.7368199776121713 0.2919503043211369 0.0 0.0 3.5616760248985546</pose>
      <name>maize_02_0353</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>1.0729326524803762 -3.5801218757910216 0.2919503043211369 0.0 0.0 4.1482117479207465</pose>
      <name>maize_02_0354</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>1.0676889700856558 -3.40049687194929 0.2919503043211369 0.0 0.0 4.603024437688789</pose>
      <name>maize_02_0355</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>1.0700234199880674 -3.253091920586529 0.2919503043211369 0.0 0.0 0.8900767538349641</pose>
      <name>maize_01_0356</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>1.0690534198441597 -3.086820266679394 0.2919503043211369 0.0 0.0 3.4906960673399556</pose>
      <name>maize_01_0357</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>1.0590250316993526 -2.907873022247977 0.2919503043211369 0.0 0.0 0.33091194437045196</pose>
      <name>maize_01_0358</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>1.0848820282126113 -2.7800899249762554 0.2919503043211369 0.0 0.0 0.4462667667234649</pose>
      <name>maize_01_0359</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>1.0576082057231995 -2.6129704478135714 0.2919503043211369 0.0 0.0 3.5254409114714385</pose>
      <name>maize_01_0360</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>1.0856739020250608 -2.4815121037974897 0.2919503043211369 0.0 0.0 1.6319309536822104</pose>
      <name>maize_02_0361</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>1.048225000638542 -2.294277249077359 0.2919503043211369 0.0 0.0 1.6585288598954495</pose>
      <name>maize_01_0362</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>1.0683206315498595 -2.109284594123223 0.2919503043211369 0.0 0.0 4.287006487787303</pose>
      <name>maize_02_0363</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>1.0663743111976238 -1.9261736996463936 0.2919503043211369 0.0 0.0 1.68214402632025</pose>
      <name>maize_01_0364</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>1.0667095134394495 -1.7823566350766522 0.2919503043211369 0.0 0.0 2.2831355304751355</pose>
      <name>maize_01_0365</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>1.0547032042099618 -1.6132650488364262 0.2919503043211369 0.0 0.0 5.575589273728159</pose>
      <name>maize_01_0366</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>1.0522312415707296 -1.429152823149312 0.2919503043211369 0.0 0.0 4.349998630140278</pose>
      <name>maize_02_0367</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>1.0648115376458138 -1.2543707288082762 0.2919503043211369 0.0 0.0 5.756929889646097</pose>
      <name>maize_02_0368</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>1.060456760611034 -1.1080435863700169 0.2919503043211369 0.0 0.0 6.214123198181906</pose>
      <name>maize_02_0369</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>1.063661139198362 -0.9188120488195803 0.2919503043211369 0.0 0.0 4.544613487511747</pose>
      <name>maize_01_0370</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>1.0632526800481337 -0.4576303416359693 0.29323568028999825 0.0 0.0 0.1505362145883905</pose>
      <name>maize_01_0371</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>1.0735930442409503 -0.33396261610916245 0.29323568028999825 0.0 0.0 0.0317923379282769</pose>
      <name>maize_02_0372</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>1.0826238359363822 -0.1733299098054033 0.29323568028999825 0.0 0.0 4.888649392728227</pose>
      <name>maize_01_0373</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>1.0663837989622893 -0.018437782837223082 0.29323568028999825 0.0 0.0 2.9122702772386155</pose>
      <name>maize_02_0374</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>1.0541431491509918 0.13258219287406448 0.29323568028999825 0.0 0.0 5.942026440788445</pose>
      <name>maize_01_0375</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>1.051576628154467 0.28486427664757485 0.29323568028999825 0.0 0.0 2.457277866650178</pose>
      <name>maize_01_0376</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>1.073668647845313 0.4417381023531819 0.29323568028999825 0.0 0.0 3.974315583152913</pose>
      <name>maize_02_0377</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>1.0783689534215455 0.6173030835952362 0.29323568028999825 0.0 0.0 5.504266472378785</pose>
      <name>maize_02_0378</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>1.077677039280581 0.786983226069573 0.29323568028999825 0.0 0.0 0.13250029290182905</pose>
      <name>maize_01_0379</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>1.0644306888073523 0.9970794692783542 0.29323568028999825 0.0 0.0 4.408578118246324</pose>
      <name>maize_01_0380</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>1.05669884212175 1.1649852444204614 0.29323568028999825 0.0 0.0 0.769800710754553</pose>
      <name>maize_02_0381</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>1.0585169325708685 1.3390287297823216 0.29323568028999825 0.0 0.0 0.8342974597832188</pose>
      <name>maize_01_0382</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>1.0707481571577961 1.476325273982039 0.29323568028999825 0.0 0.0 1.887166933974127</pose>
      <name>maize_02_0383</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>1.058741071883551 1.6502103013125922 0.29323568028999825 0.0 0.0 2.1372896930752274</pose>
      <name>maize_02_0384</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>1.06080065247831 1.8044659879904499 0.29323568028999825 0.0 0.0 0.23631138148217692</pose>
      <name>maize_01_0385</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>1.0483063128814303 1.9588979582348909 0.29323568028999825 0.0 0.0 0.2247458255490909</pose>
      <name>maize_01_0386</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>1.0674770532524223 2.1116831034794457 0.29323568028999825 0.0 0.0 1.114406825838587</pose>
      <name>maize_02_0387</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>1.061220572675369 2.7562157419939766 0.2890915899789252 0.0 0.0 5.55043552882403</pose>
      <name>maize_02_0388</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>1.0633822858087054 2.9434972734087106 0.2890915899789252 0.0 0.0 4.740650339854593</pose>
      <name>maize_01_0389</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>1.0725061840951664 3.1348889632363237 0.2890915899789252 0.0 0.0 1.9709059647289087</pose>
      <name>maize_02_0390</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>1.058260052617424 3.275106988868685 0.2890915899789252 0.0 0.0 3.1902708954948324</pose>
      <name>maize_01_0391</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>1.07099042997379 3.4268010759965373 0.2890915899789252 0.0 0.0 4.901837963504838</pose>
      <name>maize_01_0392</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>1.077362728178639 3.5797824414262527 0.2890915899789252 0.0 0.0 5.126551791581402</pose>
      <name>maize_01_0393</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>1.0544376465582443 3.7118068870905914 0.2890915899789252 0.0 0.0 0.1086269044626479</pose>
      <name>maize_01_0394</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>1.0504314470173428 3.8895716260060746 0.2890915899789252 0.0 0.0 4.9410695228505</pose>
      <name>maize_02_0395</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>1.0682334316469229 4.076781815292134 0.2890915899789252 0.0 0.0 1.9214591701387982</pose>
      <name>maize_02_0396</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>1.0491272591516232 4.2399648981357485 0.2890915899789252 0.0 0.0 1.1690949064877547</pose>
      <name>maize_01_0397</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>1.0858712419368333 4.407152086235377 0.2890915899789252 0.0 0.0 2.317917595260347</pose>
      <name>maize_02_0398</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>1.0829347774443665 4.538109377163367 0.2890915899789252 0.0 0.0 3.69083647354739</pose>
      <name>maize_01_0399</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>1.0747523931648937 4.722058332764036 0.2890915899789252 0.0 0.0 5.809129826859533</pose>
      <name>maize_02_0400</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>1.0754538245477159 4.880068039211737 0.2890915899789252 0.0 0.0 6.052179171102539</pose>
      <name>maize_01_0401</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>1.804129829835487 -5.1308236997256245 0.2918372252026984 0.0 0.0 5.707531364078959</pose>
      <name>maize_01_0402</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>1.8279618559131707 -4.801768434920175 0.2918372252026984 0.0 0.0 3.9280790554748277</pose>
      <name>maize_01_0403</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>1.8161240507234098 -4.626897178675301 0.2918372252026984 0.0 0.0 4.2024967202083765</pose>
      <name>maize_01_0404</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>1.815687995743879 -4.486693382445907 0.2918372252026984 0.0 0.0 2.644311840999304</pose>
      <name>maize_02_0405</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>1.8335628571784204 -4.3199936301616475 0.2918372252026984 0.0 0.0 3.8207015993340705</pose>
      <name>maize_01_0406</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>1.8028394051364316 -4.128327678884117 0.2918372252026984 0.0 0.0 5.218907785134299</pose>
      <name>maize_01_0407</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>1.8120357440098527 -3.954023302708374 0.2918372252026984 0.0 0.0 2.7015649564494124</pose>
      <name>maize_01_0408</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>1.8248394147954317 -3.777772576399994 0.2918372252026984 0.0 0.0 2.597876254563599</pose>
      <name>maize_01_0409</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>1.8034598670110098 -3.632159258475602 0.2918372252026984 0.0 0.0 6.258715469768118</pose>
      <name>maize_02_0410</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>1.8003495773572986 -3.4431499355705144 0.2918372252026984 0.0 0.0 5.841682565239426</pose>
      <name>maize_01_0411</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>1.8267797774760837 -3.326356278468878 0.2918372252026984 0.0 0.0 2.8272216955308562</pose>
      <name>maize_02_0412</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>1.819311469932182 -3.190519316415098 0.2918372252026984 0.0 0.0 0.4623813086807979</pose>
      <name>maize_01_0413</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>1.8299513577628783 -3.0103309951875628 0.2918372252026984 0.0 0.0 5.317968990482784</pose>
      <name>maize_02_0414</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>1.8191342403864432 -2.8673589263958026 0.2918372252026984 0.0 0.0 3.7458212855981787</pose>
      <name>maize_02_0415</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>1.8141841118917341 -2.698682706341955 0.2918372252026984 0.0 0.0 2.5414237210364514</pose>
      <name>maize_01_0416</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>1.8257416821843133 -2.5463846479335586 0.2918372252026984 0.0 0.0 1.712784393340744</pose>
      <name>maize_02_0417</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>1.8160086258253285 -2.3993556749869707 0.2918372252026984 0.0 0.0 4.573348511098549</pose>
      <name>maize_02_0418</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>1.8302007060736258 -2.220016762592179 0.2918372252026984 0.0 0.0 3.480816241534504</pose>
      <name>maize_02_0419</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>1.8152721015552782 -2.0543194651969805 0.2918372252026984 0.0 0.0 3.0424043914900603</pose>
      <name>maize_01_0420</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>1.8207319202898846 -1.8753672895414546 0.2918372252026984 0.0 0.0 3.5491323583723093</pose>
      <name>maize_02_0421</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>1.8080641531360593 -1.6873609551880744 0.2918372252026984 0.0 0.0 5.134068211675712</pose>
      <name>maize_01_0422</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>1.8325060950167522 -1.541118650677114 0.2918372252026984 0.0 0.0 2.14744227119538</pose>
      <name>maize_02_0423</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>1.8038178966603917 -1.4081880818129262 0.2918372252026984 0.0 0.0 3.1941081713622625</pose>
      <name>maize_02_0424</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>1.8231373006048157 -1.212493365640766 0.2918372252026984 0.0 0.0 4.871580412210183</pose>
      <name>maize_01_0425</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>1.8087482800178964 -1.0619890112101311 0.2918372252026984 0.0 0.0 5.087648466171334</pose>
      <name>maize_01_0426</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>1.8175052899909767 -0.8892451082072261 0.2918372252026984 0.0 0.0 5.182787399498052</pose>
      <name>maize_02_0427</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>1.8257550227838468 -0.7509524062767179 0.2918372252026984 0.0 0.0 4.015287567797579</pose>
      <name>maize_02_0428</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>1.8155225668617305 -0.6307380531880691 0.2918372252026984 0.0 0.0 0.22989275469329365</pose>
      <name>maize_01_0429</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>1.8152746896921155 -0.41615506785028433 0.2918372252026984 0.0 0.0 2.3724863545388053</pose>
      <name>maize_01_0430</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>1.8143788575421649 -0.2850556863067881 0.2918372252026984 0.0 0.0 2.158802906589259</pose>
      <name>maize_01_0431</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>1.8076254074105211 -0.0815454271749072 0.2918372252026984 0.0 0.0 1.6982515774087759</pose>
      <name>maize_01_0432</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>1.821876916836306 0.0606656938971355 0.2918372252026984 0.0 0.0 1.0486999650850708</pose>
      <name>maize_01_0433</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>1.8157334539035377 0.22025844577969256 0.2918372252026984 0.0 0.0 2.3027291052878374</pose>
      <name>maize_01_0434</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>1.8308413565442478 0.37198325061151216 0.2918372252026984 0.0 0.0 5.5688448634323535</pose>
      <name>maize_01_0435</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>1.7987700340439634 0.5307198905566475 0.2918372252026984 0.0 0.0 6.067998944095289</pose>
      <name>maize_02_0436</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>1.8297417448859523 0.6570467786876533 0.2918372252026984 0.0 0.0 5.904393978275375</pose>
      <name>maize_02_0437</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>1.811880892157684 0.8284544572256163 0.2918372252026984 0.0 0.0 2.3410277253260694</pose>
      <name>maize_02_0438</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>1.8240214804233226 0.9911564307862344 0.2918372252026984 0.0 0.0 3.1389924026521507</pose>
      <name>maize_01_0439</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>1.809716775774489 1.1731767667865833 0.2918372252026984 0.0 0.0 2.0682088828669687</pose>
      <name>maize_02_0440</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>1.82499545169859 1.3352589601962066 0.2918372252026984 0.0 0.0 6.23070426993852</pose>
      <name>maize_02_0441</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>1.8218840568588774 1.4954914441572713 0.2918372252026984 0.0 0.0 2.315491942103324</pose>
      <name>maize_01_0442</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>1.8117189596403778 1.6559285273048365 0.2918372252026984 0.0 0.0 6.129848892712126</pose>
      <name>maize_01_0443</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>1.82397487855106 1.8101621824782672 0.2918372252026984 0.0 0.0 3.8853721749457435</pose>
      <name>maize_01_0444</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>1.8289198776872988 1.9926178414788733 0.2918372252026984 0.0 0.0 1.7122890526694283</pose>
      <name>maize_01_0445</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>1.8252039925298709 2.107380653430436 0.2918372252026984 0.0 0.0 3.7423797224913615</pose>
      <name>maize_02_0446</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>1.8309586156794886 2.2578406264517 0.2918372252026984 0.0 0.0 5.3834840943601066</pose>
      <name>maize_01_0447</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>1.8221621535450794 2.444207862941239 0.2918372252026984 0.0 0.0 4.838887028968765</pose>
      <name>maize_02_0448</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>1.831927152837193 2.6265390065167953 0.2918372252026984 0.0 0.0 2.9255241456168313</pose>
      <name>maize_01_0449</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>1.816087349147704 2.8174757907030408 0.2918372252026984 0.0 0.0 0.18650749479954795</pose>
      <name>maize_02_0450</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>1.7992213429593216 2.9688047968140774 0.2918372252026984 0.0 0.0 5.013415565585028</pose>
      <name>maize_02_0451</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>1.8229836218212832 3.146742944385422 0.2918372252026984 0.0 0.0 2.2113891069402176</pose>
      <name>maize_02_0452</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>1.8343103403310694 3.3265017908193935 0.2918372252026984 0.0 0.0 5.055974485959704</pose>
      <name>maize_02_0453</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>1.8144877242169661 3.4502662406233826 0.2918372252026984 0.0 0.0 3.6745921494836407</pose>
      <name>maize_01_0454</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>1.8143189428917568 3.620750850138024 0.2918372252026984 0.0 0.0 4.798363009247344</pose>
      <name>maize_01_0455</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>1.834639915172418 3.7679535372577933 0.2918372252026984 0.0 0.0 5.748794309996874</pose>
      <name>maize_01_0456</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>1.8049225597359806 3.9212594955955025 0.2918372252026984 0.0 0.0 0.7764860317290411</pose>
      <name>maize_02_0457</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>1.8174062034731264 4.092490885621252 0.2918372252026984 0.0 0.0 1.5622958686685182</pose>
      <name>maize_01_0458</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>1.81147913592251 4.270449925516137 0.2918372252026984 0.0 0.0 3.497526363726621</pose>
      <name>maize_01_0459</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>1.8032083792023776 4.423899251818816 0.2918372252026984 0.0 0.0 2.2913588643259897</pose>
      <name>maize_01_0460</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>1.7985922942768973 4.62707147544205 0.2918372252026984 0.0 0.0 1.1870446348188533</pose>
      <name>maize_01_0461</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>1.835414270027365 4.774740345740496 0.2918372252026984 0.0 0.0 2.556666074229793</pose>
      <name>maize_01_0462</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>1.8142390712988865 4.976821272407768 0.2918372252026984 0.0 0.0 3.2333826409012354</pose>
      <name>maize_01_0463</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>2.575475209270329 -5.070886188414573 0.2933670585041182 0.0 0.0 5.139281086318576</pose>
      <name>maize_01_0464</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>2.556074741802675 -3.993044389096895 0.2963292877710954 0.0 0.0 0.8524512436008357</pose>
      <name>maize_02_0465</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>2.575720664028874 -3.8025117992708775 0.2963292877710954 0.0 0.0 2.2605865319149765</pose>
      <name>maize_02_0466</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>2.5664120542560456 -3.650438507790331 0.2963292877710954 0.0 0.0 1.1577770772140226</pose>
      <name>maize_01_0467</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>2.5676115549727605 -3.476624899482786 0.2963292877710954 0.0 0.0 4.833644280191713</pose>
      <name>maize_01_0468</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>2.5709399651986233 -3.3016515305134866 0.2963292877710954 0.0 0.0 2.8160639143812296</pose>
      <name>maize_01_0469</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>2.549810618027209 -3.114263438338864 0.2963292877710954 0.0 0.0 3.6209131156545697</pose>
      <name>maize_01_0470</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>2.579565568947017 -2.962949595664294 0.2963292877710954 0.0 0.0 2.1524861177853833</pose>
      <name>maize_02_0471</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>2.585172322604503 -2.840053670448342 0.2963292877710954 0.0 0.0 1.2860276269145883</pose>
      <name>maize_01_0472</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>2.5643304229366777 -2.692179205237061 0.2963292877710954 0.0 0.0 2.05970171794084</pose>
      <name>maize_02_0473</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>2.5466479409138323 -2.5095574289009686 0.2963292877710954 0.0 0.0 2.2012509808608476</pose>
      <name>maize_01_0474</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>2.5818285990323444 -2.3545639908965983 0.2963292877710954 0.0 0.0 5.094081731791023</pose>
      <name>maize_02_0475</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>2.570451086119342 -2.177571451742745 0.2963292877710954 0.0 0.0 1.3451507746168418</pose>
      <name>maize_01_0476</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>2.554216962626243 -2.022197044515193 0.2963292877710954 0.0 0.0 0.4679284278551523</pose>
      <name>maize_01_0477</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>2.584582748201573 -1.8740522158784425 0.2963292877710954 0.0 0.0 1.2313539170887964</pose>
      <name>maize_01_0478</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>2.5778309430990527 -1.7053533740658207 0.2963292877710954 0.0 0.0 3.7047394654038945</pose>
      <name>maize_02_0479</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>2.555710292885097 -1.58194053807544 0.2963292877710954 0.0 0.0 2.109305123199748</pose>
      <name>maize_01_0480</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>2.5756048184458793 -1.4510269487464722 0.2963292877710954 0.0 0.0 0.00177454864369916</pose>
      <name>maize_01_0481</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>2.57366461102826 -1.2781630232394687 0.2963292877710954 0.0 0.0 5.863993243923704</pose>
      <name>maize_02_0482</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>2.569708665156619 -1.1458345772087664 0.2963292877710954 0.0 0.0 0.7912821188095087</pose>
      <name>maize_02_0483</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>2.5646441064912073 -0.9385877893030665 0.2963292877710954 0.0 0.0 6.1495728567959445</pose>
      <name>maize_02_0484</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>2.5801696642307395 -0.7953979576072685 0.2963292877710954 0.0 0.0 4.048479611078721</pose>
      <name>maize_02_0485</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>2.571541720808093 -0.6491858722337867 0.2963292877710954 0.0 0.0 1.6376153950326526</pose>
      <name>maize_02_0486</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>2.5602414436966043 -0.48006138791541364 0.2963292877710954 0.0 0.0 1.4545035070015493</pose>
      <name>maize_02_0487</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>2.5570614707419255 -0.35078786627030123 0.2963292877710954 0.0 0.0 2.0202170745590253</pose>
      <name>maize_01_0488</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>2.562998958831484 -0.1803134720155235 0.2963292877710954 0.0 0.0 0.9908915840690025</pose>
      <name>maize_01_0489</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>2.5556961830895313 -0.013809035114541679 0.2963292877710954 0.0 0.0 2.6450200276170004</pose>
      <name>maize_02_0490</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>2.5732270674249698 0.162306307003524 0.2963292877710954 0.0 0.0 4.3579970637852545</pose>
      <name>maize_02_0491</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>2.550261897725478 0.3233438571145584 0.2963292877710954 0.0 0.0 1.238322299785758</pose>
      <name>maize_01_0492</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>2.575009149884722 0.5146218659592625 0.2963292877710954 0.0 0.0 1.6856946498055392</pose>
      <name>maize_01_0493</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>2.5524038224798673 0.6328944300457344 0.2963292877710954 0.0 0.0 0.7885316128518108</pose>
      <name>maize_01_0494</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>2.5756359988173223 0.775399235830994 0.2963292877710954 0.0 0.0 0.8735777214051358</pose>
      <name>maize_02_0495</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>2.5763686258571825 0.9486454883737041 0.2963292877710954 0.0 0.0 1.6165789318960473</pose>
      <name>maize_02_0496</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>2.5746871218971226 1.0863754456847259 0.2963292877710954 0.0 0.0 2.559426886927097</pose>
      <name>maize_02_0497</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>2.584344981698785 1.274474979354551 0.2963292877710954 0.0 0.0 2.480458287678998</pose>
      <name>maize_02_0498</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>2.56792519623841 1.411374791744814 0.2963292877710954 0.0 0.0 4.434988409618544</pose>
      <name>maize_01_0499</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>2.5718068600606285 1.5606850697841521 0.2963292877710954 0.0 0.0 0.92254123685831</pose>
      <name>maize_01_0500</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>2.5822066329796574 1.7394428072122983 0.2963292877710954 0.0 0.0 0.4278229434872883</pose>
      <name>maize_02_0501</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>2.5708386873118227 1.8599169862745528 0.2963292877710954 0.0 0.0 2.867951329857533</pose>
      <name>maize_01_0502</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>2.565402850732755 2.031087964175353 0.2963292877710954 0.0 0.0 3.48408768300121</pose>
      <name>maize_02_0503</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>2.550868084218739 2.2131603843616583 0.2963292877710954 0.0 0.0 0.24811783559863268</pose>
      <name>maize_02_0504</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>2.571589231635281 2.3324559996799206 0.2963292877710954 0.0 0.0 0.8685951577650025</pose>
      <name>maize_01_0505</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>2.5482912278261027 2.5182954694164756 0.2963292877710954 0.0 0.0 4.020212361240479</pose>
      <name>maize_02_0506</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>2.5860891877908188 2.675020389147665 0.2963292877710954 0.0 0.0 3.8701801934403206</pose>
      <name>maize_02_0507</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>2.5562117165859597 2.817915970038335 0.2963292877710954 0.0 0.0 4.122900711603466</pose>
      <name>maize_02_0508</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>2.5721040363859764 3.0027773047091886 0.2963292877710954 0.0 0.0 2.0522063928055174</pose>
      <name>maize_01_0509</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>2.556812961799648 3.1919665689375654 0.2963292877710954 0.0 0.0 2.723984052570594</pose>
      <name>maize_01_0510</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>2.5682480902084794 4.011437744989903 0.29245024329647806 0.0 0.0 4.2571390895800745</pose>
      <name>maize_02_0511</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>2.5688276277604505 4.134429398931942 0.29245024329647806 0.0 0.0 2.342632751020782</pose>
      <name>maize_01_0512</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>2.582056412904648 4.296309298252495 0.29245024329647806 0.0 0.0 2.7075035677258517</pose>
      <name>maize_01_0513</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>2.553456355265612 4.463756806655664 0.29245024329647806 0.0 0.0 5.305429951120262</pose>
      <name>maize_02_0514</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>2.572665405200576 4.640391764553118 0.29245024329647806 0.0 0.0 0.9591500411908032</pose>
      <name>maize_02_0515</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>2.5778737817560637 4.809617352021287 0.29245024329647806 0.0 0.0 1.459565071760566</pose>
      <name>maize_02_0516</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>2.581657680469201 4.958437210721272 0.29245024329647806 0.0 0.0 1.4424252487278844</pose>
      <name>maize_02_0517</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>3.331330635938352 -5.068806939541005 0.29529128244130187 0.0 0.0 2.5895056319738887</pose>
      <name>maize_01_0518</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>3.305612538558533 -4.867902239746203 0.29529128244130187 0.0 0.0 2.109830777634846</pose>
      <name>maize_02_0519</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>3.301399356212178 -4.71587234993552 0.29529128244130187 0.0 0.0 4.83069165370643</pose>
      <name>maize_01_0520</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>3.309786722237339 -4.587748042995561 0.29529128244130187 0.0 0.0 4.195231486741177</pose>
      <name>maize_01_0521</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>3.332113784123295 -4.4110298646859185 0.29529128244130187 0.0 0.0 4.1436691807857295</pose>
      <name>maize_01_0522</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>3.3188798924890612 -4.267087438180458 0.29529128244130187 0.0 0.0 4.277250825222418</pose>
      <name>maize_01_0523</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>3.300987349527648 -4.120805164401788 0.29529128244130187 0.0 0.0 2.508776078846899</pose>
      <name>maize_01_0524</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>3.312285881151153 -3.9890307054123175 0.29529128244130187 0.0 0.0 0.32374652286500505</pose>
      <name>maize_01_0525</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>3.3150420806355685 -3.8419519544039336 0.29529128244130187 0.0 0.0 1.1481479915853119</pose>
      <name>maize_01_0526</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>3.2975704597604647 -3.704623583671041 0.29529128244130187 0.0 0.0 1.4220357210137549</pose>
      <name>maize_01_0527</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>3.3203081639936176 -3.525554104804749 0.29529128244130187 0.0 0.0 3.9724949899524575</pose>
      <name>maize_01_0528</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>3.3022165406172697 -3.209842038212316 0.30068614695107165 0.0 0.0 4.453287849905427</pose>
      <name>maize_01_0529</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>3.307948960005098 -3.0560645655444323 0.30068614695107165 0.0 0.0 1.9598462386046098</pose>
      <name>maize_01_0530</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>3.316825195369134 -2.905777816620282 0.30068614695107165 0.0 0.0 5.830584010741264</pose>
      <name>maize_02_0531</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>3.3046395978925727 -2.738778084147521 0.30068614695107165 0.0 0.0 6.108932859694953</pose>
      <name>maize_01_0532</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>3.332056440529497 -2.5555759365219815 0.30068614695107165 0.0 0.0 4.706444341906088</pose>
      <name>maize_01_0533</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>3.322649245059214 -2.402359559839601 0.30068614695107165 0.0 0.0 6.1937971329625245</pose>
      <name>maize_01_0534</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>3.322578595208036 -2.255749690542292 0.30068614695107165 0.0 0.0 2.696752903150091</pose>
      <name>maize_02_0535</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>3.2963801583450714 -2.0578367944521903 0.30068614695107165 0.0 0.0 0.33041768354680234</pose>
      <name>maize_01_0536</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>3.3251905083430238 -1.9075791329763838 0.30068614695107165 0.0 0.0 5.368082461619131</pose>
      <name>maize_01_0537</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>3.3043309957057776 -1.7644494342912425 0.30068614695107165 0.0 0.0 3.2555789275540588</pose>
      <name>maize_01_0538</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>3.307273043565509 -1.5816751648313776 0.30068614695107165 0.0 0.0 4.874600751293297</pose>
      <name>maize_01_0539</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>3.2975402601835704 -1.4280136536799972 0.30068614695107165 0.0 0.0 3.310262293806041</pose>
      <name>maize_02_0540</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>3.3233366605539674 -1.2790097361640118 0.30068614695107165 0.0 0.0 1.8413862387719138</pose>
      <name>maize_01_0541</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>3.327179733604945 -1.1418088274968272 0.30068614695107165 0.0 0.0 0.2093886486485677</pose>
      <name>maize_01_0542</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>3.3256119072863015 -1.0167253298799093 0.30068614695107165 0.0 0.0 5.455685941634258</pose>
      <name>maize_01_0543</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>3.3090359197969352 -0.8378461937205026 0.30068614695107165 0.0 0.0 4.372460897020319</pose>
      <name>maize_02_0544</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>3.2966656250800774 -0.6850555625987731 0.30068614695107165 0.0 0.0 6.092167791949493</pose>
      <name>maize_01_0545</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>3.3163841430218293 -0.5486926046594816 0.30068614695107165 0.0 0.0 4.146600607126494</pose>
      <name>maize_01_0546</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>3.326658631853072 -0.3758607449692528 0.30068614695107165 0.0 0.0 6.224997869699028</pose>
      <name>maize_01_0547</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>3.33589074993277 -0.21121855257882238 0.30068614695107165 0.0 0.0 5.568662963745979</pose>
      <name>maize_01_0548</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>3.321145825662983 -0.027416874604573316 0.30068614695107165 0.0 0.0 0.13256035883309492</pose>
      <name>maize_02_0549</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>3.321781532729654 0.13799866475798517 0.30068614695107165 0.0 0.0 4.6308775604242935</pose>
      <name>maize_01_0550</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>3.314688914027599 0.315409042280713 0.30068614695107165 0.0 0.0 1.0228766572683328</pose>
      <name>maize_01_0551</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>3.3293152285941767 0.4385028535669937 0.30068614695107165 0.0 0.0 5.285616213177664</pose>
      <name>maize_02_0552</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>3.3073619971472903 0.6070830824611173 0.30068614695107165 0.0 0.0 3.5584484264918346</pose>
      <name>maize_02_0553</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>3.3122703158072584 0.8035014197290336 0.30068614695107165 0.0 0.0 4.9976375477473285</pose>
      <name>maize_02_0554</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>3.3026336010450548 0.9802050838903682 0.30068614695107165 0.0 0.0 4.651191917393255</pose>
      <name>maize_02_0555</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>3.317277768525206 1.1116739319198947 0.30068614695107165 0.0 0.0 0.6992182842946117</pose>
      <name>maize_02_0556</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>3.3120976681968273 1.2806674487477148 0.30068614695107165 0.0 0.0 0.04772855762638532</pose>
      <name>maize_01_0557</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>3.324951198462629 1.4093972692502472 0.30068614695107165 0.0 0.0 3.59014907367544</pose>
      <name>maize_02_0558</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>3.2991572098432425 1.6120719139498991 0.30068614695107165 0.0 0.0 1.8443352055152489</pose>
      <name>maize_02_0559</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>3.3029917553661887 1.7340572844207083 0.30068614695107165 0.0 0.0 4.876910148434578</pose>
      <name>maize_01_0560</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>3.307610606666083 1.896704370779032 0.30068614695107165 0.0 0.0 2.553406269741312</pose>
      <name>maize_02_0561</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>3.3022343849183633 2.0335244328979263 0.30068614695107165 0.0 0.0 3.7923832952090915</pose>
      <name>maize_01_0562</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>3.2971248761974037 2.1938261711459415 0.30068614695107165 0.0 0.0 0.22864291997156563</pose>
      <name>maize_02_0563</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>3.305506504251362 2.3588698255184966 0.30068614695107165 0.0 0.0 5.670926161847847</pose>
      <name>maize_02_0564</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>3.324196684001249 2.5153537963359307 0.30068614695107165 0.0 0.0 3.4096495741500195</pose>
      <name>maize_01_0565</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>3.310769899951116 2.646027148408681 0.30068614695107165 0.0 0.0 5.0664535503551</pose>
      <name>maize_01_0566</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>3.3036942561116565 2.7937652657567176 0.30068614695107165 0.0 0.0 3.522157714442807</pose>
      <name>maize_01_0567</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>3.3227652327532047 2.9876728399870105 0.30068614695107165 0.0 0.0 5.151688254945536</pose>
      <name>maize_01_0568</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>3.3314339196246694 3.1588266425075977 0.30068614695107165 0.0 0.0 0.6473136790838828</pose>
      <name>maize_02_0569</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>3.331504197815093 3.3189228340798023 0.30068614695107165 0.0 0.0 3.279709832878908</pose>
      <name>maize_02_0570</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>3.3201938971294815 3.5236344340872607 0.30068614695107165 0.0 0.0 0.10414150044357587</pose>
      <name>maize_02_0571</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>3.317016954779585 3.6745158765563746 0.30068614695107165 0.0 0.0 1.582019763914752</pose>
      <name>maize_01_0572</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>3.3221559488225347 3.8684370874109595 0.30068614695107165 0.0 0.0 2.203485589097451</pose>
      <name>maize_02_0573</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>3.3346331052422187 4.044090491951759 0.30068614695107165 0.0 0.0 3.686342784445853</pose>
      <name>maize_02_0574</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>3.309682009465952 4.200885059842643 0.30068614695107165 0.0 0.0 3.344125068645913</pose>
      <name>maize_02_0575</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>3.308532905085462 4.365908038569409 0.30068614695107165 0.0 0.0 6.137741401195502</pose>
      <name>maize_01_0576</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>3.3193237409973477 4.552005082134815 0.30068614695107165 0.0 0.0 1.6668166776682067</pose>
      <name>maize_02_0577</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_02</uri>
      <pose>3.322520982339409 4.741993435342839 0.30068614695107165 0.0 0.0 4.618100914337945</pose>
      <name>maize_02_0578</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>3.3182720506500996 4.879572260201788 0.30068614695107165 0.0 0.0 5.8200929411586015</pose>
      <name>maize_01_0579</name>
      <static>false</static>
    </include>
    
    <include>
      <uri>model://maize_01</uri>
      <pose>3.3341425317132742 5.007014762715236 0.30068614695107165 0.0 0.0 1.571426042305542</pose>
      <name>maize_01_0580</name>
      <static>false</static>
    </include>
    <model name='ghost_nettle_0000'>
      <pose frame=''>3.206805271272259 -3.1285579994065134 0.30068614695107165 0.0 0.0 1.958735413889797</pose>
      <static>true</static>
      <link name='nettle_0000_link'>
        <visual name="visual">
        <geometry>
          <mesh>
            <uri>model://nettle/meshes/nettle.dae</uri>
          </mesh>
        </geometry>
        <material>
          <script>
            <uri>model://nettle/materials/scripts/</uri>
            <uri>model://nettle/materials/textures/</uri>
            <name>nettle_leaf</name>
          </script>
        </material>
      </visual>
    <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
    </model>
    
    <model name='ghost_unknown_weed_0001'>
      <pose frame=''>3.262869503326823 -3.2011052792261836 0.30068614695107165 0.0 0.0 4.691347005989413</pose>
      <static>true</static>
      <link name='unknown_weed_0001_link'>
        <visual name="visual">
        <geometry>
          <mesh>
            <uri>model://unknown_weed/meshes/unknown_weed.dae</uri>
          </mesh>
        </geometry>
        <material>
          <script>
            <uri>model://unknown_weed/materials/scripts/</uri>
            <uri>model://unknown_weed/materials/textures/</uri>
            <name>unknown_weed_leaf</name>
          </script>
        </material>
      </visual>
    <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
    </model>
    
    <model name='ghost_nettle_0002'>
      <pose frame=''>0.6089677917592997 -1.3915982474112267 0.2928726102947117 0.0 0.0 4.246125037363039</pose>
      <static>true</static>
      <link name='nettle_0002_link'>
        <visual name="visual">
        <geometry>
          <mesh>
            <uri>model://nettle/meshes/nettle.dae</uri>
          </mesh>
        </geometry>
        <material>
          <script>
            <uri>model://nettle/materials/scripts/</uri>
            <uri>model://nettle/materials/textures/</uri>
            <name>nettle_leaf</name>
          </script>
        </material>
      </visual>
    <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
    </model>
    
    <model name='ghost_nettle_0003'>
      <pose frame=''>-2.9056607991095342 -1.9578982151768227 0.2907688925864437 0.0 0.0 2.0654489450884728</pose>
      <static>true</static>
      <link name='nettle_0003_link'>
        <visual name="visual">
        <geometry>
          <mesh>
            <uri>model://nettle/meshes/nettle.dae</uri>
          </mesh>
        </geometry>
        <material>
          <script>
            <uri>model://nettle/materials/scripts/</uri>
            <uri>model://nettle/materials/textures/</uri>
            <name>nettle_leaf</name>
          </script>
        </material>
      </visual>
    <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
    </model>
    
    <model name='ghost_nettle_0004'>
      <pose frame=''>-3.3692890426615296 -1.6203229464128266 0.29114602237360276 0.0 0.0 2.0706089261308716</pose>
      <static>true</static>
      <link name='nettle_0004_link'>
        <visual name="visual">
        <geometry>
          <mesh>
            <uri>model://nettle/meshes/nettle.dae</uri>
          </mesh>
        </geometry>
        <material>
          <script>
            <uri>model://nettle/materials/scripts/</uri>
            <uri>model://nettle/materials/textures/</uri>
            <name>nettle_leaf</name>
          </script>
        </material>
      </visual>
    <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
    </model>
    
    <model name='ghost_unknown_weed_0005'>
      <pose frame=''>0.7573125189366321 4.969543151715542 0.2890915899789252 0.0 0.0 1.3630792526079163</pose>
      <static>true</static>
      <link name='unknown_weed_0005_link'>
        <visual name="visual">
        <geometry>
          <mesh>
            <uri>model://unknown_weed/meshes/unknown_weed.dae</uri>
          </mesh>
        </geometry>
        <material>
          <script>
            <uri>model://unknown_weed/materials/scripts/</uri>
            <uri>model://unknown_weed/materials/textures/</uri>
            <name>unknown_weed_leaf</name>
          </script>
        </material>
      </visual>
    <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
    </model>
    
    <model name='ghost_unknown_weed_0006'>
      <pose frame=''>-0.16280823940335898 0.06500975527259634 0.29382564493481345 0.0 0.0 2.295745315276478</pose>
      <static>true</static>
      <link name='unknown_weed_0006_link'>
        <visual name="visual">
        <geometry>
          <mesh>
            <uri>model://unknown_weed/meshes/unknown_weed.dae</uri>
          </mesh>
        </geometry>
        <material>
          <script>
            <uri>model://unknown_weed/materials/scripts/</uri>
            <uri>model://unknown_weed/materials/textures/</uri>
            <name>unknown_weed_leaf</name>
          </script>
        </material>
      </visual>
    <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
    </model>
    
    <model name='ghost_nettle_0007'>
      <pose frame=''>2.028872736411703 -3.89169385048075 0.2918372252026984 0.0 0.0 5.272380128427541</pose>
      <static>true</static>
      <link name='nettle_0007_link'>
        <visual name="visual">
        <geometry>
          <mesh>
            <uri>model://nettle/meshes/nettle.dae</uri>
          </mesh>
        </geometry>
        <material>
          <script>
            <uri>model://nettle/materials/scripts/</uri>
            <uri>model://nettle/materials/textures/</uri>
            <name>nettle_leaf</name>
          </script>
        </material>
      </visual>
    <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
    </model>
    
    <model name='ghost_nettle_0008'>
      <pose frame=''>-1.8150626598795803 -1.7636972363724088 0.29805819867314265 0.0 0.0 1.259828595853124</pose>
      <static>true</static>
      <link name='nettle_0008_link'>
        <visual name="visual">
        <geometry>
          <mesh>
            <uri>model://nettle/meshes/nettle.dae</uri>
          </mesh>
        </geometry>
        <material>
          <script>
            <uri>model://nettle/materials/scripts/</uri>
            <uri>model://nettle/materials/textures/</uri>
            <name>nettle_leaf</name>
          </script>
        </material>
      </visual>
    <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
    </model>
    
    <model name='ghost_nettle_0009'>
      <pose frame=''>-1.8133706210563194 -0.7926720882024867 0.29805819867314265 0.0 0.0 0.28473936679227113</pose>
      <static>true</static>
      <link name='nettle_0009_link'>
        <visual name="visual">
        <geometry>
          <mesh>
            <uri>model://nettle/meshes/nettle.dae</uri>
          </mesh>
        </geometry>
        <material>
          <script>
            <uri>model://nettle/materials/scripts/</uri>
            <uri>model://nettle/materials/textures/</uri>
            <name>nettle_leaf</name>
          </script>
        </material>
      </visual>
    <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
    </model>
    
    <model name='ghost_nettle_0010'>
      <pose frame=''>0.22320159973298814 -4.074395055158598 0.2928726102947117 0.0 0.0 4.588747322503437</pose>
      <static>true</static>
      <link name='nettle_0010_link'>
        <visual name="visual">
        <geometry>
          <mesh>
            <uri>model://nettle/meshes/nettle.dae</uri>
          </mesh>
        </geometry>
        <material>
          <script>
            <uri>model://nettle/materials/scripts/</uri>
            <uri>model://nettle/materials/textures/</uri>
            <name>nettle_leaf</name>
          </script>
        </material>
      </visual>
    <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
    </model>
    
    <model name='ghost_unknown_weed_0011'>
      <pose frame=''>-0.28593010818183107 -0.1607831896703269 0.29382564493481345 0.0 0.0 4.119361143845086</pose>
      <static>true</static>
      <link name='unknown_weed_0011_link'>
        <visual name="visual">
        <geometry>
          <mesh>
            <uri>model://unknown_weed/meshes/unknown_weed.dae</uri>
          </mesh>
        </geometry>
        <material>
          <script>
            <uri>model://unknown_weed/materials/scripts/</uri>
            <uri>model://unknown_weed/materials/textures/</uri>
            <name>unknown_weed_leaf</name>
          </script>
        </material>
      </visual>
    <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
    </model>
    
    <model name='ghost_nettle_0012'>
      <pose frame=''>-2.1927038831525594 1.6703839539795533 0.2933576391843192 0.0 0.0 2.911884703483416</pose>
      <static>true</static>
      <link name='nettle_0012_link'>
        <visual name="visual">
        <geometry>
          <mesh>
            <uri>model://nettle/meshes/nettle.dae</uri>
          </mesh>
        </geometry>
        <material>
          <script>
            <uri>model://nettle/materials/scripts/</uri>
            <uri>model://nettle/materials/textures/</uri>
            <name>nettle_leaf</name>
          </script>
        </material>
      </visual>
    <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
    </model>
    
    <model name='ghost_unknown_weed_0013'>
      <pose frame=''>2.4926952621086 0.08216895914296174 0.2963292877710954 0.0 0.0 0.6476255799904461</pose>
      <static>true</static>
      <link name='unknown_weed_0013_link'>
        <visual name="visual">
        <geometry>
          <mesh>
            <uri>model://unknown_weed/meshes/unknown_weed.dae</uri>
          </mesh>
        </geometry>
        <material>
          <script>
            <uri>model://unknown_weed/materials/scripts/</uri>
            <uri>model://unknown_weed/materials/textures/</uri>
            <name>unknown_weed_leaf</name>
          </script>
        </material>
      </visual>
    <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
    </model>
    
    <model name='ghost_nettle_0014'>
      <pose frame=''>-3.0022765614293014 -0.9578762727818892 0.2907688925864437 0.0 0.0 2.6115412643676823</pose>
      <static>true</static>
      <link name='nettle_0014_link'>
        <visual name="visual">
        <geometry>
          <mesh>
            <uri>model://nettle/meshes/nettle.dae</uri>
          </mesh>
        </geometry>
        <material>
          <script>
            <uri>model://nettle/materials/scripts/</uri>
            <uri>model://nettle/materials/textures/</uri>
            <name>nettle_leaf</name>
          </script>
        </material>
      </visual>
    <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
    </model>
    
    <model name='ghost_unknown_weed_0015'>
      <pose frame=''>0.8726865176798144 -3.539489760161425 0.2919503043211369 0.0 0.0 2.78329972728935</pose>
      <static>true</static>
      <link name='unknown_weed_0015_link'>
        <visual name="visual">
        <geometry>
          <mesh>
            <uri>model://unknown_weed/meshes/unknown_weed.dae</uri>
          </mesh>
        </geometry>
        <material>
          <script>
            <uri>model://unknown_weed/materials/scripts/</uri>
            <uri>model://unknown_weed/materials/textures/</uri>
            <name>unknown_weed_leaf</name>
          </script>
        </material>
      </visual>
    <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
    </model>
    
    <model name='ghost_nettle_0016'>
      <pose frame=''>2.561902840775651 1.5554644868129701 0.2963292877710954 0.0 0.0 0.5881369361181411</pose>
      <static>true</static>
      <link name='nettle_0016_link'>
        <visual name="visual">
        <geometry>
          <mesh>
            <uri>model://nettle/meshes/nettle.dae</uri>
          </mesh>
        </geometry>
        <material>
          <script>
            <uri>model://nettle/materials/scripts/</uri>
            <uri>model://nettle/materials/textures/</uri>
            <name>nettle_leaf</name>
          </script>
        </material>
      </visual>
    <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
    </model>
    
    <model name='ghost_unknown_weed_0017'>
      <pose frame=''>2.3107692577924936 -2.7115478643722466 0.2963292877710954 0.0 0.0 2.574332957851102</pose>
      <static>true</static>
      <link name='unknown_weed_0017_link'>
        <visual name="visual">
        <geometry>
          <mesh>
            <uri>model://unknown_weed/meshes/unknown_weed.dae</uri>
          </mesh>
        </geometry>
        <material>
          <script>
            <uri>model://unknown_weed/materials/scripts/</uri>
            <uri>model://unknown_weed/materials/textures/</uri>
            <name>unknown_weed_leaf</name>
          </script>
        </material>
      </visual>
    <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
    </model>
    
    <model name='ghost_unknown_weed_0018'>
      <pose frame=''>1.6197037912068986 -3.854308785869816 0.2918372252026984 0.0 0.0 4.966560257372089</pose>
      <static>true</static>
      <link name='unknown_weed_0018_link'>
        <visual name="visual">
        <geometry>
          <mesh>
            <uri>model://unknown_weed/meshes/unknown_weed.dae</uri>
          </mesh>
        </geometry>
        <material>
          <script>
            <uri>model://unknown_weed/materials/scripts/</uri>
            <uri>model://unknown_weed/materials/textures/</uri>
            <name>unknown_weed_leaf</name>
          </script>
        </material>
      </visual>
    <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
    </model>
    
    <model name='ghost_unknown_weed_0019'>
      <pose frame=''>1.7277056739288952 -1.0411767610041682 0.2918372252026984 0.0 0.0 2.5464629663662173</pose>
      <static>true</static>
      <link name='unknown_weed_0019_link'>
        <visual name="visual">
        <geometry>
          <mesh>
            <uri>model://unknown_weed/meshes/unknown_weed.dae</uri>
          </mesh>
        </geometry>
        <material>
          <script>
            <uri>model://unknown_weed/materials/scripts/</uri>
            <uri>model://unknown_weed/materials/textures/</uri>
            <name>unknown_weed_leaf</name>
          </script>
        </material>
      </visual>
    <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
    </model>
    """

    return crops_weeds

def create_world_file(num_of_drones, home_area, coverage_radii, file_path):
    world_file = open(file_path, "w")

    world_file.write("<sdf version='1.7'>\n")
    world_file.write("<world name='default'>\n")
    world_file.write(generate_physics())
    # world_file.write(generate_ground_plane())
    world_file.write(generate_heightmap())
    world_file.write(generate_crops_weeds())
    world_file.write(generate_light())

    # world_file.write(generate_coverage_area(coverage_radii))
    # for drone in generate_drones_in_grid(home_area, num_of_drones):
    #     world_file.write(drone)
    for drone in generate_drones_at_home(num_of_drones, x_sep=10, y_ext=6):
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

