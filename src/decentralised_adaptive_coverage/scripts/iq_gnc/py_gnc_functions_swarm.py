# -*- coding: utf-8 -*-
from iq_gnc.PrintColours import *
import rospy
import numpy as np
from math import atan2, pow, sqrt, degrees, radians, sin, cos
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from nav_msgs.msg import Odometry
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandTOL, CommandTOLRequest
from mavros_msgs.srv import CommandLong, CommandLongRequest
from mavros_msgs.srv import CommandBool, CommandBoolRequest
from mavros_msgs.srv import SetMode, SetModeRequest

"""Control Functions
	This module is designed to make high level control programming simple.
"""


class gnc_api:
    def __init__(self):
        """This function is called at the beginning of a program and will start of the communication links to the FCU.
        """
        self.current_state_g = State()
        self.current_pose_g = Odometry()
        self.neighbour_pose_g = Odometry()
        self.correction_vector_g = Pose()
        self.local_offset_pose_g = Point()
        self.waypoint_g = PoseStamped()

        self.current_heading_g = 0.0
        self.local_offset_g = 0.0
        self.correction_heading_g = 0.0
        self.local_desired_heading_g = 0.0

        self.ns = rospy.get_namespace()
        self.neighbor_ns = rospy.get_param('~neighbour_namespace', '/')

        self.is_leader = rospy.get_param('~leader', 'false')

        if self.ns == "/":
            rospy.loginfo(CBLUE2 + "Using default namespace" + CEND)
        else:
            rospy.loginfo(CBLUE2 + "Using {} namespace".format(self.ns) + CEND)

        self.local_pos_pub = rospy.Publisher(
            name="{}mavros/setpoint_position/local".format(self.ns),
            data_class=PoseStamped,
            queue_size=10,
        )

        self.currentPos = rospy.Subscriber(
            name="{}mavros/global_position/local".format(self.ns),
            data_class=Odometry,
            queue_size=10,
            callback=self.pose_cb,
        )

        self.neighborPos = rospy.Subscriber(
            name="{}mavros/global_position/local".format(self.neighbor_ns),
            data_class=Odometry,
            queue_size=10,
            callback=self.neighbor_pose_cb,
        )

        self.state_sub = rospy.Subscriber(
            name="{}mavros/state".format(self.ns),
            data_class=State,
            queue_size=10,
            callback=self.state_cb,
        )

        rospy.wait_for_service("{}mavros/cmd/arming".format(self.ns))

        self.arming_client = rospy.ServiceProxy(
            name="{}mavros/cmd/arming".format(self.ns), service_class=CommandBool
        )

        rospy.wait_for_service("{}mavros/cmd/land".format(self.ns))

        self.land_client = rospy.ServiceProxy(
            name="{}mavros/cmd/land".format(self.ns), service_class=CommandTOL
        )

        rospy.wait_for_service("{}mavros/cmd/takeoff".format(self.ns))

        self.takeoff_client = rospy.ServiceProxy(
            name="{}mavros/cmd/takeoff".format(self.ns), service_class=CommandTOL
        )

        rospy.wait_for_service("{}mavros/set_mode".format(self.ns))

        self.set_mode_client = rospy.ServiceProxy(
            name="{}mavros/set_mode".format(self.ns), service_class=SetMode
        )

        rospy.wait_for_service("{}mavros/cmd/command".format(self.ns))

        self.command_client = rospy.ServiceProxy(
            name="{}mavros/cmd/command".format(self.ns), service_class=CommandLong
        )
        rospy.loginfo(CBOLD + CGREEN2 + "Initialization Complete." + CEND)

    def state_cb(self, message):
        self.current_state_g = message

    def pose_cb(self, msg):
        """Gets the raw pose of the drone and processes it for use in control.

        Args:
                msg (geometry_msgs/Pose): Raw pose of the drone.
        """
        self.current_pose_g = msg
        # self.enu_2_local_self()
        self.enu_2_local(self.current_pose_g, self.local_offset_g)

        q0, q1, q2, q3 = (
            self.current_pose_g.pose.pose.orientation.w,
            self.current_pose_g.pose.pose.orientation.x,
            self.current_pose_g.pose.pose.orientation.y,
            self.current_pose_g.pose.pose.orientation.z,
        )

        psi = atan2((2 * (q0 * q3 + q1 * q2)),
                    (1 - 2 * (pow(q2, 2) + pow(q3, 2))))

        self.current_heading_g = degrees(psi) - self.local_offset_g

    def neighbor_pose_cb(self, msg):
        """Callback for the neighbor's pose.

        Args:
            msg (Odometry): The neighbor's pose.
        """
        self.neighbour_pose_g = msg

    def enu_2_local_self(self):
        x, y, z = (
            self.current_pose_g.pose.pose.position.x,
            self.current_pose_g.pose.pose.position.y,
            self.current_pose_g.pose.pose.position.z,
        )

        current_pos_local = Point()

        current_pos_local.x = x * cos(radians((self.local_offset_g - 90))) - y * sin(
            radians((self.local_offset_g - 90))
        )

        current_pos_local.y = x * sin(radians((self.local_offset_g - 90))) + y * cos(
            radians((self.local_offset_g - 90))
        )

        current_pos_local.z = z

        return current_pos_local

    def enu_2_local(self, pose:Odometry, local_offset):
        """Convert a position from the ENU coordinate system to a local coordinate system.

        Args:
            pos (Odometry): The pose in the ENU coordinate system.
            local_offset (float): The angle (in degrees) by which the local coordinate system is rotated around the z-axis.

        Returns:
            Point: The position in the local coordinate system.
        """

        position = pose.pose.pose.position
        x, y, z = position.x, position.y, position.z

        current_pos_local = Point()

        current_pos_local.x = x * cos(radians((local_offset - 90))) - y * sin(
            radians((local_offset - 90))
        )

        current_pos_local.y = x * sin(radians((local_offset - 90))) + y * cos(
            radians((local_offset - 90))
        )

        current_pos_local.z = z

        return current_pos_local


    def get_heading(self, pose:Odometry):
        """Get the heading from a pose message.

        Args:
            pose (Odometry): The pose message.

        Returns:
            float: The heading in degrees.
        """
        # Extract the orientation quaternion from the pose message.
        q0, q1, q2, q3 = (
            pose.pose.pose.orientation.w,
            pose.pose.pose.orientation.x,
            pose.pose.pose.orientation.y,
            pose.pose.pose.orientation.z,
        )

        # Convert the quaternion to a heading (yaw) using a quaternion-to-Euler-angle conversion.
        psi = atan2((2 * (q0 * q3 + q1 * q2)),
                    (1 - 2 * (pow(q2, 2) + pow(q3, 2))))

        # Convert the heading to degrees and return it.
        return degrees(psi)


    def get_current_heading(self):
        """Returns the current heading of the drone.

        Returns:
            Heading (Float): θ in is degrees.
        """
        return self.current_heading_g

    def get_current_location(self):
        """Returns the current position of the drone.

        Returns:
            Position (geometry_msgs.Point()): Returns position of type geometry_msgs.Point().
        """
        return self.enu_2_local(self.current_pose_g, self.local_offset_g)

    def get_neighbor_position_in_local_frame(self):
        """Get the neighbor's position in the local frame.

        Returns:
            Point: The neighbor's position in the local frame.
        """
        # Get the neighbor's position in its own local frame.
        neighbor_pos_local = self.enu_2_local(self.neighbour_pose_g, self.local_offset_g)

        # Calculate the offset heading between the two drones.
        offset_heading = self.current_heading_g - self.get_heading(self.neighbour_pose_g)

        # Rotate the neighbor's position based on the offset heading.
        theta = radians(offset_heading)
        x = neighbor_pos_local.x * cos(theta) - neighbor_pos_local.y * sin(theta)
        y = neighbor_pos_local.x * sin(theta) + neighbor_pos_local.y * cos(theta)
        z = neighbor_pos_local.z

        return Point(x, y, z)


    def get_distance_to_neighbor(self):
        """Calculate the Euclidean distance to the neighbor.

        Returns:
            float: The distance to the neighbor.
        """
        dx = self.current_pose_g.pose.pose.position.x - self.neighbour_pose_g.pose.pose.position.x
        dy = self.current_pose_g.pose.pose.position.y - self.neighbour_pose_g.pose.pose.position.y
        dz = self.current_pose_g.pose.pose.position.z - self.neighbour_pose_g.pose.pose.position.z
        return sqrt(dx**2 + dy**2 + dz**2)

    def land(self):
        """The function changes the mode of the drone to LAND.

        Returns:
                0 (int): LAND successful
                -1 (int): LAND unsuccessful.
        """
        srv_land = CommandTOLRequest(0, 0, 0, 0, 0)
        response = self.land_client(srv_land)
        if response.success:
            rospy.loginfo(
                CGREEN2 + "Land Sent {}".format(str(response.success)) + CEND)
            return 0
        else:
            rospy.logerr(CRED2 + "Landing failed" + CEND)
            return -1

    def wait4connect(self):
        """Wait for connect is a function that will hold the program until communication with the FCU is established.

        Returns:
                0 (int): Connected to FCU.
                -1 (int): Failed to connect to FCU.
        """
        rospy.loginfo(CYELLOW2 + "Waiting for FCU connection" + CEND)
        while not rospy.is_shutdown() and not self.current_state_g.connected:
            rospy.sleep(0.01)
        else:
            if self.current_state_g.connected:
                rospy.loginfo(CGREEN2 + "FCU connected" + CEND)
                return 0
            else:
                rospy.logerr(CRED2 + "Error connecting to drone's FCU" + CEND)
                return -1

    def wait4start(self):
        """This function will hold the program until the user signals the FCU to mode enter GUIDED mode. This is typically done from a switch on the safety pilot's remote or from the Ground Control Station.

        Returns:
                0 (int): Mission started successfully.
                -1 (int): Failed to start mission.
        """
        rospy.loginfo(CYELLOW2 + CBLINK +
                      "Waiting for user to set mode to GUIDED" + CEND)
        while not rospy.is_shutdown() and self.current_state_g.mode != "GUIDED":
            rospy.sleep(0.01)
        else:
            if self.current_state_g.mode == "GUIDED":
                rospy.loginfo(
                    CGREEN2 + "Mode set to GUIDED. Starting Mission..." + CEND)
                return 0
            else:
                rospy.logerr(CRED2 + "Error startting mission" + CEND)
                return -1

    def set_mode(self, mode):
        """This function changes the mode of the drone to a user specified mode. This takes the mode as a string. Ex. set_mode("GUIDED").

        Args:
                mode (String): Can be set to modes given in https://ardupilot.org/copter/docs/flight-modes.html

        Returns:
                0 (int): Mode Set successful.
                -1 (int): Mode Set unsuccessful.
        """
        SetMode_srv = SetModeRequest(0, mode)
        response = self.set_mode_client(SetMode_srv)
        if response.mode_sent:
            rospy.loginfo(CGREEN2 + "SetMode Was successful" + CEND)
            return 0
        else:
            rospy.logerr(CRED2 + "SetMode has failed" + CEND)
            return -1

    def set_speed(self, speed_mps):
        """This function is used to change the speed of the vehicle in guided mode. It takes the speed in meters per second as a float as the input.

        Args:
                speed_mps (Float): Speed in m/s.

        Returns:
                0 (int): Speed set successful.
                -1 (int): Speed set unsuccessful.
        """
        speed_cmd = CommandLongRequest()
        speed_cmd.command = 178
        speed_cmd.param1 = 1
        speed_cmd.param2 = speed_mps
        speed_cmd.param3 = -1
        speed_cmd.param4 = 0

        rospy.loginfo(
            CBLUE2 + "Setting speed to {}m/s".format(str(speed_mps)) + CEND)
        response = self.command_client(speed_cmd)

        if response.success:
            rospy.loginfo(
                CGREEN2 + "Speed set successfully with code {}".format(str(response.success)) + CEND)
            rospy.loginfo(
                CGREEN2 + "Change Speed result was {}".format(str(response.result)) + CEND)
            return 0
        else:
            rospy.logerr(
                CRED2 + "Speed set failed with code {}".format(str(response.success)) + CEND)
            rospy.logerr(
                CRED2 + "Speed set result was {}".format(str(response.result)) + CEND)
            return -1

    def set_heading(self, heading):
        """This function is used to specify the drone's heading in the local reference frame. Psi is a counter clockwise rotation following the drone's reference frame defined by the x axis through the right side of the drone with the y axis through the front of the drone.

        Args:
                heading (Float): θ(degree) Heading angle of the drone.
        """
        self.local_desired_heading_g = heading
        heading = heading + self.correction_heading_g + self.local_offset_g

        rospy.loginfo("The desired heading is {}".format(
            self.local_desired_heading_g))

        yaw = radians(heading)
        pitch = 0.0
        roll = 0.0

        cy = cos(yaw * 0.5)
        sy = sin(yaw * 0.5)

        cr = cos(roll * 0.5)
        sr = sin(roll * 0.5)

        cp = cos(pitch * 0.5)
        sp = sin(pitch * 0.5)

        qw = cy * cr * cp + sy * sr * sp
        qx = cy * sr * cp - sy * cr * sp
        qy = cy * cr * sp + sy * sr * cp
        qz = sy * cr * cp - cy * sr * sp

        self.waypoint_g.pose.orientation = Quaternion(qx, qy, qz, qw)

    def set_destination(self, x, y, z, psi):
        """This function is used to command the drone to fly to a waypoint. These waypoints should be specified in the local reference frame. This is typically defined from the location the drone is launched. Psi is counter clockwise rotation following the drone's reference frame defined by the x axis through the right side of the drone with the y axis through the front of the drone.

        Args:
                x (Float): x(m) Distance with respect to your local frame.
                y (Float): y(m) Distance with respect to your local frame.
                z (Float): z(m) Distance with respect to your local frame.
                psi (Float): θ(degree) Heading angle of the drone.
        """
        self.set_heading(psi)

        theta = radians((self.correction_heading_g + self.local_offset_g - 90))

        Xlocal = x * cos(theta) - y * sin(theta)
        Ylocal = x * sin(theta) + y * cos(theta)
        Zlocal = z

        x = Xlocal + self.correction_vector_g.position.x + self.local_offset_pose_g.x

        y = Ylocal + self.correction_vector_g.position.y + self.local_offset_pose_g.y

        z = Zlocal + self.correction_vector_g.position.z + self.local_offset_pose_g.z

        rospy.loginfo(
            "Destination set to x:{} y:{} z:{} origin frame".format(x, y, z))

        self.waypoint_g.pose.position = Point(x, y, z)

        self.local_pos_pub.publish(self.waypoint_g)

    def set_destination_wrt_neighbour(self, x, y, z, psi, safe_distance=1.0):
        """Set the destination, adjusting if necessary to maintain a safe distance from the neighbor.

        Args:
            x (float): The desired x-coordinate.
            y (float): The desired y-coordinate.
            z (float): The desired z-coordinate.
            psi (float): The desired heading.
            safe_distance (float, optional): The safe distance to maintain from the neighbor. Defaults to 1.0.
        """
        # Get the current position of the neighbor drone.
        neighbor_position = self.get_neighbor_position_in_local_frame()

        # Calculate the distance to the neighbor.
        dx = x - neighbor_position.x
        dy = y - neighbor_position.y
        dz = z - neighbor_position.z
        distance_to_neighbour = sqrt(dx**2 + dy**2 + dz**2)

        # If the distance to the neighbor is less than the safe distance, adjust the destination.
        if distance_to_neighbour < safe_distance:
            rospy.loginfo(f"Adjusting destination to maintain safe distance from neighbor ({distance_to_neighbour})")

            # Calculate the direction vector from the drone to the neighbor.
            direction_x = dx / distance_to_neighbour
            direction_y = dy / distance_to_neighbour
            direction_z = dz / distance_to_neighbour

            # Adjust the destination to maintain the safe distance.
            x = neighbor_position.x + direction_x * safe_distance
            y = neighbor_position.y + direction_y * safe_distance
            z = neighbor_position.z + direction_z * safe_distance

        # Call the original set_destination method.
        self.set_destination(x, y, z, psi)

    def arm(self):
        """Arms the drone for takeoff.

        Returns:
                0 (int): Arming successful.
                -1 (int): Arming unsuccessful.
        """
        self.set_destination(0, 0, 0, 0)

        for _ in range(100):
            self.local_pos_pub.publish(self.waypoint_g)
            rospy.sleep(0.01)

        rospy.loginfo(CBLUE2 + "Arming Drone" + CEND)

        arm_request = CommandBoolRequest(True)

        while not rospy.is_shutdown() and not self.current_state_g.armed:
            rospy.sleep(0.1)
            response = self.arming_client(arm_request)
            self.local_pos_pub.publish(self.waypoint_g)
        else:
            if response.success:
                rospy.loginfo(CGREEN2 + "Arming successful" + CEND)
                return 0
            else:
                rospy.logerr(CRED2 + "Arming failed" + CEND)
                return -1

    def takeoff(self, takeoff_alt):
        """The takeoff function will arm the drone and put the drone in a hover above the initial position.

        Args:
                takeoff_alt (Float): The altitude at which the drone should hover.

        Returns:
                0 (int): Takeoff successful.
                -1 (int): Takeoff unsuccessful.
        """
        self.arm()
        takeoff_srv = CommandTOLRequest(0, 0, 0, 0, takeoff_alt)
        response = self.takeoff_client(takeoff_srv)
        rospy.sleep(3)
        if response.success:
            rospy.loginfo(CGREEN2 + "Takeoff successful" + CEND)
            return 0
        else:
            rospy.logerr(CRED2 + "Takeoff failed" + CEND)
            return -1

    def initialize_local_frame(self):
        """This function will create a local reference frame based on the starting location of the drone. This is typically done right before takeoff. This reference frame is what all of the the set destination commands will be in reference to."""
        self.local_offset_g = 0.0

        for i in range(30):
            rospy.sleep(0.1)

            q0, q1, q2, q3 = (
                self.current_pose_g.pose.pose.orientation.w,
                self.current_pose_g.pose.pose.orientation.x,
                self.current_pose_g.pose.pose.orientation.y,
                self.current_pose_g.pose.pose.orientation.z,
            )

            psi = atan2((2 * (q0 * q3 + q1 * q2)),
                        (1 - 2 * (pow(q2, 2) + pow(q3, 2))))

            self.local_offset_g += degrees(psi)
            self.local_offset_pose_g.x += self.current_pose_g.pose.pose.position.x
            self.local_offset_pose_g.y += self.current_pose_g.pose.pose.position.y
            self.local_offset_pose_g.z += self.current_pose_g.pose.pose.position.z

        self.local_offset_pose_g.x /= 30.0
        self.local_offset_pose_g.y /= 30.0
        self.local_offset_pose_g.z /= 30.0
        self.local_offset_g /= 30.0

        rospy.loginfo(CBLUE2 + "Coordinate offset set" + CEND)
        rospy.loginfo(
            CGREEN2 + "The X-Axis is facing: {}".format(self.local_offset_g) + CEND)

    def check_waypoint_reached(self, pos_tol=0.3, head_tol=0.01):
        """This function checks if the waypoint is reached within given tolerance and returns an int of 1 or 0. This function can be used to check when to request the next waypoint in the mission.

        Args:
                pos_tol (float, optional): Position tolerance under which the drone must be with respect to its position in space. Defaults to 0.3.
                head_tol (float, optional): Heading or angle tolerance under which the drone must be with respect to its orientation in space. Defaults to 0.01.

        Returns:
                1 (int): Waypoint reached successfully.
                0 (int): Failed to reach Waypoint.
        """
        self.local_pos_pub.publish(self.waypoint_g)

        dx = abs(
            self.waypoint_g.pose.position.x - self.current_pose_g.pose.pose.position.x
        )
        dy = abs(
            self.waypoint_g.pose.position.y - self.current_pose_g.pose.pose.position.y
        )
        dz = abs(
            self.waypoint_g.pose.position.z - self.current_pose_g.pose.pose.position.z
        )

        dMag = sqrt(pow(dx, 2) + pow(dy, 2) + pow(dz, 2))

        cosErr = cos(radians(self.current_heading_g)) - cos(
            radians(self.local_desired_heading_g)
        )

        sinErr = sin(radians(self.current_heading_g)) - sin(
            radians(self.local_desired_heading_g)
        )

        dHead = sqrt(pow(cosErr, 2) + pow(sinErr, 2))

        if dMag < pos_tol and dHead < head_tol:
            return 1
        else:
            return 0

    def follow_neighbour(self, x, y, z, psi, distance=0.5):
        """Make the drone follow its neighbor by a fixed distance.

        Args:
            distance (float): The desired separation distance between the drone and its neighbor.
        """
        if self.is_leader:
            rospy.loginfo(f"{self.ns} leading the way")
            # Leader drone follows a preset waypoint path
            self.follow_waypoint_path(x, y, z, psi)
        else:
            # Follower drone follows the neighbor by a fixed distance
            neighbor_pos_local = self.get_neighbor_position_in_local_frame()
            current_pos_local = self.get_current_location()

            dx = neighbor_pos_local.x - current_pos_local.x
            dy = neighbor_pos_local.y - current_pos_local.y
            dz = neighbor_pos_local.z - current_pos_local.z

            separation_distance = np.sqrt(dx**2 + dy**2 + dz**2)

            if separation_distance > distance:
                # If the actual separation distance is greater than the desired distance, move closer to the neighbor
                move_distance = abs(distance - separation_distance)
            else:
                move_distance = distance

            direction_vector = np.array([dx, dy, dz]) / separation_distance
            move_vector = direction_vector * move_distance

            new_pos_local = Point(
                current_pos_local.x + move_vector[0],
                current_pos_local.y + move_vector[1],
                current_pos_local.z + move_vector[2]
            )

            # Calculate the heading towards the new position
            heading = np.arctan2(move_vector[1], move_vector[0])

            self.set_destination(new_pos_local.x, new_pos_local.y, new_pos_local.z, heading)
            rospy.loginfo(f"{self.ns} following {self.neighbor_ns}")

    def follow_waypoint_path(self, x, y, z, psi):
        """Follow a preset waypoint path."""
        # Add your code to define and follow the waypoint path for the leader drone
        self.set_destination(x, y, z, psi)
        pass
    