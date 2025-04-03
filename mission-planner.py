#!/usr/bin/env python3

import rospy
import mavros
from mavros_msgs.msg import State, WaypointList, Waypoint, WaypointReached
from mavros_msgs.srv import SetMode, CommandBool, WaypointPush
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix
import math
import time
import numpy as np


class DroneMissionExecutor:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('drone_mission_executor', anonymous=True)

        # Initialize state variables
        self.current_state = None
        self.current_position = None
        self.current_gps = None
        self.waypoints_received = False
        self.mission_started = False
        self.current_waypoint_index = 0

        # Create subscribers
        self.state_sub = rospy.Subscriber('/mavros/state', State, self.state_callback)
        self.position_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.position_callback)
        self.gps_sub = rospy.Subscriber('/mavros/global_position/global', NavSatFix, self.gps_callback)
        self.waypoint_sub = rospy.Subscriber('/mavros/mission/waypoints', WaypointList, self.waypoint_callback)
        self.waypoint_reached_sub = rospy.Subscriber('/mavros/mission/reached', WaypointReached,
                                                     self.waypoint_reached_callback)

        # Create service clients
        rospy.wait_for_service('/mavros/set_mode')
        rospy.wait_for_service('/mavros/cmd/arming')
        rospy.wait_for_service('/mavros/mission/push')

        self.set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self.arming_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.waypoint_push_client = rospy.ServiceProxy('/mavros/mission/push', WaypointPush)

        # Create publishers
        self.local_position_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)

        # Define example mission waypoints (home + 4 waypoints in a square pattern)
        self.create_square_mission()

        rospy.loginfo("Drone mission executor initialized")

    def state_callback(self, msg):
        self.current_state = msg

    def position_callback(self, msg):
        self.current_position = msg

    def gps_callback(self, msg):
        self.current_gps = msg

    def waypoint_callback(self, msg):
        rospy.loginfo(f"Received {len(msg.waypoints)} waypoints")
        self.waypoints_received = True

    def waypoint_reached_callback(self, msg):
        rospy.loginfo(f"Reached waypoint {msg.wp_seq}")
        self.current_waypoint_index = msg.wp_seq

    def create_square_mission(self):
        # Create a mission with waypoints in a square pattern
        # This is just an example - typically you would load these from QGroundControl

        # Create waypoint list
        self.waypoints = []

        # Waypoint 0: Home position
        wp = Waypoint()
        wp.frame = Waypoint.FRAME_GLOBAL_REL_ALT
        wp.command = 16  # MAV_CMD_NAV_WAYPOINT
        wp.is_current = True
        wp.autocontinue = True
        wp.param1 = 0  # Hold time in seconds
        wp.param2 = 0  # Acceptance radius in meters
        wp.param3 = 0  # Pass through waypoint
        wp.param4 = float('nan')  # Desired yaw angle
        wp.x_lat = 0  # Will be set to current position when taking off
        wp.y_long = 0  # Will be set to current position when taking off
        wp.z_alt = 10  # Target altitude (10 meters above home)
        self.waypoints.append(wp)

        # Waypoint 1: North
        wp1 = Waypoint()
        wp1.frame = Waypoint.FRAME_GLOBAL_REL_ALT
        wp1.command = 16  # MAV_CMD_NAV_WAYPOINT
        wp1.is_current = False
        wp1.autocontinue = True
        wp1.param1 = 5  # Hold time in seconds
        wp1.param2 = 2  # Acceptance radius in meters
        wp1.param3 = 0  # Pass through waypoint
        wp1.param4 = float('nan')  # Desired yaw angle
        wp1.x_lat = 0.0001  # Will be offset from home
        wp1.y_long = 0
        wp1.z_alt = 10
        self.waypoints.append(wp1)

        # Waypoint 2: Northeast
        wp2 = Waypoint()
        wp2.frame = Waypoint.FRAME_GLOBAL_REL_ALT
        wp2.command = 16  # MAV_CMD_NAV_WAYPOINT
        wp2.is_current = False
        wp2.autocontinue = True
        wp2.param1 = 5  # Hold time in seconds
        wp2.param2 = 2  # Acceptance radius in meters
        wp2.param3 = 0  # Pass through waypoint
        wp2.param4 = float('nan')  # Desired yaw angle
        wp2.x_lat = 0.0001
        wp2.y_long = 0.0001
        wp2.z_alt = 15  # Different altitude
        self.waypoints.append(wp2)

        # Waypoint 3: East
        wp3 = Waypoint()
        wp3.frame = Waypoint.FRAME_GLOBAL_REL_ALT
        wp3.command = 16  # MAV_CMD_NAV_WAYPOINT
        wp3.is_current = False
        wp3.autocontinue = True
        wp3.param1 = 5  # Hold time in seconds
        wp3.param2 = 2  # Acceptance radius in meters
        wp3.param3 = 0  # Pass through waypoint
        wp3.param4 = float('nan')  # Desired yaw angle
        wp3.x_lat = 0
        wp3.y_long = 0.0001
        wp3.z_alt = 10
        self.waypoints.append(wp3)

        # Waypoint 4: Return to Home
        wp4 = Waypoint()
        wp4.frame = Waypoint.FRAME_GLOBAL_REL_ALT
        wp4.command = 16  # MAV_CMD_NAV_WAYPOINT
        wp4.is_current = False
        wp4.autocontinue = True
        wp4.param1 = 5  # Hold time in seconds
        wp4.param2 = 2  # Acceptance radius in meters
        wp4.param3 = 0  # Pass through waypoint
        wp4.param4 = float('nan')  # Desired yaw angle
        wp4.x_lat = 0
        wp4.y_long = 0
        wp4.z_alt = 10
        self.waypoints.append(wp4)

        # Waypoint 5: Land
        wp5 = Waypoint()
        wp5.frame = Waypoint.FRAME_GLOBAL_REL_ALT
        wp5.command = 21  # MAV_CMD_NAV_LAND
        wp5.is_current = False
        wp5.autocontinue = True
        wp5.param1 = 0
        wp5.param2 = 0
        wp5.param3 = 0
        wp5.param4 = float('nan')  # Desired yaw angle
        wp5.x_lat = 0
        wp5.y_long = 0
        wp5.z_alt = 0
        self.waypoints.append(wp5)

    def update_mission_with_current_position(self):
        """Update the mission waypoints with the current position as home"""
        if self.current_gps is None:
            rospy.logwarn("Cannot update mission: No GPS data available")
            return False

        home_lat = self.current_gps.latitude
        home_lon = self.current_gps.longitude

        # Update waypoint 0 (home)
        self.waypoints[0].x_lat = home_lat
        self.waypoints[0].y_long = home_lon

        # Update all other waypoints relative to home
        for i in range(1, len(self.waypoints)):
            if self.waypoints[i].command != 21:  # If not a land command
                self.waypoints[i].x_lat = home_lat + self.waypoints[i].x_lat
                self.waypoints[i].y_long = home_lon + self.waypoints[i].y_long

        return True

    def upload_mission(self):
        """Upload the mission to the vehicle"""
        if not self.update_mission_with_current_position():
            return False

        try:
            response = self.waypoint_push_client(start_index=0, waypoints=self.waypoints)
            rospy.loginfo(f"Mission uploaded: {response.success}")
            return response.success
        except rospy.ServiceException as e:
            rospy.logerr(f"Mission upload failed: {e}")
            return False

    def arm_and_takeoff(self):
        """Arm and switch to GUIDED mode"""
        if self.current_state is None:
            rospy.logwarn("Cannot arm: No state information")
            return False

        # Set mode to GUIDED
        response = self.set_mode_client(custom_mode="GUIDED")
        if not response.mode_sent:
            rospy.logwarn("Failed to set GUIDED mode")
            return False

        # Wait for mode change
        timeout = rospy.Time.now() + rospy.Duration(5)
        while self.current_state.mode != "GUIDED" and rospy.Time.now() < timeout:
            rospy.sleep(0.1)

        if self.current_state.mode != "GUIDED":
            rospy.logwarn("Vehicle did not enter GUIDED mode")
            return False

        # Arm the vehicle
        response = self.arming_client(True)
        if not response.success:
            rospy.logwarn("Failed to arm")
            return False

        # Wait for arming
        timeout = rospy.Time.now() + rospy.Duration(5)
        while not self.current_state.armed and rospy.Time.now() < timeout:
            rospy.sleep(0.1)

        if not self.current_state.armed:
            rospy.logwarn("Vehicle did not arm")
            return False

        rospy.loginfo("Vehicle armed in GUIDED mode")
        return True

    def start_mission(self):
        """Start the mission in AUTO mode"""
        if not self.waypoints_received:
            rospy.logwarn("Cannot start mission: No waypoints received")
            return False

        # Set mode to AUTO
        response = self.set_mode_client(custom_mode="AUTO")
        if not response.mode_sent:
            rospy.logwarn("Failed to set AUTO mode")
            return False

        # Wait for mode change
        timeout = rospy.Time.now() + rospy.Duration(5)
        while self.current_state.mode != "AUTO" and rospy.Time.now() < timeout:
            rospy.sleep(0.1)

        if self.current_state.mode != "AUTO":
            rospy.logwarn("Vehicle did not enter AUTO mode")
            return False

        self.mission_started = True
        rospy.loginfo("Mission started in AUTO mode")
        return True

    def execute_mission(self):
        """Execute the complete mission workflow"""
        rate = rospy.Rate(10)  # 10 Hz

        # Wait for system to be connected
        rospy.loginfo("Waiting for vehicle connection...")
        while not rospy.is_shutdown() and (self.current_state is None or not self.current_state.connected):
            rate.sleep()

        rospy.loginfo("Vehicle connected")

        # Wait for GPS fix
        rospy.loginfo("Waiting for GPS fix...")
        while not rospy.is_shutdown() and (self.current_gps is None):
            rate.sleep()

        rospy.loginfo("GPS fix obtained")

        # Upload mission
        rospy.loginfo("Uploading mission...")
        if not self.upload_mission():
            rospy.logerr("Failed to upload mission")
            return

        rospy.loginfo("Mission uploaded successfully")

        # Arm and takeoff
        rospy.loginfo("Arming vehicle...")
        if not self.arm_and_takeoff():
            rospy.logerr("Failed to arm vehicle")
            return

        rospy.loginfo("Vehicle armed")

        # Start mission
        rospy.loginfo("Starting mission...")
        if not self.start_mission():
            rospy.logerr("Failed to start mission")
            return

        rospy.loginfo("Mission started")

        # Monitor mission progress
        rospy.loginfo("Monitoring mission progress...")

        while not rospy.is_shutdown() and self.mission_started:
            if self.current_state.mode != "AUTO":
                rospy.logwarn(f"Vehicle not in AUTO mode: {self.current_state.mode}")
                break

            if not self.current_state.armed:
                rospy.loginfo("Mission complete: Vehicle disarmed")
                break

            # Log mission progress
            if self.current_waypoint_index == len(self.waypoints) - 1:
                rospy.loginfo("Executing final waypoint (landing)")

            # Log current position
            if self.current_gps:
                rospy.loginfo(
                    f"Current position: Lat: {self.current_gps.latitude:.7f}, Lon: {self.current_gps.longitude:.7f}, Alt: {self.current_gps.altitude:.2f}")

            rate.sleep()

        rospy.loginfo("Mission execution complete")


if __name__ == '__main__':
    try:
        executor = DroneMissionExecutor()
        executor.execute_mission()
    except rospy.ROSInterruptException:
        pass