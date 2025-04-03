#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, NavSatFix
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import os
from datetime import datetime
import csv


class DroneDataProcessor:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('drone_data_processor', anonymous=True)

        # Create a CV bridge
        self.bridge = CvBridge()

        # Create directory to store data
        self.data_dir = os.path.expanduser("~/drone_data")
        self.session_dir = os.path.join(self.data_dir, datetime.now().strftime("%Y%m%d_%H%M%S"))
        self.images_dir = os.path.join(self.session_dir, "images")

        # Create directories if they don't exist
        for directory in [self.data_dir, self.session_dir, self.images_dir]:
            if not os.path.exists(directory):
                os.makedirs(directory)

        # Create CSV file for telemetry data
        self.telemetry_file = os.path.join(self.session_dir, "telemetry.csv")
        with open(self.telemetry_file, 'w') as f:
            writer = csv.writer(f)
            writer.writerow(['timestamp', 'image_filename', 'latitude', 'longitude', 'altitude', 'x', 'y', 'z'])

        # State variables
        self.latest_gps = None
        self.latest_pose = None
        self.frame_count = 0
        self.save_interval = 10  # Save every 10th frame

        # Subscribe to the camera topic
        self.image_sub = rospy.Subscriber('/iris/camera/image_raw', Image, self.image_callback)

        # Subscribe to GPS and position topics
        self.gps_sub = rospy.Subscriber('/mavros/global_position/global', NavSatFix, self.gps_callback)
        self.pos_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.pose_callback)

        # Create display window
        cv2.namedWindow('Drone Camera', cv2.WINDOW_NORMAL)
        cv2.namedWindow('Processed Image', cv2.WINDOW_NORMAL)

        rospy.loginfo(f"Drone data processor started. Saving data to: {self.session_dir}")
        rospy.loginfo("Press 'q' to quit, 's' to save the current frame")

    def image_callback(self, msg):
        try:
            # Convert ROS Image to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # Process the image
            processed_image = self.process_image(cv_image)

            # Display the images
            cv2.imshow('Drone Camera', cv_image)
            cv2.imshow('Processed Image', processed_image)

            # Save images periodically
            self.frame_count += 1
            if self.frame_count % self.save_interval == 0:
                self.save_data(cv_image, processed_image)

            # Handle keyboard input
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                rospy.signal_shutdown("User requested shutdown")
            elif key == ord('s'):
                self.save_data(cv_image, processed_image, force=True)

        except Exception as e:
            rospy.logerr(f"Error processing image: {e}")

    def process_image(self, image):
        """
        Process the image with computer vision techniques
        """
        # Convert to grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Apply Gaussian blur
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)

        # Apply Canny edge detection
        edges = cv2.Canny(blurred, 50, 150)

        # Find contours
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Create a copy of the original image for drawing
        result = image.copy()

        # Draw contours on the result image
        cv2.drawContours(result, contours, -1, (0, 255, 0), 2)

        # Add text with metrics
        cv2.putText(result, f"Objects detected: {len(contours)}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        # Add GPS coordinates if available
        if self.latest_gps:
            gps_text = f"GPS: {self.latest_gps.latitude:.6f}, {self.latest_gps.longitude:.6f}"
            cv2.putText(result, gps_text, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        return result

    def gps_callback(self, msg):
        self.latest_gps = msg

    def pose_callback(self, msg):
        self.latest_pose = msg

    def save_data(self, original, processed, force=False):
        """
        Save the original and processed images along with telemetry data
        """
        timestamp = rospy.get_time()

        # Create filenames
        original_filename = f"frame_{timestamp:.2f}_original.jpg"
        processed_filename = f"frame_{timestamp:.2f}_processed.jpg"

        # Save images
        cv2.imwrite(os.path.join(self.images_dir, original_filename), original)
        cv2.imwrite(os.path.join(self.images_dir, processed_filename), processed)

        # Log telemetry data
        if self.latest_gps and self.latest_pose:
            with open(self.telemetry_file, 'a') as f:
                writer = csv.writer(f)
                writer.writerow([
                    timestamp,
                    original_filename,
                    self.latest_gps.latitude,
                    self.latest_gps.longitude,
                    self.latest_gps.altitude,
                    self.latest_pose.pose.position.x,
                    self.latest_pose.pose.position.y,
                    self.latest_pose.pose.position.z
                ])

        if force:
            rospy.loginfo(f"Manually saved frame at timestamp {timestamp:.2f}")
        else:
            rospy.loginfo(f"Auto-saved frame at timestamp {timestamp:.2f}")

    def run(self):
        rospy.spin()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    try:
        processor = DroneDataProcessor()
        processor.run()
    except rospy.ROSInterruptException:
        pass