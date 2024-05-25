#!/usr/bin/env python3

import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import numpy as np
import tf


class AutonomousRacecar:
    def __init__(self):
        rospy.init_node('autonomous_racecar', anonymous=True)
        racecar = ''
        # Publisher for Ackermann drive commands
        self.drive_pub = rospy.Publisher(f'/vesc{racecar}/ackermann_cmd_mux/input/teleop', AckermannDriveStamped,
                                         queue_size=10)

        # Subscriber for laser scan data
        self.scan_sub = rospy.Subscriber(f'/scan{racecar}', LaserScan, self.scan_callback)

        # Subscriber for navigation commands
        self.nav_sub = rospy.Subscriber(f'/vesc{racecar}/ackermann_cmd_mux/input/navigation', AckermannDriveStamped,
                                        self.nav_callback)

        # Subscriber for safety commands
        self.safety_sub = rospy.Subscriber(f'/vesc{racecar}/ackermann_cmd_mux/input/safety', AckermannDriveStamped,
                                           self.safety_callback)

        # Subscriber for odometry data
        self.odom_sub = rospy.Subscriber(f'/vesc{racecar}/odom', Odometry, self.odom_callback)

        self.drive_msg = AckermannDriveStamped()
        self.rate = rospy.Rate(10)  # 10 Hz
        self.recovery = False
        self.previous_steering_angles = []
        # Control parameters
        self.speed = 1.0
        self.reverse_speed = -2.0  # Speed for reversing
        self.steering_gain = 0.5  # Gain for steering correction based on the error
        self.max_steering_angle = 0.34  # Maximum steering angle in radians (approximately 20 degrees)
        self.dead_zone = 0.1  # Dead zone to ignore small errors
        self.obstacle_distance_threshold = 1.0  # Threshold distance to consider an obstacle in meters
        self.laser_range_left_index = 60  # Index for left laser scan
        self.laser_range_right_index = 300  # Index for right laser scan
        self.laser_range_front_index = 180  # Index for front laser scan
        self.reverse_distance_threshold = 0.5  # Distance threshold to start reversing
        self.laser_range_front_indices = range(170, 191)  # Indices for front laser scan (average)
        self.br = tf.TransformBroadcaster()

    def scan_callback(self, scan_data):
        # Extract distances from the laser scan data
        left_distance = scan_data.ranges[self.laser_range_left_index]
        right_distance = scan_data.ranges[self.laser_range_right_index]
        front_distances = [scan_data.ranges[i] for i in self.laser_range_front_indices]

        # Filter out invalid measurements (inf)
        front_distances = [d for d in front_distances if d < float('inf')]

        # Calculate the average front distance
        if front_distances:
            front_distance = np.mean(front_distances)
        else:
            front_distance = float('inf')

        # Calculate the error (difference between left and right distances)
        error = right_distance - left_distance

        # Apply dead zone to ignore small errors
        if abs(error) < self.dead_zone:
            error = 0

        # Initialize variables
        steering_angle = 0
        speed = self.speed

        # Obstacle avoidance and reversing logic
        if front_distance < self.reverse_distance_threshold:
            # Reverse and steer away from the obstacle
            speed = self.reverse_speed
            if left_distance > right_distance:
                steering_angle = self.max_steering_angle  # Turn left while reversing
            else:
                steering_angle = -self.max_steering_angle  # Turn right while reversing
        elif front_distance < self.obstacle_distance_threshold:
            # Avoid obstacle by steering
            if left_distance > right_distance:
                steering_angle = self.max_steering_angle  # Turn left to avoid obstacle
            else:
                steering_angle = -self.max_steering_angle  # Turn right to avoid obstacle
        else:
            # Calculate the steering angle based on the error and limit it to max_steering_angle
            steering_angle = -self.steering_gain * error
            steering_angle = max(min(steering_angle, self.max_steering_angle), -self.max_steering_angle)

        # Print statements for debugging
        rospy.loginfo(
            f"Left distance: {left_distance}, Right distance: {right_distance}, Front distance: {front_distance}, Error: {error}, Steering angle: {steering_angle}, Speed: {speed}")

        # Create and publish the drive command
        drive_cmd = AckermannDriveStamped()
        drive_cmd.drive.steering_angle = steering_angle
        drive_cmd.drive.speed = speed
        self.drive_pub.publish(drive_cmd)

    def nav_callback(self, data):
        # Use navigation commands to control the car
        self.drive_msg.drive.speed = data.drive.speed
        self.drive_msg.drive.steering_angle = data.drive.steering_angle
        print(
            f"Received navigation command: speed={self.drive_msg.drive.speed}, steering_angle={self.drive_msg.drive.steering_angle}")

    def safety_callback(self, data):
        # Use safety commands to override other commands for safety reasons
        if data.drive.speed == 0:  # If the safety command is to stop the car
            self.drive_msg.drive.speed = 0
            print("Received safety command: stop the car")

    def odom_callback(self, odom):
        position = odom.pose.pose.position
        orientation = odom.pose.pose.orientation

        # Broadcast the transform
        self.br.sendTransform((position.x, position.y, position.z),
                              (orientation.x, orientation.y, orientation.z, orientation.w),
                              rospy.Time.now(),
                              "first_car",
                              "world")

    def run(self):
        try:
            while not rospy.is_shutdown():
                self.drive_pub.publish(self.drive_msg)
                self.rate.sleep()
                print("Published drive command in main loop")
        finally:
            self.drive_msg.header.stamp = rospy.Time.now()
            # This block will be executed when the script is closed
            print("Stopping the vehicle")
            self.drive_msg.drive.speed = 0  # Stop the vehicle
            self.drive_pub.publish(self.drive_msg)
            print("Published final drive command")

    def scan_callback2(self, data):
        self.drive_msg.header.stamp = rospy.Time.now()
        #
        # # Split the laser scan data into left and right halves
        # left_scan = data.ranges[:len(data.ranges) // 2]
        # right_scan = data.ranges[len(data.ranges) // 2:]
        #
        # # Replace inf values with the maximum range of the sensor
        # left_scan = [x if x != float('inf') else 10.0 for x in left_scan]
        # right_scan = [x if x != float('inf') else 10.0 for x in right_scan]
        #
        # # Calculate the average distance to the left and right side of the car
        # left_distance = sum(left_scan) / len(left_scan)
        # right_distance = sum(right_scan) / len(right_scan)
        #
        # # Define safe distance
        # safe_distance = 0.5  # Increase the safe distance
        #
        # # Minimum distance from the obstacle
        # min_distance = min(data.ranges)
        #
        # # Check if the car is in recovery state
        # if self.recovery:
        #     if rospy.Time.now() - self.recovery_start < rospy.Duration(5):  # Extend recovery to 5 seconds
        #         return  # Ignore the normal logic while in recovery state
        #     else:
        #         self.recovery = False  # End the recovery state
        #
        # if min_distance < safe_distance:  # Obstacle detected
        #     self.drive_msg.drive.speed = -0.5  # Move backwards at reduced speed
        #     # Calculate turning angle based on distance to obstacle
        #     self.drive_msg.drive.steering_angle = 1.0 - min_distance / safe_distance
        #     self.recovery = True  # Start the recovery state
        #     self.recovery_start = rospy.Time.now()  # Record the start time of the recovery state
        # else:
        #     # Path is clear, move forward
        #     # Adjust speed based on the sigmoid of the minimum distance
        #     self.drive_msg.drive.speed = 1.0 / (1 + np.exp(-min_distance))
        #     # Calculate steering angle based on the distances to the left and right sides
        #     # Add a bias to the steering angle to make the car prefer the left side
        #     self.drive_msg.drive.steering_angle = (right_distance - left_distance) / (
        #             right_distance + left_distance + 0.01) + 0.1
        #
        # # Smooth the steering angle
        # self.drive_msg.drive.steering_angle = self.smooth_steering_angle(self.drive_msg.drive.steering_angle)
        #
        # self.drive_msg.drive.steering_angle_velocity = 2  # Increase the turning speed

        # Extract left and right distances from the laser scan data
        left_distance = data.ranges[self.laser_range_left_index]
        right_distance = data.ranges[self.laser_range_right_index]

        # Calculate the error (difference between left and right distances)
        error = right_distance - left_distance

        # Calculate the steering angle based on the error
        steering_angle = -self.steering_gain * error

        self.drive_msg.drive.steering_angle = steering_angle
        self.drive_msg.drive.speed = 0.5
        self.drive_pub.publish(self.drive_msg)

    def smooth_steering_angle(self, current_steering_angle):
        # Implement your smoothing function here
        # For example, a simple moving average:
        self.previous_steering_angles.append(current_steering_angle)
        if len(self.previous_steering_angles) > 5:  # Keep the last 5 steering angles
            self.previous_steering_angles.pop(0)
        return sum(self.previous_steering_angles) / len(self.previous_steering_angles)


if __name__ == '__main__':
    try:
        racecar = AutonomousRacecar()
        racecar.run()
    except rospy.ROSInterruptException:
        pass
