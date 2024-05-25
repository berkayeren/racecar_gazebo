#!/usr/bin/env python3

import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import numpy as np


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

    def odom_callback(self, data):
        # Use odometry information to track the car's movement and adjust its speed and steering angle
        pass

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

    def scan_callback(self, data):
        self.drive_msg.header.stamp = rospy.Time.now()

        # Split the laser scan data into left and right halves
        left_scan = data.ranges[:len(data.ranges) // 2]
        right_scan = data.ranges[len(data.ranges) // 2:]

        # Replace inf values with the maximum range of the sensor
        left_scan = [x if x != float('inf') else 10.0 for x in left_scan]
        right_scan = [x if x != float('inf') else 10.0 for x in right_scan]

        # Calculate the average distance to the left and right side of the car
        left_distance = sum(left_scan) / len(left_scan)
        right_distance = sum(right_scan) / len(right_scan)

        # Define safe distance
        safe_distance = 0.5  # Increase the safe distance

        # Minimum distance from the obstacle
        min_distance = min(data.ranges)

        # Check if the car is in recovery state
        if self.recovery:
            if rospy.Time.now() - self.recovery_start < rospy.Duration(5):  # Extend recovery to 5 seconds
                return  # Ignore the normal logic while in recovery state
            else:
                self.recovery = False  # End the recovery state

        if min_distance < safe_distance:  # Obstacle detected
            self.drive_msg.drive.speed = -0.5  # Move backwards at reduced speed
            # Calculate turning angle based on distance to obstacle
            self.drive_msg.drive.steering_angle = 1.0 - min_distance / safe_distance
            self.recovery = True  # Start the recovery state
            self.recovery_start = rospy.Time.now()  # Record the start time of the recovery state
        else:
            # Path is clear, move forward
            # Adjust speed based on the sigmoid of the minimum distance
            self.drive_msg.drive.speed = 1.0 / (1 + np.exp(-min_distance))
            # Calculate steering angle based on the distances to the left and right sides
            # Add a bias to the steering angle to make the car prefer the left side
            self.drive_msg.drive.steering_angle = (right_distance - left_distance) / (
                        right_distance + left_distance + 0.01) + 0.1

        # Smooth the steering angle
        self.drive_msg.drive.steering_angle = self.smooth_steering_angle(self.drive_msg.drive.steering_angle)

        self.drive_msg.drive.steering_angle_velocity = 2  # Increase the turning speed

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
