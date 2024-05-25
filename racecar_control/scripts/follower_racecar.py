#!/usr/bin/env python3
import rospy
import tf
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Twist
import tf2_ros
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import numpy as np


class FollowFirstCar:
    def __init__(self):
        rospy.init_node('follow_first_car', anonymous=True)
        self.drive_pub = rospy.Publisher('/vesc2/ackermann_cmd_mux/input/teleop', AckermannDriveStamped, queue_size=10)
        self.listener = tf.TransformListener()
        self.rate = rospy.Rate(10)  # 10 Hz

        # Control parameters
        self.speed = 1.0
        self.max_steering_angle = 0.34  # Maximum steering angle in radians (approximately 20 degrees)
        self.follow_distance = 1.0  # Desired following distance in meters
        self.steering_gain = 0.5  # Gain for steering correction based on the error
        # Subscriber for laser scan data
        self.scan_sub = rospy.Subscriber(f'/scan2', LaserScan, self.scan_callback)

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
        self.laser_range_left_index = 60  # Index for left laser scan
        self.laser_range_right_index = 300  # Index for right laser scan
        self.laser_range_front_index = 180  # Index for front laser scan
        self.reverse_distance_threshold = 0.5  # Distance threshold to start reversing

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
        self.drive_msg.drive.steering_angle = steering_angle
        self.drive_msg.drive.speed = speed
        self.drive_msg.header.stamp = rospy.Time.now()

        self.drive_pub.publish(self.drive_msg)

    def run(self):
        while not rospy.is_shutdown():
            try:
                now = rospy.Time.now()
                self.listener.waitForTransform("/world", "/first_car", now, rospy.Duration(1.0))
                (trans, rot) = self.listener.lookupTransform("/world", "/first_car", now)

                # Calculate the distance to the first car
                distance = trans[0]

                # Calculate the error (difference from the desired following distance)
                distance_error = distance - self.follow_distance

                # Adjust speed based on the distance to the first car
                if distance > self.follow_distance:
                    speed = self.speed
                else:
                    speed = 0.5  # Slow down if too close

                # Use the steering gain to keep the car centered
                steering_angle = -self.steering_gain * distance_error
                steering_angle = max(min(steering_angle, self.max_steering_angle), -self.max_steering_angle)

                # Print statements for debugging
                rospy.loginfo(
                    f"Distance to first car: {distance}, Distance error: {distance_error}, Steering angle: {steering_angle}, Speed: {speed}")

                self.drive_msg.drive.steering_angle = steering_angle
                self.drive_msg.drive.speed = speed
                self.drive_pub.publish(self.drive_msg)

            except (
                    tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException,
                    tf2_ros.TransformException):
                continue

            self.rate.sleep()


if __name__ == '__main__':
    try:
        follower = FollowFirstCar()
        follower.run()
    except rospy.ROSInterruptException:
        pass
