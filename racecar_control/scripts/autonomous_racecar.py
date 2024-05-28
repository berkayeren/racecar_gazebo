#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import tf
import numpy as np
import math


class AutonomousRacecar:
    def __init__(self):
        rospy.init_node('autonomous_racecar', anonymous=True)
        self.drive_pub = rospy.Publisher('/vesc/ackermann_cmd_mux/input/teleop', AckermannDriveStamped, queue_size=10)
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        rospy.Subscriber('/vesc/odom', Odometry, self.odom_callback)
        self.rate = rospy.Rate(10)  # 10 Hz

        # Control parameters
        self.speed = 1.0
        self.reverse_speed = -1.0
        self.steering_gain = 0.5  # Gain for steering correction based on the error
        self.max_steering_angle = 0.34  # Maximum steering angle in radians (approximately 20 degrees)
        self.dead_zone = 0.1  # Dead zone to ignore small errors
        self.obstacle_distance_threshold = 0.75  # Threshold distance to consider an obstacle in meters
        self.reverse_distance_threshold = 1.0  # Distance threshold to start reversing
        self.reverse_time = 2.0  # Time to reverse when stuck (seconds)
        self.avoid_time = 1.0  # Time to avoid obstacle (seconds)
        self.stuck_time_threshold = 3.0  # Time threshold to detect being stuck (seconds)
        self.recovery_time = 2.0  # Time for recovery maneuver (seconds)
        self.stuck_speed_threshold = 0.1  # Speed threshold to consider the car stuck

        # Orientation variables
        self.current_yaw = 0.0
        self.last_yaw = 0.0

        # Position variables
        self.last_position = None
        self.position_stuck_threshold = 0.1  # Threshold to detect significant position change

        # Velocity variable
        self.current_speed = 0.0

        # State variables
        self.state = 'DRIVING'
        self.state_start_time = rospy.get_time()
        self.stuck_start_time = None
        # TF broadcaster
        self.br = tf.TransformBroadcaster()

    def odom_callback(self, odom):
        orientation_q = odom.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        _, _, yaw = euler_from_quaternion(orientation_list)
        self.current_yaw = yaw

        # Broadcast the transformation
        position = odom.pose.pose.position
        self.br.sendTransform(
            (position.x, position.y, 0),
            (orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w),
            rospy.Time.now(),
            "leader_car",
            "world"
        )

        # Track position to detect if stuck
        if self.last_position is None:
            self.last_position = position
        else:
            distance_moved = np.sqrt(
                (position.x - self.last_position.x) ** 2 + (position.y - self.last_position.y) ** 2)
            if distance_moved < self.position_stuck_threshold:
                if self.stuck_start_time is None:
                    self.stuck_start_time = rospy.get_time()
            else:
                self.stuck_start_time = None
            self.last_position = position

        # Calculate current speed
        self.current_speed = np.sqrt(odom.twist.twist.linear.x ** 2 + odom.twist.twist.linear.y ** 2)

    def scan_callback(self, scan_data):
        # Divide the laser scan data into regions
        regions = {
            "right": min(min(scan_data.ranges[0:143]), 10),
            "front_right": min(min(scan_data.ranges[144:287]), 10),
            "front": min(min(scan_data.ranges[288:431]), 10),
            "front_left": min(min(scan_data.ranges[432:575]), 10),
            "left": min(min(scan_data.ranges[576:719]), 10),
        }

        # Print out the distances for debugging
        rospy.loginfo(f"Regions: {regions}")

        # Calculate the error (difference between left and right distances)
        left_distance = regions['left']
        right_distance = regions['right']
        front_distance = regions['front']
        error = right_distance - left_distance

        # Apply dead zone to ignore small errors
        if abs(error) < self.dead_zone:
            error = 0

        # Control the vehicle based on the scan data
        self.control_vehicle(left_distance, right_distance, front_distance, error)

    def control_vehicle(self, left_distance, right_distance, front_distance, error):
        steering_angle = 0
        speed = self.speed

        # Simplified state management
        current_time = rospy.get_time()

        if self.state == 'DRIVING':
            if not self.can_clear_obstacle(front_distance, error):
                self.state = 'REVERSING'
                self.state_start_time = current_time
            else:
                if front_distance < self.reverse_distance_threshold:
                    self.state = 'REVERSING'
                    self.state_start_time = current_time
                elif front_distance < self.obstacle_distance_threshold:
                    self.state = 'AVOIDING'
                    self.state_start_time = current_time
                else:
                    steering_angle = -self.steering_gain * error
                    steering_angle = max(min(steering_angle, self.max_steering_angle), -self.max_steering_angle)
                    speed = self.speed

            # Check for being stuck
            if (self.stuck_start_time and current_time - self.stuck_start_time > self.stuck_time_threshold and
                    self.current_speed < self.stuck_speed_threshold):
                self.state = 'STUCK'
                self.state_start_time = current_time

        elif self.state == 'AVOIDING':
            if current_time - self.state_start_time < self.avoid_time:
                if left_distance > right_distance:
                    steering_angle = self.max_steering_angle  # Turn left to avoid obstacle
                else:
                    steering_angle = -self.max_steering_angle  # Turn right to avoid obstacle
                speed = self.speed * 0.5  # Slow down while avoiding
            else:
                self.state = 'DRIVING'
                self.stuck_start_time = None

        elif self.state == 'REVERSING':
            if self.can_clear_obstacle(front_distance, error):
                self.state = 'DRIVING'
                self.stuck_start_time = None
                steering_angle = -self.steering_gain * error
                steering_angle = max(min(steering_angle, self.max_steering_angle), -self.max_steering_angle)
                speed = self.speed
            else:
                if current_time - self.state_start_time < self.reverse_time:
                    speed = self.reverse_speed
                    if left_distance > right_distance:
                        steering_angle = self.max_steering_angle  # Turn left while reversing
                    else:
                        steering_angle = -self.max_steering_angle  # Turn right while reversing
                else:
                    self.state = 'STUCK'
                    self.state_start_time = current_time

        elif self.state == 'STUCK':
            if current_time - self.state_start_time < self.recovery_time:
                speed = self.reverse_speed
                steering_angle = self.max_steering_angle  # Turn while reversing to get unstuck
            else:
                self.state = 'DRIVING'
                self.stuck_start_time = None

        # Print statements for debugging
        rospy.loginfo(
            f"State: {self.state}, Left distance: {left_distance}, Right distance: {right_distance}, Front distance: {front_distance}, Error: {error}, Steering angle: {steering_angle}, Speed: {speed}, Yaw: {self.current_yaw}")

        # Create and publish the drive command
        drive_cmd = AckermannDriveStamped()
        drive_cmd.drive.steering_angle = steering_angle
        drive_cmd.drive.speed = speed
        self.drive_pub.publish(drive_cmd)

    def can_clear_obstacle(self, front_distance, error):
        # Calculate the predicted position of the car
        speed = self.speed
        steering_angle = -self.steering_gain * error
        steering_angle = max(min(steering_angle, self.max_steering_angle), -self.max_steering_angle)

        # Predict the new position based on the current speed and steering angle
        predicted_x = front_distance + speed * math.cos(steering_angle)
        predicted_y = speed * math.sin(steering_angle)

        # Calculate the new front distance
        new_front_distance = np.sqrt(predicted_x ** 2 + predicted_y ** 2)

        # Determine if the car can clear the obstacle
        return new_front_distance > self.obstacle_distance_threshold

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        car = AutonomousRacecar()
        car.run()
    except rospy.ROSInterruptException:
        pass
