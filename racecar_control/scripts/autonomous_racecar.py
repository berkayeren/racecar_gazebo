#!/usr/bin/env python3

import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry


class AutonomousRacecar:
    def __init__(self):
        rospy.init_node('autonomous_racecar', anonymous=True)

        # Publisher for Ackermann drive commands
        self.drive_pub = rospy.Publisher('/vesc2/ackermann_cmd_mux/input/teleop', AckermannDriveStamped, queue_size=10)

        # Subscriber for laser scan data
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)

        # Publisher for odometry data
        self.odom_pub = rospy.Publisher('/vesc2/odom', Odometry, queue_size=1)

        self.drive_msg = AckermannDriveStamped()
        self.rate = rospy.Rate(10)  # 10 Hz

    def scan_callback(self, data):
        print("Scan callback invoked")

        # Minimum distance from the obstacle
        min_distance = min(data.ranges)
        print(f"Minimum distance from obstacle: {min_distance}")

        # Define safe distance
        safe_distance = 0.5  # meters

        if min_distance < safe_distance:
            # Obstacle detected, turn the car
            print("Obstacle detected, turning")
            self.drive_msg.drive.speed = 0.0
            self.drive_msg.drive.steering_angle = 0.5  # Turn
        else:
            # Path is clear, move forward
            print("Path clear, moving forward")
            self.drive_msg.drive.speed = 1.0
            self.drive_msg.drive.steering_angle = 0.0  # Go straight

        print(
            f"Publishing drive command: speed={self.drive_msg.drive.speed}, steering_angle={self.drive_msg.drive.steering_angle}")
        self.drive_pub.publish(self.drive_msg)
        print("Published drive command")

    def run(self):
        while not rospy.is_shutdown():
            self.drive_pub.publish(self.drive_msg)
            self.rate.sleep()
            print("Published drive command in main loop")


if __name__ == '__main__':
    try:
        racecar = AutonomousRacecar()
        racecar.run()
    except rospy.ROSInterruptException:
        pass
