#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class SimpleWallFollower:
    def __init__(self):
        # Robot speed parameters
        self.linear_speed = 0.2
        self.angular_speed = 0.5
        self.target_distance_from_wall = 0.5  # Desired distance from the wall

        # Initialize ROS node
        rospy.init_node('simple_wall_follower', anonymous=True)

        # Publisher to send velocity commands
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        # Subscriber to laser scan topic
        rospy.Subscriber("scan", LaserScan, self.scan_callback)

    def scan_callback(self, msg):
        # Extract the distances at 0 degrees and 90 degrees
        front_distance = msg.ranges[0]
        right_distance = msg.ranges[90]

        # Initialize the Twist message
        move_cmd = Twist()

        # Adjust linear speed based on front distance
        if front_distance < 1.0:  # Too close to an obstacle in front
            move_cmd.linear.x = 0.0  # Stop moving forward
        else:
            move_cmd.linear.x = self.linear_speed

        # Adjust angular velocity based on right distance
        if right_distance > self.target_distance_from_wall:
            # Too far from the wall, turn towards the wall
            move_cmd.angular.z = -self.angular_speed
        elif right_distance < self.target_distance_from_wall:
            # Too close to the wall, turn away from the wall
            move_cmd.angular.z = self.angular_speed
        else:
            move_cmd.angular.z = 0.0  # Keep moving straight

        # Publish the move command
        self.vel_pub.publish(move_cmd)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        wall_follower = SimpleWallFollower()
        wall_follower.run()
    except rospy.ROSInterruptException:
        pass

