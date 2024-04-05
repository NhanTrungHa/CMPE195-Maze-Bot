#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class SimpleWallFollower:
    def __init__(self):
        # Robot speed parameters
        self.linear_speed = 0.2
        self.angular_speed = 0.5
        self.target_distance_from_wall = 0.5

        # Initialize ROS node
        rospy.init_node('simple_wall_follower', anonymous=True)
        
        # Publisher to send velocity commands
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        # Subscriber to laser scan topic
        rospy.Subscriber("/scan", LaserScan, self.scan_callback)

        # Run rate
        self.rate = rospy.Rate(10)  # 10hz

    def scan_callback(self, data):
        # Simplified scan processing
        # Assuming 360 degree lidar, and interested in right side scans
        right_side_scans = data.ranges[:30] + data.ranges[-30:]
        if not right_side_scans:
            return
        
        # Calculate the minimum distance to the right
        min_distance = min(right_side_scans)
        
        # Control logic to maintain the distance to the right wall
        error = self.target_distance_from_wall - min_distance
        
        # Prepare Twist message
        move_cmd = Twist()
        
        if abs(error) > 0.1:  # Only correct if error is significant
            move_cmd.linear.x = self.linear_speed
            move_cmd.angular.z = -self.angular_speed if error < 0 else self.angular_speed
        else:
            # Move straight ahead if we're at the target distance
            move_cmd.linear.x = self.linear_speed
            move_cmd.angular.z = 0
        
        # Publish the move command
        self.vel_pub.publish(move_cmd)

    def run(self):
        # Main loop
        try:
            while not rospy.is_shutdown():
                # The logic is handled in the callback
                self.rate.sleep()
        except rospy.ROSInterruptException:
            pass

if __name__ == '__main__':
    wall_follower = SimpleWallFollower()
    wall_follower.run()

