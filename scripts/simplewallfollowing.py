#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math
import time

class SimpleWallFollower:
    ANGLE_DEGREE = 23
    SAFETY_DISTANCE_MULTIPLIER = 2
    FRONT_SCAN_THRESHOLD = 0.5
    MAX_FRONT_RANGE = 10

    def __init__(self, speed, wall_distance, wall_side):
        self.speed = speed
        self.wall_distance = wall_distance
        self.wall_side_factor = 1 if wall_side == 'left' else -1
        self.angular_velocity = 0

        rospy.init_node('SimpleWallFollower', anonymous=True)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber("scan", LaserScan, self.scan_callback)
        
        rospy.loginfo(f'Following wall on the {wall_side} side...')
        self.rate = rospy.Rate(10)

    def update_velocity(self, linear_velocity, angular_velocity):
        msg = Twist()
        msg.linear.x = linear_velocity
        msg.angular.z = angular_velocity
        self.cmd_vel_pub.publish(msg)

    def scan_callback(self, msg):
        self.update_scan_regions(msg)
        self.calculate_wall_following_parameters(msg.range_max)

    def update_scan_regions(self, msg):
        max_scan_value = msg.range_max
        self.scan_regions = {
            'N': min(min(msg.ranges[0:5] + msg.ranges[-5:]), max_scan_value),
            'NNW': min(min(msg.ranges[11:20]), max_scan_value),
            'NW': min(min(msg.ranges[41:50]), max_scan_value),
            'WNW': min(min(msg.ranges[64:73]), max_scan_value),
            'W': min(min(msg.ranges[86:95]), max_scan_value),
            'E': min(min(msg.ranges[266:275]), max_scan_value),
            'ENE': min(min(msg.ranges[289:298]), max_scan_value),
            'NE': min(min(msg.ranges[311:320]), max_scan_value),
            'NNE': min(min(msg.ranges[341:350]), max_scan_value),
            'frontwide': min(min(msg.ranges[0:40] + msg.ranges[-40:]), self.MAX_FRONT_RANGE)
        }

    def calculate_wall_following_parameters(self, max_scan_value):
        current_wall_distance = self.scan_regions['E'] if self.wall_side_factor == -1 else self.scan_regions['W']
        projected_x_position = self.scan_regions['ENE'] if self.wall_side_factor == -1 else self.scan_regions['WNW']
        projected_y_position = projected_x_position * math.sin(math.radians(self.ANGLE_DEGREE))
        
        if self.scan_regions['N'] < max_scan_value * self.SAFETY_DISTANCE_MULTIPLIER and current_wall_distance > self.wall_distance:
            rospy.loginfo("Turning sideways to avoid wall")
            self.angular_velocity = -math.pi / 4 * self.wall_side_factor
        else:
            front_scan = min([self.scan_regions['N'], self.scan_regions['NNW'], self.scan_regions['NNE']])
            self.angular_velocity = -self.wall_side_factor * math.atan2(front_scan - self.wall_distance, projected_y_position)

    def run(self):
        try:
            start_time = time.time()
            while not rospy.is_shutdown():
                self.update_velocity(self.robot_speed, self.angular_velocity)
                self.rate.sleep()

            rospy.loginfo("Maze Solved!")
            return time.time() - start_time

        except rospy.ROSInterruptException:
            rospy.loginfo("Maze solving interrupted by ROS.")
        except KeyboardInterrupt:
            rospy.loginfo("Maze solving interrupted by user.")
        finally:
            end_time = time.time()
            rospy.loginfo(f"Time Taken: {end_time - start_time}")

if __name__ == "__main__":
    follower = SimpleWallFollower(robot_speed=0.2, wall_distance=1.0, wall_side='left')
    duration = follower.run()
    rospy.loginfo(f"Total Duration: {duration}")
