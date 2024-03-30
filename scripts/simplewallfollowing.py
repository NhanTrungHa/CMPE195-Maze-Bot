#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math
import time

class SimpleWallFollower:
    def __init__(self, robot_speed, wall_distance, wall_side):
        # Initialize the SimpleWallFollower class with provided parameters
        self.x_positions = []
        self.y_positions = []

        self.robot_speed = robot_speed
        self.wall_distance = wall_distance
        self.wall_lead_distance = 0.5

        # Determine which side to follow based on input parameter
        self.wall_side_factor = 1 if wall_side == 'left' else -1
        self.angular_velocity = 0

        rospy.init_node('SimpleWallFollower')

        self.rate = rospy.Rate(10)

        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1000)
        rospy.Subscriber("scan", LaserScan, self.scan_callback)
        print(f'Following wall on the {wall_side} side...')
        rospy.sleep(1)

    def update_velocity(self, linear_velocity, angular_velocity):
        """Update the robot's velocity."""
        # Update the robot's velocity based on linear and angular velocity values
        msg = Twist()
        msg.linear.x = linear_velocity
        msg.angular.z = angular_velocity
        self.cmd_vel_pub.publish(msg)

    def scan_callback(self, msg):
        # Callback function to handle incoming laser scan data
        max_scan_value = msg.range_max

        # Define regions for laser scan data
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
            'frontwide': min(min(msg.ranges[0:40] + msg.ranges[-40:]), 10)
        }

        y0 = self.scan_regions['E'] if self.wall_side_factor == -1 else self.scan_regions['W']
        x1 = (self.scan_regions['ENE'] if self.wall_side_factor == -1 else self.scan_regions['WNW']) * math.sin(math.radians(23))
        y1 = (self.scan_regions['ENE'] if self.wall_side_factor == -1 else self.scan_regions['WNW']) * math.cos(math.radians(23))

        if y0 >= self.wall_distance * 2 and self.scan_regions['N'] < max_scan_value:
            # If the robot is heading into the wall, turn sideways
            print("Turning sideways")
            self.angular_velocity = -math.pi / 4 * self.wall_side_factor
        else:
            front_scan = min([self.scan_regions['N'], self.scan_regions['NNW'] + (max_scan_value - self.scan_regions['WNW']), self.scan_regions['NNE'] + (max_scan_value - self.scan_regions['ENE'])])
            turn_fix = 0 if front_scan >= 0.5 else 1 - front_scan
            abs_alpha = math.atan2(y1 - self.wall_distance, x1 + self.wall_lead_distance - y0) - turn_fix * 1.5
            self.angular_velocity = self.wall_side_factor * abs_alpha

    def time(self):
        """Function to return current time."""
        return time.time()

    def run(self):
        try:
            maze_completed = False

            # Print parameters for debugging
            print('Robot Speed:', str(self.robot_speed))
            print('Distance to Wall:', str(self.wall_distance))
            print('')

            start_time = self.time()

            # Run the wall following logic until ROS is shutdown
            while not rospy.is_shutdown():
                self.update_velocity(self.robot_speed, self.angular_velocity)
                self.rate.sleep()

            end_time = self.time()

            print("Maze Solved!")
            maze_completed = True

            return [], 0, end_time - start_time, maze_completed

        except KeyboardInterrupt:
            # If Ctrl+C is pressed, print the time taken and exit
            end_time = self.time()
            print("Maze solving interrupted!")
            print("Time Taken:", end_time - start_time)
            return [], 0, end_time - start_time, False

        except rospy.ROSInterruptException:
            pass

if __name__ == "__main__":
    # Initialize the SimpleWallFollower node
    simple_wall_follower = SimpleWallFollower(robot_speed=0.5, wall_distance=1.0, wall_side='left')

    # Run the wall following logic
    path_taken, path_length, time_taken, maze_completed = simple_wall_follower.run()

    # Print results
    print("Path Taken:", path_taken)
    print("Path Length:", path_length)
    print("Time Taken:", time_taken)
    print("Maze Completed:", maze_completed)

