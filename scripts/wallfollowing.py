#!/usr/bin/env python3


import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math, time

class WallFollower:
    def __init__(self, speed, distance_wall, side):
        self.x_history = []
        self.y_history = []

        self.speed = speed
        self.distance_wall = distance_wall
        self.wall_lead = 0.5

        self.side = 1 if side == 'left' else -1
        self.g_alpha = 0

        rospy.init_node('PythonControl')

        self.rate = rospy.Rate(10)
       
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1000)
        rospy.Subscriber("scan", LaserScan, self.scan_callback)
        print(f'Follow wall from the {side}...')
        rospy.sleep(1)

    def update_vel(self, linear_vel, angular_vel):
        """Update the robot's velocity."""
        msg = Twist()
        msg.linear.x = linear_vel
        msg.angular.z = angular_vel
        self.cmd_vel_pub.publish(msg)

    def scan_callback(self, msg):
        scan_max_value = msg.range_max

        self.regions = {
            'N':  min(min(msg.ranges[0:5] + msg.ranges[-5:]), scan_max_value),
            'NNW':  min(min(msg.ranges[11:20]), scan_max_value),
            'NW':  min(min(msg.ranges[41:50]), scan_max_value),
            'WNW':  min(min(msg.ranges[64:73]), scan_max_value),
            'W':  min(min(msg.ranges[86:95]), scan_max_value),
            'E':  min(min(msg.ranges[266:275]), scan_max_value),
            'ENE':  min(min(msg.ranges[289:298]), scan_max_value),
            'NE':  min(min(msg.ranges[311:320]), scan_max_value),
            'NNE':  min(min(msg.ranges[341:350]), scan_max_value),
            'frontwide': min(min(msg.ranges[0:40] + msg.ranges[-40:]), 10)
        }

        y0 = self.regions['E'] if self.side == -1 else self.regions['W']
        x1 = (self.regions['ENE'] if self.side == -1 else self.regions['WNW']) * math.sin(math.radians(23))
        y1 = (self.regions['ENE'] if self.side == -1 else self.regions['WNW']) * math.cos(math.radians(23))

        if y0 >= self.distance_wall * 2 and self.regions['N'] < scan_max_value:
            print("Turning sideways")
            self.g_alpha = -math.pi / 4 * self.side
        else:
            front_scan = min([self.regions['N'], self.regions['NNW'] + (scan_max_value - self.regions['WNW']), self.regions['NNE'] + (scan_max_value - self.regions['ENE'])])
            turn_fix = (0 if front_scan >= 0.5 else 1 - front_scan)
            abs_alpha = math.atan2(y1 - self.distance_wall, x1 + self.wall_lead - y0) - turn_fix * 1.5
            self.g_alpha = self.side * abs_alpha

    def run(self):
        start_time = time.time()
        path = []  # Initialize an empty path
        length = 0  # Initialize the length of the path
        completed = False  # Initially, the task is not completed
        try:
            # Assuming this is where you implement the logic for the robot to follow the wall
            while not rospy.is_shutdown():
                self.update_vel(self.speed, self.g_alpha)  # Make sure to call update_vel to move the robot
                self.rate.sleep()
            completed = True  # If the loop exits normally, set completed to True
        except rospy.ROSInterruptException:
            rospy.loginfo("Wall Following Node was interrupted")
        finally:
            # Calculate time taken and return the results
            time_taken = time.time() - start_time
            return path, length, time_taken, completed

if __name__ == "__main__":
    # Initialize the WallFollower node
    wall_follower = WallFollower(speed=0.5, distance_wall=1.0, side='left')

    #

