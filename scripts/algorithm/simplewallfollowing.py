#!/usr/bin/env python3
# License information omitted for brevity

import rospy
from TurtlebotDriving import TurtlebotDriving
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math


class SimpleWallFollower():
    def __init__(self, speed, wall_distance, side):
        self.position_history_x = []
        self.position_history_y = []

        self.speed = speed
        self.wall_distance = wall_distance
        self.lead_distance = 0.5

        self.side_factor = 1 if side == 'left' else -1
        self.turn_rate = 0

        rospy.init_node('PythonControl')

        self.rate = rospy.Rate(10)

        self.speed_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1000)
        rospy.Subscriber("scan", LaserScan, self.laser_scan_callback)
        rospy.loginfo(f'Wall following initiated on the {side} side...')
        rospy.sleep(1)

    def update_speed(self, linear_speed, angular_speed):
        """Method to update the speed of the turtlebot."""

        msg = Twist()
        msg.linear.x = linear_speed
        msg.angular.z = angular_speed
        self.speed_publisher.publish(msg)

    def laser_scan_callback(self, msg):
        max_scan_value = msg.range_max

        # Each sector covers 9 degrees
        self.scan_zones = {
            'North': min(min(msg.ranges[0:5] + msg.ranges[-5:]), max_scan_value),
            'North-North-West': min(min(msg.ranges[11:20]), max_scan_value),
            'North-West': min(min(msg.ranges[41:50]), max_scan_value),
            'West-North-West': min(min(msg.ranges[64:73]), max_scan_value),
            'West': min(min(msg.ranges[86:95]), max_scan_value),
            'East': min(min(msg.ranges[266:275]), max_scan_value),
            'East-North-East': min(min(msg.ranges[289:298]), max_scan_value),
            'North-East': min(min(msg.ranges[311:320]), max_scan_value),
            'North-North-East': min(min(msg.ranges[341:350]), max_scan_value),
            'wide_front': min(min(msg.ranges[0:40] + msg.ranges[-40:]), 10)
        }

        close_wall = self.scan_zones['East'] if self.side_factor == -1 else self.scan_zones['West']
        next_x = (self.scan_zones['East-North-East'] if self.side_factor == -1 else self.scan_zones[
            'West-North-West']) * math.sin(math.radians(23))
        next_y = (self.scan_zones['East-North-East'] if self.side_factor == -1 else self.scan_zones[
            'West-North-West']) * math.cos(math.radians(23))

        # Adjust the bot's direction if too close to a wall
        if close_wall >= self.wall_distance * 2 and self.scan_zones['North'] < max_scan_value:
            rospy.loginfo("Adjusting direction due to proximity to wall")
            self.turn_rate = -math.pi / 4 * self.side_factor
        else:
            # Frontal scan to determine wall proximity
            frontal_scan = min([self.scan_zones['North'], self.scan_zones['North-North-West'] + (
                        max_scan_value - self.scan_zones['West-North-West']), self.scan_zones['North-North-East'] + (
                                            max_scan_value - self.scan_zones['East-North-East'])])
            adjust_turn = (0 if frontal_scan >= 0.5 else 1 - frontal_scan)
            abs_turn_rate = math.atan2(next_y - self.wall_distance,
                                       next_x + self.lead_distance - close_wall) - adjust_turn * 1.5
            self.turn_rate = self.side_factor * abs_turn_rate

    def run(self):

        try:
            bot_driver = TurtlebotDriving()

            operation_complete = False

            while self.scan_zones['wide_front'] < 10 and not rospy.is_shutdown():
                self.update_speed(self.speed, self.turn_rate)
                self.rate.sleep()

            rospy.loginfo("Maze navigation completed!")
            bot_driver.stop()
            trajectory = bot_driver.obtainpath()
            bot_driver.relaunch()
            operation_complete = True

            return trajectory, len(trajectory), operation_complete

        except rospy.ROSInterruptException:
            pass
