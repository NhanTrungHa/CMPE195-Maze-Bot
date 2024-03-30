#!/usr/bin/env python3

import rospy
import logging
import os
from wallfollowing import WallFollower
from simplewallfollowing import SimpleWallFollower

config = {
    "algorithm": "simplewallfollowing"
}

def main():
    # Set up logging
    logging.basicConfig(level=logging.INFO)

    # Check which algorithm to use
    if config["algorithm"].casefold() == "wallfollowing":
        name = "Wall Follower Algorithm"
        algorithm = WallFollower(speed=0.2, distance_wall=0.4, side="right")
    elif config["algorithm"].casefold() == "simplewallfollowing":
        name = "Simple Wall Follower Algorithm"
        algorithm = SimpleWallFollower(robot_speed=0.3, wall_distance=0.5, wall_side="right")
    else:
        logging.error("Unknown algorithm specified.")
        return

    logging.info(f"Starting Solve using {name}")

    # Run the selected algorithm
    try:
        algorithm.run()
        # Note: In this structure, we're assuming run() doesn't return a value and runs indefinitely or until ROS is shut down.
        logging.info("Algorithm execution completed.")
    except rospy.ROSInterruptException:
        logging.info(f"{name} was interrupted.")

if __name__ == '__main__':
    main()

