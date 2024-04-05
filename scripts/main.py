#!/usr/bin/env python3

import rospy
from wallfollowingbot import WallFollowingBot
from TurtlebotDriving import TurtlebotDriving

def get_algorithm(config):
    algorithm_type = config.get("algorithm", "").casefold()

    if algorithm_type == "wallfollowingbot":
        return WallFollowingBot(speed=0.2, wall_distance=0.3, side="right"), "Wall Following Algorithm"
    else:
        raise ValueError(f"Unknown algorithm specified: {config['algorithm']}")

def main():
    config = {
        "algorithm": "wallfollowingbot"
    }

    try:
        algorithm, name = get_algorithm(config)
        rospy.loginfo(f"Starting {name}")
        algorithm.run()
        rospy.spin()  # Keeps the node running until manually interrupted
        rospy.loginfo("Algorithm execution completed successfully.")
    except ValueError as e:
        rospy.logerr(e)
    except rospy.ROSInterruptException:
        rospy.loginfo(f"{name} was interrupted by ROS.")
    except Exception as e:
        rospy.logerr(f"An unexpected error occurred: {str(e)}")

if __name__ == '__main__':
    main()

