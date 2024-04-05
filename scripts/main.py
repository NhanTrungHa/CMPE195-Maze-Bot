#!/usr/bin/env python3

import rospy
<<<<<<< HEAD
from wallfollowingbot import WallFollowingBot
from TurtlebotDriving import TurtlebotDriving
=======
import logging
import os
from algorithm.simplewallfollowing import SimpleWallFollower
from algorithm.depthfirst import DFS
from algorithm.breadthfirst import BFS
from algorithm.floodfill import FloodFill
import cv2
import matplotlib.pyplot as plt
import numpy as np
import os
import yaml
import logging
import time

from scripts.TurtlebotDriving import TurtlebotDriving
>>>>>>> 72a18a06ee8a9979dee29244878c4cff144e9e9c

def get_algorithm(config):
    algorithm_type = config.get("algorithm", "").casefold()

<<<<<<< HEAD
    if algorithm_type == "wallfollowingbot":
        return WallFollowingBot(speed=0.2, wall_distance=0.3, side="right"), "Wall Following Algorithm"
=======
def main():
    # Set up logging
    logging.basicConfig(level=logging.INFO)

    # Map Setup
    os.chdir(r'./src/maze_solver')
    with open(os.path.join(config["map_dir"], config["map_info"])) as file:
        map_config = yaml.load(file, Loader=yaml.FullLoader)

    # Read image
    input = cv2.imread(os.path.join(config["map_dir"], map_config["image"]), -1)

    # Make wall = 1, path = 0
    input = (input != 254).astype(int)

    print("Creating Maze...")
    maze = Maze(input)

    # Check which algorithm to use
    if config["algorithm"].casefold() == "simplewallfollowing":
        name = "Simple Wall Follower Algorithm"
        algorithm = SimpleWallFollower(speed=0.2, distance_wall=0.4, side="right")
    elif config["algorithm"].casefold() == "dfs":
        name = "Depth First Search Algorithm"
        algorithm = DFS(maze)
    elif config["algorithm"].casefold() == "bfs":
        name = "Breadth First Search Algorithm"
        algorithm = BFS(maze)
    elif config["algorithm"].casefold() == "floodfill":
        name = "Flood Fill Algorithm"
        algorithm = FloodFill(maze)
>>>>>>> 72a18a06ee8a9979dee29244878c4cff144e9e9c
    else:
        raise ValueError(f"Unknown algorithm specified: {config['algorithm']}")

def main():
    config = {
        "algorithm": "wallfollowingbot"
    }

<<<<<<< HEAD
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
=======
    if name != "Wall Following":

        t0 = time.time()
        path, count, length, completed = algorithm.solve()
        t1 = time.time()

        if completed:
            print("Path found:")
            print(path)
            print("Node explored:", count)
            print("Path length:", length)

        else:
            print("\nNo path found")

    i = 0

    while i < (len(path) - 2):
        if path[i][0] == path[i + 1][0] == path[i + 2][0] or path[i][1] == path[i + 1][1] == path[i + 2][1]:
            path.remove(path[i + 1])

        else:
            i += 1

    try:
        bot = TurtlebotDriving()

        for i in range(len(path) - 1):
            bot.move(path[i], path[i + 1])

        print("Maze Solved!")
        bot.relaunch()

    except rospy.ROSInterruptException:
        pass

    # --------------------------------- Automous Solving ---------------------------------

    # Wall Following

    else:

        path, length, timetaken, completed = algorithm.run()

        if completed:
            print("Path found:")
            print(path)
            print("Path length:", length)

        else:
            print("\nNo path found")

        print("Time taken :", timetaken, "s\n")
>>>>>>> 72a18a06ee8a9979dee29244878c4cff144e9e9c

if __name__ == '__main__':
    main()

