#!/usr/bin/env python3

import rospy
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
from scripts.maze import Maze

config = {
    "algorithm": "simplewallfollowing",
    "directory": "map",
    "yamlFile":"map3.yaml",
}

def main():
    # Set up logging
    logging.basicConfig(level=logging.INFO)

    # Map Setup
    os.chdir(r'./src/maze_solver')
    with open(os.path.join(config["directory"], config["yamlFile"])) as file:
        map_config = yaml.load(file, Loader=yaml.FullLoader)

    # Read image and process into binary values
    input = cv2.imread(os.path.join(config["map_dir"], map_config["image"]), -1)
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
    else:
        logging.error("Unknown algorithm specified.")
        return

    logging.info(f"Starting Solve using {name}")

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

    # Wall Following

    else:

        path, length, completed = algorithm.run()

        if completed:
            print("Path found:")
            print(path)
            print("Path length:", length)

        else:
            print("\nNo path found")


if __name__ == '__main__':
    main()

