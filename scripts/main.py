#!/usr/bin/env python3

import rospy
import logging
import os
from algorithm.wallfollowingbot import WallFollowingBot
from algorithm.depthfirst import DFS
from algorithm.breadthfirst import BFS
from algorithm.floodfill import FloodFill
import cv2
import matplotlib.pyplot as plt
import numpy as np
import yaml
import logging
import time
from maze import Maze
from TurtlebotDriving import TurtlebotDriving

config = {
    "algorithm": "dfs",
    "directory": "map",
    "yamlFile":"map1.yaml",
}

def main():
	# Set up logging
	logging.basicConfig(level=logging.INFO)

	# Map Setup
	os.chdir(r'./src/CMPE195-Maze-Bot')
	with open(os.path.join(config["directory"], config["yamlFile"])) as file:
		map_config = yaml.load(file, Loader=yaml.FullLoader)

	# Read image and process into binary values
	input = cv2.imread(os.path.join(config["directory"], map_config["image"]), -1)
	input = (input != 254).astype(int)

	print("Creating Maze...")

	# Check which algorithm to use
	if config["algorithm"].casefold() == "simplewallfollowing":
		name = "Simple Wall Follower Algorithm"
		algorithm = WallFollowingBot(speed=0.2, wall_distance=0.4, side="right")
	elif config["algorithm"].casefold() == "dfs":
		name = "Depth First Search Algorithm"
		maze = Maze(input)
		algorithm = DFS(maze)

	elif config["algorithm"].casefold() == "bfs":
		name = "Breadth First Search Algorithm"
		maze = Maze(input)
		algorithm = BFS(maze)
	elif config["algorithm"].casefold() == "floodfill":
		name = "Flood Fill Algorithm"
		maze = Maze(input)
		algorithm = FloodFill(maze)

	else:
		raise ValueError(f"Unknown algorithm specified: {config['algorithm']}")

	# dfs and bfs
	if name == "Depth First Search Algorithm" or name == "Breadth First Search Algorithm":
		path, count, length, completed = algorithm.solve()
		if completed:
		    print("Path found:")
		    print(path)
		    print("Node explored:", count)
		    print("Path length:", length)
		
		else:
		    print("\nNo path found")
		
		i = 0
		while i < (len(path)-2):
		    if path[i][0] == path[i+1][0] == path[i+2][0] or path[i][1] == path[i+1][1] == path[i+2][1]:
		        path.remove(path[i+1])

		    else:
		        i+=1

		try:
		    bot = TurtlebotDriving()

		    for i in range(len(path)-1):
		        bot.move(path[i], path[i+1])


		    print("Maze Solved!")
		    bot.plot_trajectory(name)
		    bot.relaunch()
		    print("Time taken :",t1-t0,"s\n")
		    

		except rospy.ROSInterruptException:
		    pass


	# Flood fill
	if name == "Flood Fill Algorithm":
		path, count, length, completed, maze = algorithm.solve()
		distance_array = maze.get_distances_array()
		i=0

		while i < (len(path)-2):
		    if path[i][0] == path[i+1][0] == path[i+2][0] or path[i][1] == path[i+1][1] == path[i+2][1]:
		        path.remove(path[i+1])

		    else:
		        i+=1

		try:
		    bot = TurtlebotDriving()

		    for i in range(len(path)-1):
		        bot.move(path[i], path[i+1])


		    print("Maze Solved!")
		    bot.plot_trajectory(name)
		    bot.relaunch()
		    

		except rospy.ROSInterruptException:
		    pass


		for row in distance_array:
	    		print(row)
		if completed:
	    		print("Path found:")
	    		print(path)
	    		print("Node explored:", count)
	    		print("Path length:", length)

		else:
	    		print("\nNo path found")
    		
    	# Wall Following
	else:
		path, length, timetaken, completed = algorithm.solve()

		if completed:
			print("Path found:")
			print(path)
			print("Path length:", length)
        
		else:
			print("\nNo path found")
			print("Time taken :",timetaken,"s\n")
        


if __name__ == '__main__':
    main()
