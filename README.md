# Maze Navigation Robot

This project implements a maze navigation robot capable of autonomously navigating through a maze environment. The robot is designed to explore the maze, locate the destination, and efficiently traverse from the starting point to the destination while avoiding obstacles.

## Features

- Autonomous navigation: The robot utilizes various sensors and algorithms to navigate through the maze without human intervention.
- Maze exploration: The robot systematically explores the maze environment to create a map and identify potential paths to the destination.
- Obstacle avoidance: Equipped with obstacle detection sensors, the robot can detect and avoid obstacles in its path to ensure safe navigation.
- Pathfinding: Using a suitable pathfinding algorithm, the robot determines the shortest or most efficient path from the starting point to the destination within the maze.

## Components

- Hardware:
  TurtleBot3 (Burger)

- Software:
  - Maze exploration algorithm
  - Obstacle avoidance algorithm
  - Pathfinding algorithm
  - Control and decision-making logic
  - Robot Operating System (ROS)

## How to Run
1. **Set up turtlebot noetic**: Follow the guide on Turtlebot3 Quick Start Guide
2. **Clone Repo**: Clone the repo onto a folder on your local 
3. **Run these commands**: 
- $cd catkin_ws/src
- $git clone {our respository link}
- $cd ~/catkin_ws
- $catkin_make
- $source ~/catkin_ws/devel/setup.bash
- $chmod a+x ~/catkin_ws/src/CMPE195-Maze-Bot/scripts/main.py
- $roslaunch {Folder Name} maze1.launch

Wait for Gazebo to launch.
On a seperate terminal:
Navigate to the folder with the scripts
- $rosrun {Folder Name} main.py

##Github Repository


