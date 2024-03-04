import rospy
import logging

from TurtlebotDriving import TurtlebotDriving
from algorithm.wallfollower import WallFollower
config = {
    "algorithm": "wallfollowing"
}

def main():
    os.chdir(r'./src/maze_solver')

    if config["algorithm"].casefold() == "wallfollowing":
        name = "Wall Follower Algorithm"
        algorithm = WallFollower(speed=0.2, distance_wall=0.4, side="right")


    logging.info(f'''Starting Solve using {name}''')

    if name == "Wall Follower Algorithm":
        path, length, timetaken, completed = algorithm.run()

        if completed:
            print("Path found:")
            print(path)
            print("Path length:", length)
        
        else:
            print("\nNo path found")

        print("Time taken :",timetaken,"s\n")

if __name__ == '__main__':
    main()
