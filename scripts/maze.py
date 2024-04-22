import numpy as np


class Maze:
    class Node:
        def __init__(self, position=None):
            self.position = position
            self.distance = float('inf')  # Initialize distance to infinity
            self.neighbours = [None, None, None, None]

        def __lt__(self, other):
            return (self.position < other.position)

        def __gt__(self, other):
            return (self.position > other.position)

        def __le__(self, other):
            return (self < other) or (self == other)

        def __ge__(self, other):
            return (self > other) or (self == other)

    def calculate_distances(self):
        """
        Calculate distances from each node to the end node using Euclidean distance.
        """
        # Initialize distance from end node to itself as 0
        self.end.distance = 0

        # Initialize a queue for the flood-fill algorithm
        queue = [self.end]

        # Perform flood-fill until the queue is empty
        while queue:
            current_node = queue.pop(0)  # Get the next node from the queue

            # Check all neighbors of the current node
            for neighbor in current_node.neighbours:
                # Skip if the neighbor is None
                if neighbor is None:
                    continue

                # Calculate the Euclidean distance between the current node and its neighbor
                distance_to_neighbor = self.calculate_distance_between_two_nodes(current_node, neighbor)

                # Update the neighbor's distance if the newly calculated distance is shorter
                if current_node.distance + distance_to_neighbor < neighbor.distance:
                     neighbor.distance = current_node.distance + distance_to_neighbor
                     # Add the neighbor to the queue to process its neighbors in the next iteration
                     queue.append(neighbor)


    def calculate_distance_between_two_nodes(self, start: 'Maze.Node', end: 'Maze.Node'):
        """
        Calculates the distance between two nodes using Manhattan distance.
        """
        return abs(start.position[0] - end.position[0]) + abs(start.position[1] - end.position[1])

    def __init__(self, arr):

        maze = np.array(arr)

        self.width = maze.shape[0]
        self.height = maze.shape[1]

        self.start = None
        self.end = None
        nodecount = 0
        left = None
        toprownodes = [None] * self.width

        # Starting node
        for y in range(self.height):
            if maze[0, y] == 0:
                self.start = Maze.Node((0, y))
                toprownodes[y] = self.start
                nodecount += 1

        for x in range(1, self.width - 1):

            prev = False
            current = False
            next = maze[x, 1] == 0

            for y in range(1, self.height - 1):

                prev = current
                current = next
                next = maze[x, y + 1] == 0

                n = None

                if not current:
                    continue

                if prev:
                    n = Maze.Node((x, y))
                    left.neighbours[1] = n
                    n.neighbours[3] = left

                    if next:
                        left = n
                    else:
                        left = None

                else:
                    n = Maze.Node((x, y))
                    left = n

                if n is not None:
                    if maze[x - 1, y] == 0:
                        t = toprownodes[y]
                        t.neighbours[2] = n
                        n.neighbours[0] = t

                    if maze[x + 1, y] == 0:
                        toprownodes[y] = n

                    else:
                        toprownodes[y] = None

                    nodecount += 1

        # Ending node
        for y in range(self.height):
            if maze[-1, y] == 0:
                self.end = Maze.Node((self.height - 1, y))
                t = toprownodes[y]
                t.neighbours[2] = self.end
                self.end.neighbours[0] = t
                nodecount += 1
                break

        self.nodecount = nodecount
        self.calculate_distances()





