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


        def get_distances_array(self):
            """
	    Returns a 2D array containing distances of each node from the end node.
	    """
            distances_array = [[node.distance for node in row] for row in self.iterate_maze()]
            return distances_array


        def iterate_maze(self):
            """
	    Iterates through the maze to collect all nodes.
	    """
            maze_nodes = [[None for _ in range(self.height)] for _ in range(self.width)]
            queue = [self.start]
            visited = set()

            while queue:
                current_node = queue.pop(0)
                visited.add(current_node)
                x, y = current_node.position
                maze_nodes[x][y] = current_node

                for neighbour in current_node.neighbours:
                    if neighbour is not None and neighbour not in visited:
                        queue.append(neighbour)

            return maze_nodes


        def calculate_distances(self):
            """
            Calculate distances from each node to the end node using Euclidean distance.
            """
            if self.end is None:
                return

            queue = [self.end]
            visited = set()

            while queue:
                current_node = queue.pop(0)
                visited.add(current_node)

                for neighbour in current_node.neighbours:
                    if neighbour is not None and neighbour not in visited:
                        neighbour_distance = self.calculate_distance(neighbour, self.end)
                        neighbour.set_distance(neighbour_distance)
                        queue.append(neighbour)


        def calculate_distance(self, start: 'Maze.Node', end: 'Maze.Node'):
            """
            Calculates the distance between two nodes using Manhattan distance.
            """
            return abs(start.position[0] - end.position[0]) + abs(start.position[1] - end.position[1])


        def calculate_distance(self, start: 'Maze.Node', end: 'Maze.Node'):
            """
	    Calculates the distance between two nodes using Manhattan distance.
	    """
            return abs(start.position[0] - end.position[0]) + abs(start.position[1] - end.position[1])
