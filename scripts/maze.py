import numpy as np
from typing import List, Optional, Tuple

class Maze:
    class Node:
        def __init__(self, position: Tuple[int, int]) -> None:
            self.position = position
            self.neighbours = [None, None, None, None]
            self.distance = float('inf')  # Initialize distance to infinity

        def __lt__(self, other: 'Maze.Node') -> bool:
            return self.position < other.position

        def __gt__(self, other: 'Maze.Node') -> bool:
            return self.position > other.position

        def __le__(self, other: 'Maze.Node') -> bool:
            return self < other or self == other

        def __ge__(self, other: 'Maze.Node') -> bool:
            return self > other or self == other

        def set_distance(self, distance: int) -> None:
            self.distance = distance

    def __init__(self, arr: List[List[int]]) -> None:
        maze = np.array(arr)
        self.width, self.height = maze.shape
        self.start, self.end = None, None
        self.nodecount = 0
        left = None
        top_row_nodes: List[Optional['Maze.Node']] = [None] * self.width

        # Starting node
        for y, cell in enumerate(maze[0]):
            if cell == 0:
                self.start = self.Node((0, y))
                self.start.set_distance(0)  # Set start node distance to 0
                top_row_nodes[y] = self.start
                self.nodecount += 1

        for x in range(1, self.width - 1):
            for y, cell in enumerate(maze[x]):
                prev_cell = maze[x, y - 1] if y > 0 else False
                next_cell = maze[x, y + 1] if y < self.height - 1 else False
                current_cell = cell == 0

                if not current_cell:
                    continue

                if prev_cell:
                    n = self.Node((x, y))
                    left.neighbours[1] = n
                    n.neighbours[3] = left

                    if next_cell:
                        left = n
                    else:
                        left = None
                else:
                    n = self.Node((x, y))
                    left = n

                if n is not None:
                    if x > 0 and maze[x - 1, y] == 0:
                        t = top_row_nodes[y]
                        t.neighbours[2] = n
                        n.neighbours[0] = t

                    if x < self.width - 1 and maze[x + 1, y] == 0:
                        top_row_nodes[y] = n
                    else:
                        top_row_nodes[y] = None

                    self.nodecount += 1

        # Ending node
        for y, cell in enumerate(maze[-1]):
            if cell == 0:
                self.end = self.Node((self.width - 1, y))
                top_row_nodes[y].neighbours[2] = self.end
                self.end.neighbours[0] = top_row_nodes[y]
                self.nodecount += 1
                break

        # Calculate distances from each node to the end
        self.calculate_distances()

    def calculate_distances(self) -> None:
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

    def calculate_distance(self, start: 'Maze.Node', end: 'Maze.Node') -> int:
        """
        Calculates the distance between two nodes using Manhattan distance.
        """
        return abs(start.position[0] - end.position[0]) + abs(start.position[1] - end.position[1])
