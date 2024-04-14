from collections import deque


class FloodFill:
    def __init__(self, maze):
        self.maze = maze
        self.width = maze.width
        self.height = maze.height
        self.start = maze.start
        self.end = maze.end

    def solve(self):
        frontier = deque([self.start])  # Initialize queue

        # nodes array
        visited = [False] * (self.width * self.height)
        previous = [None] * (self.width * self.height)

        count = 0

        while frontier:
            count += 1
            current = frontier.popleft()  # Take front node from queue

            if current == self.end:
                completed = True
                break

            min_distance = min(neighbor.distance for neighbor in current)
            for n in current.neighbours:
                if current.distance <= min_distance:
                    current.distance = min_distance + 1  # Update distance of current node

                if n is not None:
                    nodepos = n.position[0] * self.width + n.position[1]

                    if not visited[nodepos]:
                        frontier.append(n)  # Add accessible neighbors to queue
                        visited[nodepos] = True
                        previous[nodepos] = current

        # Backtrack to find the path
        pathnode = deque()
        current = self.end

        while current is not None:
            nodepos = current.position[0] * self.width + current.position[1]
            pathnode.appendleft(current)
            current = previous[nodepos]

        path = [coord.position for coord in pathnode]
        return path, count, len(path), completed, self.maze