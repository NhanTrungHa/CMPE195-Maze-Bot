from collections import deque

class FloodFill:
    def __init__(self, maze):
        self.maze = maze
        self.width = maze.width
        self.height = maze.height
        self.start = maze.start
        self.end = maze.end

    def solve(self):
        frontier = deque([self.start]) # Initialize queue for BFS
        deque.front = 0  # Set distance of start node to 0

        # visited node
        visited = [False] * (self.width * self.height)

        # previous node
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
        return path, count, len(path), completed

    def get_accessible_neighbors(self, node):
        neighbors = []
        for dx, dy in [(1, 0), (-1, 0), (0, 1), (0, -1)]:
            x, y = node.position[0] + dx, node.position[1] + dy
            if 0 <= x < self.width and 0 <= y < self.height:
                neighbor = self.maze.get_node((x, y))
                if neighbor is not None:
                    neighbors.append(neighbor)
        return neighbors
