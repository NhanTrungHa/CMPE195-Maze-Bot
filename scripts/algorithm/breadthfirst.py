from collections import deque

class BFS:
    def __init__(self, maze):
        self.width = maze.width
        self.height = maze.height
        self.start = maze.start
        self.end = maze.end
        self.visited = [False] * (self.width * self.height)
        self.previous = [None] * (self.width * self.height)

    def solve(self):
        frontier = deque([self.start])
        count = 0
        completed = False

        self.mark_as_visited(self.start)
        
        while frontier:
            count += 1
            current = frontier.popleft()

            if current == self.end:
                completed = True
                break

            for neighbor in current.neighbours:
                if neighbor is not None and not self.is_visited(neighbor):
                    self.mark_as_visited(neighbor)
                    frontier.append(neighbor)
                    self.previous[neighbor.position[0] * self.width + neighbor.position[1]] = current

        path = self.generate_path()
        return path, count, len(path), completed

    def mark_as_visited(self, node):
        self.visited[node.position[0] * self.width + node.position[1]] = True

    def is_visited(self, node):
        return self.visited[node.position[0] * self.width + node.position[1]]

    def generate_path(self):
        path = deque()
        current = self.end

        while current is not None:
            path.appendleft(current)
            current = self.previous[current.position[0] * self.width + current.position[1]]

        return [coord.position for coord in path]
