from collections import deque

class DFS():
    def __init__(self, maze):
        self.width = maze.width
        self.height = maze.height
        self.start = maze.start
        self.end = maze.end

    def solve(self):
        stack, visited, previous, count, completed = self.initialize()
        stack, visited, previous, count, completed = self.dfs_traversal(stack, visited, previous, count, completed)
        path = self.backtrack(previous)
        return path, count, len(path), completed

    def initialize(self):
        stack = deque([self.start])
        visited = [False] * (self.width * self.height)
        previous = [None] * (self.width * self.height)
        count = 0
        completed = False
        visited[self.start.position[0] * self.width + self.start.position[1]] = True
        return stack, visited, previous, count, completed

    def dfs_traversal(self, stack, visited, previous, count, completed):
        while stack:
            count += 1
            current = stack.pop()
            if current == self.end:
                completed = True
                break
            for n in current.neighbours:
                if n is not None and not visited[n.position[0] * self.width + n.position[1]]:
                    nodepos = n.position[0] * self.width + n.position[1]
                    stack.append(n)
                    visited[nodepos] = True
                    previous[nodepos] = current
        return stack, visited, previous, count, completed

    def backtrack(self, previous):
        pathnode = deque()
        current = self.end
        while current is not None:
            nodepos = current.position[0] * self.width + current.position[1]
            pathnode.appendleft(current)
            current = previous[nodepos]
        path = [coord.position for coord in pathnode]
        return path
