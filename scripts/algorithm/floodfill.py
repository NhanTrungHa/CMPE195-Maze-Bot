from collections import deque

class Node:
    def __init__(self, position, distance):
        self.position = position
        self.distance = distance

class FloodFill():
    def __init__(self, maze):
        self.width = maze.width
        self.height = maze.height
        self.start = maze.start
        self.end = maze.end
        self.maze = maze
        
    def calculate_manhattan_distance(self, pos1, pos2):
        return abs(pos1[0] - pos2[0]) + abs(pos1[1] - pos2[1])
    
    def flood_fill(self):
        # Initialize distances for each cell
        distances = [[float('inf') for _ in range(self.width)] for _ in range(self.height)]
        distances[self.end.position[0]][self.end.position[1]] = 0
        
        # Initialize queue
        queue = deque([self.end])
        
        # Flood fill
        while queue:
            current = queue.popleft()
            
            for neighbor in current.neighbours:
                if neighbor and distances[neighbor.position[0]][neighbor.position[1]] == float('inf'):
                    queue.append(neighbor)
                    distances[neighbor.position[0]][neighbor.position[1]] = distances[current.position[0]][current.position[1]] + 1
        
        return distances
    
    def solve(self):
        # Initialize distances using flood fill
        distances = self.flood_fill()
        
        # Initialize start node
        frontier = deque([self.start])
        
        # Initialize visited nodes
        visited = set()
        
        # Initialize previous nodes
        previous = {}
        
        # Initialize path count
        count = 0
        
        completed = False
        
        visited.add(self.start.position)
        
        # BFS
        while frontier:
            count += 1
            current = frontier.popleft()
            
            if current == self.end:
                completed = True
                break
            
            for neighbor in current.neighbours:
                if neighbor and neighbor.position not in visited:
                    node_distance = distances[neighbor.position[0]][neighbor.position[1]]
                    if node_distance <= distances[current.position[0]][current.position[1]]:
                        frontier.append(neighbor)
                        visited.add(neighbor.position)
                        previous[neighbor.position] = current
        
        # Backtracking
        path = []
        current = self.end
        while current is not None:
            path.append(current.position)
            current = previous.get(current.position)
        
        path.reverse()
        
        return path, count, len(path), completed
