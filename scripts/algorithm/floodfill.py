from collections import deque

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
    
        # Initialize queue
        queue = deque([self.end])
    
        # Initialize distances for the end cell
        distances[self.end.position[0]][self.end.position[1]] = 0
    
        # Flood fill
        while queue:
            current = queue.popleft()
        
            for neighbor in current.neighbours:
                if neighbor:
                    new_distance = distances[current.position[0]][current.position[1]] + self.calculate_manhattan_distance(neighbor.position, self.end.position)
                    if new_distance < distances[neighbor.position[0]][neighbor.position[1]]:
                        queue.append(neighbor)
                        distances[neighbor.position[0]][neighbor.position[1]] = new_distance
    
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
