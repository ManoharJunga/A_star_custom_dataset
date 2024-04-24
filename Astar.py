import csv
import matplotlib.pyplot as plt
import math
from heapq import heappush, heappop

# Function to calculate Haversine distance between two points
def haversine(lat1, lon1, lat2, lon2):
    R = 6371.0  # Radius of the Earth in km
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    a = math.sin(dlat / 2) ** 2 + math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.sin(dlon / 2) ** 2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    distance = R * c
    return distance

# A* algorithm
def astar(graph, start, goal):
    # Heuristic function (Haversine distance)
    def heuristic(node):
        return haversine(node[0], node[1], goal[0], goal[1])
    
    # Initialize open and closed sets
    open_set = [(0, start)]
    came_from = {}
    g_score = {node: float('inf') for node in graph}
    g_score[start] = 0
    f_score = {node: float('inf') for node in graph}
    f_score[start] = heuristic(start)
    
    while open_set:
        current = heappop(open_set)[1]
        
        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            return path[::-1]
        
        for neighbor in graph[current]:
            tentative_g_score = g_score[current] + graph[current][neighbor]
            if tentative_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = tentative_g_score + heuristic(neighbor)
                if neighbor not in [node[1] for node in open_set]:
                    heappush(open_set, (f_score[neighbor], neighbor))
    
    return None

# Parse CSV file and create graph
graph = {}
with open('gps.csv', 'r') as file:
    reader = csv.reader(file)
    next(reader)  # Skip header
    points = [(float(row[0]), float(row[1])) for row in reader]

# Create graph with edges between neighboring points within a certain distance threshold
threshold_distance = 0.1  # Adjust this threshold as needed
for i, point1 in enumerate(points):
    graph[point1] = {}
    for j, point2 in enumerate(points):
        if i != j:
            distance = haversine(point1[0], point1[1], point2[0], point2[1])
            if distance < threshold_distance:
                graph[point1][point2] = distance

# Choose start and goal points (you can change these)
start_point = (16.462086, 80.506577)
goal_point = (16.463552, 80.508575)

# Find path using A*
path = astar(graph, start_point, goal_point)

# Plot the points and the path
plt.figure(figsize=(10, 8))
plt.scatter(*zip(*graph.keys()), color='red', label='Points')
plt.scatter(*start_point, color='green', label='Start')
plt.scatter(*goal_point, color='blue', label='Goal')
if path:
    path_x, path_y = zip(*path)
    plt.plot(path_x, path_y, color='orange', label='Path')
plt.xlabel('Latitude')
plt.ylabel('Longitude')
plt.title('Path Finding Using A* Algorithm')
plt.legend()
plt.grid(True)
plt.show()
