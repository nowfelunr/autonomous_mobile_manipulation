import numpy as np


import numpy as np

def calculate_distance(p1, p2):
    """ Calculate Euclidean distance between two PointCustom objects """
    return np.sqrt((p1.x - p2.x) ** 2 + (p1.y - p2.y) ** 2 + (p1.z - p2.z) ** 2)

def nearest_neighbor(points_dict):
    """ Find a nearest neighbor tour among the given points, returning list of tuples (index, PointCustom) """
    if not points_dict:
        return []

    # Extract the keys and corresponding PointCustom objects into lists
    indices = list(points_dict.keys())
    points = list(points_dict.values())
    n = len(points)
    
    if n == 1:
        return [(indices[0], points[0])]  # Only one point, return it directly
    
    # Initialize tour starting at the first point
    current_idx = 0
    tour = [(indices[current_idx], points[current_idx])]
    unvisited = set(range(1, n))  # Excluding the first point from unvisited

    # Calculate distances between all pairs of points
    dist_matrix = [[calculate_distance(points[i], points[j]) for j in range(n)] for i in range(n)]
    
    while unvisited:
        next_idx = min(unvisited, key=lambda x: dist_matrix[current_idx][x])
        tour.append((indices[next_idx], points[next_idx]))
        current_idx = next_idx
        unvisited.remove(current_idx)

    # Return to the start to close the tour loop
    tour.append(tour[0])
    return tour


np.random.seed(42)
points = np.random.rand(20, 3)
print(points)
