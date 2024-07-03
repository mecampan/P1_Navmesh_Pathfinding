from math import inf, sqrt
from heapq import heappop, heappush
import queue

def find_path (source_point, destination_point, mesh):

    """
    Searches for a path from source_point to destination_point through the mesh

    Args:
        source_point: starting point of the pathfinder
        destination_point: the ultimate goal the pathfinder must reach
        mesh: pathway constraints the path adheres to

    Returns:

        A path (list of points) from source_point to destination_point if exists
        A list of boxes explored by the algorithm
    """

    path = []
    boxes = {}

    # Find path using BFS
    queue = []
    visited = {get_box_from_point(mesh, source_point): None}

    startBox = get_box_from_point(mesh, source_point)
    endBox = get_box_from_point(mesh, destination_point)

    # Prevent out of bounds from being chosen
    if startBox is None or endBox is None:
        print("No Path Found!")
        return path, boxes.keys()

    queue.append(startBox)
    while queue:					                # queue and process
        current_box = queue.pop()

        if current_box == endBox: 			                # success test
            path = path_to(visited, source_point, destination_point, startBox, endBox)
            boxes = boxes_visited(visited, source_point, endBox)            
            break
        
        if current_box is not None:
            for neighbor in mesh['adj'][current_box]:
                if neighbor not in visited and neighbor is not None:
                    visited[neighbor] = current_box
                    queue.append(neighbor)

    if not path:
        print("No Path Found!")
    
    return path, boxes.keys()

def get_box_from_point(mesh, point):
    for box in mesh['boxes']:
        if box[0] <= point[0] <= box[1] and box[2] <= point[1] <= box[3]:
            return box
    return None

def get_point_from_box(box):
    # Return the center point of the box
    return ((box[0] + box[1]) / 2, (box[2] + box[3]) / 2)


def get_constrained_point_from_box(boxA, boxB):
    # Return the connecting points of two boxes
    x = max(boxA[0], boxB[0]), min(boxA[1], boxB[1])
    y = max(boxA[2], boxB[2]), min(boxA[3], boxB[3])
    return (x[0] + x[1]) / 2, (y[0] + y[1]) / 2

def path_to(visited, startPoint, endPoint, startBox, endBox):
    path = []

    if startBox == endBox:
        path.append(startPoint)
        path.append(endPoint)
        return path
    
    current_box = endBox
    neighbor = visited[current_box]

    while neighbor is not None:
        path.append(get_constrained_point_from_box(neighbor, current_box))
        current_box = visited[current_box]
        neighbor = visited[current_box]
    
    path.insert(0, endPoint)
    path.append(startPoint)
    return path

def boxes_visited(visited, startPoint, endBox):
    boxes = {}
    current_box = endBox

    while visited[current_box] is not None:
        boxes[current_box] = get_constrained_point_from_box(visited[current_box], current_box)
        current_box = visited[current_box]
  
    boxes[current_box] = startPoint
    return boxes