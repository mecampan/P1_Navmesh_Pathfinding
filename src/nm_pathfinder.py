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

    #print("source_point: ", source_point)
    #print("destination_point: ", destination_point)
    #print("mesh: ", mesh)

    path = []
    boxes = {}

    # Find path using BFS
    queue = []
    visited = {get_box_from_point(mesh, source_point): None}

    startBox = get_box_from_point(mesh, source_point)
    endBox = get_box_from_point(mesh, destination_point)
    
    queue.append(startBox)
    
    while queue:					                # queue and process
        current_box = queue.pop()

        if current_box == endBox: 			                # success test
            path = path_to_box(visited, endBox)
            break
        
        else:
            for neighbor in mesh['adj'][current_box]:
                if neighbor not in visited:
                    visited[neighbor] = current_box
                    queue.append(neighbor)

    if not path:
        print("No Path Found!")
        return path, boxes.keys()
    

    # Grab all boxes the path passes through
    path = [source_point] + path
    path.append(destination_point)

    print("Path Found:")
    for point in path:
        box = get_box_from_point(mesh, point)
        print("Checking point: ", point, " within box: ", box)
        if box in boxes:
            print("Already in boxes")
        else:
            if box is not None:
                print("Adding Box")
                boxes[box] = point

    print("\nBoxes Passed: ", boxes, "\nPath: ", path, "\n")

    
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
    # Return the center point of the box
    x = max(boxA[0], boxB[0]), min(boxA[1], boxB[1])
    y = max(boxA[2], boxB[2]), min(boxA[3], boxB[3])
    return (x[0] + x[1]) / 2, (y[0] + y[1]) / 2

def path_to_box(visited, end_box):
    path = []
    current_box = end_box
    neighbor = visited[current_box]

    while neighbor is not None:
        path.append(get_constrained_point_from_box(neighbor, current_box))
        current_box = visited[current_box]
        neighbor = visited[current_box]
    path.reverse()  # Reverse the path to start from the source box
    return path