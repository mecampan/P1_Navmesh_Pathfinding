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
    startBox = get_box_from_point(mesh, source_point)
    endBox = get_box_from_point(mesh, destination_point)

    # Prevent out of bounds from being chosen
    if startBox is None or endBox is None:
        print("No Path Found!")
        return [], []
    
    path = []
    boxes = {}


   # Bi-Directional A* Algorithm    
    # Priority queues for A* from start and end
    start_queue = []
    end_queue = []
    heappush(start_queue, (0, startBox))
    heappush(end_queue, (0, endBox))

    # Dictionaries to track visited boxes and costs
    start_visited = {startBox: None}
    end_visited = {endBox: None}
    start_g_score = {startBox: 0}
    end_g_score = {endBox: 0}

    while start_queue and end_queue:
        # Expand from start
        if start_queue:
            _, current_box = heappop(start_queue)
            if current_box in end_visited:
                return reconstruct_bidirectional_path(start_visited, end_visited, current_box, source_point, destination_point), list(start_visited.keys()) + list(end_visited.keys())
            for neighbor in mesh['adj'][current_box]:
                new_g = start_g_score[current_box] + distance_between(current_box, neighbor)
                if neighbor not in start_g_score or new_g < start_g_score[neighbor]:
                    start_g_score[neighbor] = new_g
                    priority = new_g + A_heuristic(neighbor, destination_point)
                    heappush(start_queue, (priority, neighbor))
                    start_visited[neighbor] = current_box

        # Expand from end
        if end_queue:
            _, current_box = heappop(end_queue)
            if current_box in start_visited:
                return reconstruct_bidirectional_path(start_visited, end_visited, current_box, source_point, destination_point), list(start_visited.keys()) + list(end_visited.keys())
            for neighbor in mesh['adj'][current_box]:
                new_g = end_g_score[current_box] + distance_between(current_box, neighbor)
                if neighbor not in end_g_score or new_g < end_g_score[neighbor]:
                    end_g_score[neighbor] = new_g
                    priority = new_g + A_heuristic(neighbor, source_point)
                    heappush(end_queue, (priority, neighbor))
                    end_visited[neighbor] = current_box

    """
    # Find path using BFS
    queue = []
    visited = {get_box_from_point(mesh, source_point): None}

    # A* Algorithm    
    heappush(queue, (0, startBox))  # maintain a priority queue of cells
    pathcosts = {get_box_from_point(mesh, source_point): 0}       # maps cells to their pathcosts (found so far)

    while queue:
        
        priority, currentBox = heappop(queue)
        print("currentBox:", currentBox)
        currentPoint = get_point_from_box(currentBox)        
        
        if currentBox == endBox: 			                # success test
            path = path_to(visited, source_point, destination_point, startBox, endBox)
            boxes = boxes_visited(visited, source_point, endBox)            
            break
        else:
            for adjBox in mesh['adj'][currentBox]:
                adjPoint = get_point_from_box(adjBox)
                cost = heuristic(currentPoint, adjPoint)
                newCost = pathcosts[currentBox] + cost
                
                if adjBox not in pathcosts or newCost < pathcosts[adjBox]:
                    pathcosts[adjBox] = newCost
                    priority = newCost + heuristic(adjPoint, destination_point)
                    queue.append([priority, adjBox])
                    visited[adjBox] = currentBox

    # BFS Algorithm
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
    """

    if not path:
        print("No Path Found!")
    
    return path, boxes.keys()

def heuristic(start, goal):
    (x1, y1) = start
    (x2, y2) = goal

    return abs(x1 - x2) + abs(y1 - y2)

def A_heuristic(box, target_point):
        return sqrt((target_point[0] - get_point_from_box(box)[0])**2 +
                    (target_point[1] - get_point_from_box(box)[1])**2)

def reconstruct_bidirectional_path(start_visited, end_visited, meeting_box, source_point, destination_point):
    path = []
    # Reconstruct path from start to meeting point
    current = meeting_box
    while current is not None:
        path.append(get_point_from_box(current))
        current = start_visited[current]
    path.reverse()

    # Append the meeting box point
    path.append(get_point_from_box(meeting_box))

    # Reconstruct path from end to meeting point
    current = end_visited[meeting_box]
    while current is not None:
        path.append(get_point_from_box(current))
        current = end_visited[current]

    # Insert source and destination points
    if path:
        path[0] = source_point
        path[-1] = destination_point
        
    return path

def get_box_from_point(mesh, point):
    print(point)
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

def distance_between(boxA, boxB):
    return sqrt((get_point_from_box(boxA)[0] - get_point_from_box(boxB)[0])**2 +
                (get_point_from_box(boxA)[1] - get_point_from_box(boxB)[1])**2)

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