import numpy as np
import matplotlib.pyplot as plt
import math


def pathsToTSP(paths):
    """Converts paths into TSP nodes and creates a distance matrix

    This function takes the paths created in the fieldToPaths function
    and creates nodes that describe the endpoints of the paths.

    Parameters
    ----------
    paths : np.array
        [[[line0_x0, line0_x1],
          [line0_y0, line0_y1]],
         [[line1_x0, line1_x1],
          [line1_y0, line1_y1]],
        ...                    ]
        The paths that the vehicle is to follow. It is defined by a sets of
        x,y points representing the endpoints of the paths. It is a three
        dimensional array with the first dimension representing the path
        and then a 2D array of points below that.

    Returns
    ----------
    tsp_nodes: np.array
        [[x0, x1, x2, x3,...],
         [y0, y1, y2, y3,...]]

         The points that represent the nodes for the TSP problem.
         The nodes are derived from endpoints in the paths array.
    distance: np.array
        [[d00, d10, d20, d30,...],
         [d01, d11, d21, d31,...]
         ...]
        A distance matrix of the distances (d_xy) between node x and node y.
         """

    tsp_nodes = np.array([np.hstack(paths[:, 0, :]),
                          np.hstack(paths[:, 1, :])])

    distances = np.zeros((np.shape(tsp_nodes)[1], np.shape(tsp_nodes)[1]))
    for node1 in range(0, np.shape(tsp_nodes)[1]):
        for node2 in range(node1, np.shape(tsp_nodes)[1]):
            dist = math.sqrt((tsp_nodes[0][node1] -
                              tsp_nodes[0][node2]) ** 2 +
                             (tsp_nodes[1][node1] -
                              tsp_nodes[1][node2]) ** 2)
            distances[node1, node2] = dist
            distances[node2, node1] = dist
    return tsp_nodes, distances


def tspToSolution1(nodes, cost_mat):
    """Creates a solution to the TSP problem.
    Converts nodes to Cities and uses function from TSP notebook
    Cities has now been redefined as its own type of class instead of a
    complex point. It includes a varible to record its node number so that it
    can be referenced later.

    Parameters
    ----------
    nodes : np.array
    [[x0, x1, x2...]
     [y0, y1, y2...]]


    Returns
    ----------
    tour : list of Cities
    """
    # define neccessary functions from TSP notebook
    def cost(A, B):
        return cost_mat[A.num, B.num]

    def shortest_edges_first(cities):
        # Return all edges between distinct cities, sorted shortest first."
        edges = [(A, B) for A in cities for B in cities
                 if id(A) < id(B)]
        return sorted(edges, key=lambda edge: cost(*edge))

    def join_endpoints(endpoints, A, B):
        # Join B's segment onto the end of A's and return the segment.
        # Maintain endpoints dict."
        Asegment, Bsegment = endpoints[A], endpoints[B]
        if Asegment[-1] is not A:
            Asegment.reverse()
        if Bsegment[0] is not B:
            Bsegment.reverse()
        Asegment.extend(Bsegment)
        del endpoints[A], endpoints[B]  # A and B are no longer endpoints
        endpoints[Asegment[0]] = endpoints[Asegment[-1]] = Asegment
        return Asegment

    def greedy_tsp(cities):
        """Go through edges, shortest first.
        Use edge to join segments if possible."""
        endpoints = {c: [c] for c in cities}
        for (A, B) in shortest_edges_first(cities):
            if (A in endpoints and B in endpoints and
                    endpoints[A] != endpoints[B]):
                new_segment = join_endpoints(endpoints, A, B)
                if len(new_segment) == len(cities):
                    return new_segment

    # start of additional code

    # converting nodes into a list of cities
    class Node():
        def __init__(self, x, y, num):
            self.x = x
            self.y = y
            self.num = num

    City = Node
    cities = [City(nodes[0, i], nodes[1, i], i) for i in range(nodes.shape[1])]

    # apply greedy algorithm
    tour = greedy_tsp(cities)

    return tour


def routeToField(tour, start_point):
    """Converts a TSP route of nodes into waypoints to follow in a field.
    Returns a list of the points to go to. Also plots those points

    Parameters
    ----------
    tour : list of Cities
    [Node3 (3, 7), Node6 (7, 5), ...]

    start_point: [x0, y0]


    Returns
    ----------
    waypoint_list : np.array
    [[x0, x1, x2...]
     [y0, y1, y2...]]
     """

    for count, node in enumerate(tour):
        if(node.x == start_point[0] and node.y == start_point[1]):
            start = node
            start_pos = count
    waypoint = [[], []]
    for count, node in enumerate(tour):
        xi = int(tour[(start_pos + count) % len(tour)].x)
        yi = int(tour[(start_pos + count) % len(tour)].y)
        point = [[xi], [yi]]
        waypoint = np.hstack((waypoint, point))
    waypoint = np.hstack((waypoint, [[start.x], [start.y]]))
    return waypoint


# Create an example field by first defining the end points of paths
line0_x0 = 1
line0_x1 = 2
line0_y0 = 0
line0_y1 = 10
line1_x0 = 2
line1_x1 = 3
line1_y0 = 0
line1_y1 = 10
line2_x0 = 3
line2_x1 = 4
line2_y0 = 0
line2_y1 = 10

# Turn the line coordinates into a path array.
paths = np.array(
        [[[line0_x0, line0_x1],
          [line0_y0, line0_y1]],
         [[line1_x0, line1_x1],
          [line1_y0, line1_y1]],
         [[line2_x0, line2_x1],
          [line2_y0, line2_y1]]])
start = [line0_x0, line0_y0]

# Calculate tsp solution
tsp, cost_m = pathsToTSP(paths)

tour = tspToSolution1(tsp, cost_m)

waypointlist = routeToField(tour, start)

# Print the paths that make up the field
fig, ax = plt.subplots()  # Create a figure containing a single axes.
for line in paths:
    print(line)
    print("Next Line")
    # Plot my x and y values
    ax.plot(line[0, :], line[1, :], label='Path')
    ax.axis('equal')
    ax.set(xlabel='Distance North (m)', ylabel='Distance East (m)',
           title='Path')
# Print vehicle route
fig, ax1 = plt.subplots()  # Create a figure containing a single axes.
ax1.plot(waypointlist[0, :], waypointlist[1, :], '.-')
ax1.axis('equal')
ax1.set(xlabel='Distance North (m)', ylabel='Distance East (m)',
        title='Waypoints')
for i in range(np.shape(waypointlist)[1] - 1):  # all but the last point
    ax1.annotate(i, (waypointlist[0, i], waypointlist[1, i]))
# Final Point printed to the right so it is visible
final = np.shape(waypointlist)[1] - 1
ax1.annotate(final, (waypointlist[0, final], waypointlist[1, final]),
             horizontalalignment='right')