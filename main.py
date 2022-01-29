import csv
from queue import PriorityQueue


class Node:
    # A basic node representing a state.

    def __init__(self, cost, path_cost, heuristic, parent, location):
        self.cost = cost
        self.path_cost = path_cost
        self.heuristic = heuristic
        self.parent = parent
        self.location = location


def csv_to_list(csv_file_name):
    # Converts a csv file into an n-dimensional list.

    with open(csv_file_name, newline='') as csv_file:
        reader = csv.reader(csv_file)
        output_list = list(reader)

    # Remove left-over whitespace from each element in the newly created list.
    # This is necessary because the assignment's csv format delimits elements
    # with both a comma and space, instead of a single comma.
    for i in range(0, len(output_list)):
        for j in range(0, len(output_list[i])):
            output_list[i][j] = output_list[i][j].strip()

    return output_list


def write_optimal_path(file_name, path):
    with open(file_name, "w") as text_file:
        for node in path:
            location = '(' + str(node.location[0]) + ',' + str(node.location[1]) + ')\n'
            text_file.write(location)


def write_explored_states(file_name, explored_states):
    with open(file_name, "w") as text_file:
        for state in explored_states:
            location = '(' + str(state.location[0]) + ',' + str(state.location[1]) + ')\n'
            text_file.write(location)


def get_start_location(grid):
    # Returns the start state's location as a tuple.

    for i in range(0, len(grid)):
        for j in range(0, len(grid[i])):
            if grid[i][j] == 'S':
                return i, j


def get_goal_locations(grid):
    # Returns a list containing the location of goal states.
    # The locations themselves are tuples.

    locations = []

    for i in range(0, len(grid)):
        for j in range(0, len(grid[i])):
            if grid[i][j] == 'G':
                locations.append((i, j))

    return locations


def get_manhattan_distance(point_1, point_2):
    # Returns the manhattan distance between two points.

    return abs(point_2[0] - point_1[0]) + abs(point_2[1] - point_1[1])


def get_heuristic(state_point, goal_points):
    # Returns a state's heuristic given its location on the grid.
    # The heuristic is based on the manhattan distance between a certain
    # state and the goal state closest to it.

    heuristic = -1

    # Find the closest goal to the current state using manhattan distance.
    for i in range(0, len(goal_points)):

        distance = get_manhattan_distance(state_point, goal_points[i])
        if heuristic < 0 or distance < heuristic:
            heuristic = distance

    return heuristic


def is_in_priority_queue(element, priority_queue):
    # Returns True if an element is in the specified priority_queue.
    # This helper function is necessary since Python's PriorityQueue class makes use
    # of tuples and doesn't allow for easily checking the elements being assigned a priority.

    for item in priority_queue.queue:
        if element in item:
            return True

    return False


def get_path(goal_node, explored_states):
    # Returns the path beginning from a start state to a goal state.

    path = []
    current_node = goal_node  # Begin by tracing the path backwards from the goal state.

    # Follow the path back through the parents of each state.
    while current_node is not None:
        path.append(current_node)
        current_node = current_node.parent

    path.reverse()  # Reverse the path so it is ordered from start state to goal state.
    return path


def get_adjacent(node_grid, node):
    # Returns a list of nodes adjacent to a specific node.

    adjacent_nodes = []
    row = node.location[0]
    col = node.location[1]

    # Bounds and content checking of the node's neighbours.
    # Bound and content check left.
    if row > 0 and not node_grid[row - 1][col] == 'X':
        adjacent_nodes.append(node_grid[row - 1][col])

    # Bound and content check right.
    if row < len(node_grid) - 1 and not node_grid[row + 1][col] == 'X':
        adjacent_nodes.append(node_grid[row + 1][col])

    # Bound and content check up.
    if col > 0 and not node_grid[row][col - 1] == 'X':
        adjacent_nodes.append(node_grid[row][col - 1])

    # Bound and content check down.
    if col < len(node_grid[0]) - 1 and not node_grid[row][col + 1] == 'X':
        adjacent_nodes.append(node_grid[row][col + 1])

    return adjacent_nodes


def create_node_grid(state_grid, goal_locations):
    # Given a grid of states represented by strings, returns a
    # grid of nodes with corresponding costs and locations.

    node_grid = []

    for i in range(0, len(state_grid)):
        node_row = []

        for j in range(0, len(state_grid[i])):

            if state_grid[i][j] == 'X':
                node_row.append('X')
            else:
                cost = 0
                heuristic = get_heuristic((i, j), goal_locations)

                if state_grid[i][j].isnumeric():
                    cost = int(state_grid[i][j])

                # The newly created node has the same cost (arg0) and location (arg4) as its corresponding state.
                # Its path cost (arg1) is as of yet unknown so is set to 0.
                # Its heuristic (arg2) is the manhattan distance between itself and the nearest goal state.
                # Its parent (arg3) is as of yet unknown so is set to None.
                node_row.append(Node(cost, 0, heuristic, None, (i, j)))

        node_grid.append(node_row)

    return node_grid


def search_graph(node_grid, start_location, goal_locations):

    start_node = node_grid[start_location[0]][start_location[1]]

    frontier = PriorityQueue()

    # Nodes are added to the priority queue wrapped in tuples.
    # The first element is its priority. The second element is the node itself.
    frontier.put((start_node.heuristic, start_node))

    explored = []

    while True:

        if not frontier:
            return None

        # The leaf node is obtained by accessing the second element
        # of the tuple that is returned by the priority queue.
        leaf = (frontier.get())[1]

        explored.append(leaf)

        if leaf.location in goal_locations:
            return get_path(leaf, explored), explored

        for node in get_adjacent(node_grid, leaf):

            current_path_cost = leaf.path_cost + node.cost

            if (not is_in_priority_queue(node, frontier) and node not in explored) \
                    or (is_in_priority_queue(node, frontier) and current_path_cost < node.path_cost):

                node.parent = leaf
                node.path_cost = current_path_cost
                frontier.put(((node.path_cost + node.heuristic), node))


def pathfinding(input_filename, optimal_path_filename, explored_list_filename):
    # input_filename contains a CSV file with the input grid
    # optimal_path_filename is the name of the file the optimal path should be written to
    # explored_list_filename is the name of the file the list of explored nodes should be written to

    optimal_path_cost = 0

    state_grid = csv_to_list(input_filename)

    start_location = get_start_location(state_grid)
    goal_locations = get_goal_locations(state_grid)

    node_grid = create_node_grid(state_grid, goal_locations)

    path, explored_nodes = search_graph(node_grid, start_location, goal_locations)

    for node in path:
        optimal_path_cost += node.cost

    write_optimal_path("optimal_path.txt", path)
    write_explored_states("explored_states.txt", explored_nodes)

    return optimal_path_cost


print(pathfinding("input_02.txt", "", ""))
