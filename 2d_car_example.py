# You are given a car in grid with initial state
# init. Your task is to compute and return the car's
# optimal path to the position specified in goal;
# the costs for each motion are as defined in cost.
#
# There are four motion directions: up, left, down, and right.
# Increasing the index in this array corresponds to making a
# a left turn, and decreasing the index corresponds to making a
# right turn.

forward = [[-1, 0],  # go up
           [0, -1],  # go left
           [1, 0],  # go down
           [0, 1]]  # go right
forward_name = ['up', 'left', 'down', 'right']

# action has 3 values: right turn, no turn, left turn
action = [-1, 0, 1]
action_name = ['R', '#', 'L']

# EXAMPLE INPUTS:
# grid format:
#     0 = navigable space
#     1 = unnavigable space
grid = [[1, 1, 1, 0, 0, 0],
        [1, 1, 1, 0, 1, 0],
        [0, 0, 0, 0, 0, 0],
        [1, 1, 1, 0, 1, 1],
        [1, 1, 1, 0, 1, 1]]

init = [4, 3, 0]  # given in the form [row,col,direction]
# direction = 0: up
#             1: left
#             2: down
#             3: right

goal = [2, 0]  # given in the form [row,col]

cost = [2, 1, 20]  # cost has 3 values, corresponding to making


# a right turn, no turn, and a left turn

# EXAMPLE OUTPUT:
# calling optimum_policy2D with the given parameters should return
# [[' ', ' ', ' ', 'R', '#', 'R'],
#  [' ', ' ', ' ', '#', ' ', '#'],
#  ['*', '#', '#', '#', '#', 'R'],
#  [' ', ' ', ' ', '#', ' ', ' '],
#  [' ', ' ', ' ', '#', ' ', ' ']]


def getValue(dx, dy, heading):
    lookup_x = [[-1, 2, 1, 0],
                [1, 0, -1, 2]]
    lookup_y = [[0, -1, 2, 1],
                [0, 1, 2, -1]]
    if dx == 0:
        if dy == 1:
            return lookup_y[0][heading]
        elif dy == -1:
            return lookup_y[1][heading]
    elif dy == 0:
        if dx == 1:
            return lookup_x[0][heading]
        elif dx == -1:
            return lookup_x[1][heading]


def optimum_policy2D(grid, init, goal, cost):
    values = [[-1 for i in grid[0]] for i in grid]

    # blocking non navigable spaces
    for i in range(len(grid)):
        for j in range(len(grid[0])):
            if grid[i][j] == 1:
                values[i][j] = 99

    unvisited = [init]

    while unvisited:
        [i, j] = [unvisited[0][0], unvisited[0][1]]

        # creating set of visited neighbour cells
        visited = []
        for d in forward:
            [x, y] = [i + d[0], j + d[1]]
            if 0 <= x < len(grid) and 0 <= y < len(grid[0]) and grid[x][y] != 1:
                if values[x][y] == -1:
                    unvisited.append([x, y])
                else:
                    visited.append([values[x][y], x, y])

        # assign new values
        if visited:
            visited.sort()
            [last_value, x, y] = visited[0]
            dx = x - i
            dy = y - j

            values[i][j] = getValue(dx, dy, unvisited[0][2]) + last_value

        unvisited.remove(unvisited[0])

    return values

for i in optimum_policy2D(grid, init, goal, cost):
    print i
