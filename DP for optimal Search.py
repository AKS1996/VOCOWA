delta = [[-1, 0],  # go up
         [0, -1],  # go left
         [1, 0],  # go down
         [0, 1]]  # go right
delta_name = ['^', '<', 'v', '>']


def compute_value(grid, goal, cost):
    # @return - grid of values
    # Value of a cell is min no of moves required from that cell to goal.
    values = [[-1 for i in grid[0]] for i in grid]
    open = [goal]

    for i in range(len(grid)):
        for j in range(len(grid[0])):
            if grid[i][j] == 1:
                values[i][j] = 99

    while open:
        [curr_x, curr_y] = [open[0][0], open[0][1]]
        open.remove(open[0])

        # creating set of neighbours
        neighbours = []
        for d in delta:
            x = curr_x + d[0]
            y = curr_y + d[1]
            if 0 <= x < len(grid) and 0 <= y < len(grid[0]):
                neighbours.append([values[x][y], x, y])

        # separating them into valid(with some positive value) and new
        new = []
        valid = []
        for n in neighbours:
            if n[0] == -1:
                new.append([n[1], n[2]])
            elif n[0] != 99:
                valid.append(n)
        del neighbours

        # setting value of the current cell
        if valid:
            valid.sort()
            values[curr_x][curr_y] = valid[0][0] + 1
        else:
            values[curr_x][curr_y] = 0

        # appending new to list of open cells
        for n in new:
            open.append(n)

    # What if 'values' has unreachable cells?
    for i in range(len(values)):
        for j in range(len(values[0])):
            if values[i][j] == -1:
                values[i][j] = 99

    return values


def main():
    grid = [[0, 1, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0]]
    goal = [len(grid) - 1, len(grid[0]) - 1]
    cost = 1  # the cost associated with moving from a cell to an adjacent one
    # print compute_value(grid, goal, cost)
    for i in compute_value(grid, goal, cost):
        print i


if __name__ == '__main__':
    main()
