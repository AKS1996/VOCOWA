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
        min_value = -1
        changed = False
        for d in delta:
            x = open[0][0] + d[0]
            y = open[0][1] + d[1]
            if 0 <= x < len(grid) and 0 <= y < len(grid[0]):
                if values[x][y] == -1:
                    open.append([x, y])
                elif values[x][y] < min_value:
                    min_value = values[x][y]
                    changed = True
        if changed:
            values[open[0][0]][open[0][1]] = min_value + cost
        else:
            values[open[0][0]][open[0][1]] = 0
        open.remove(open[0])

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
