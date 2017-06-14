# Creating Occupancy grid using LIDAR sensor
# Its non cascading model i.e. dimensions of actual map will be almost equal to resultant
# That's actually a poor way - more size, slower computation

# TODO Stopping condition
# TODO Increase cell size of resultant map
# TODO Set botL = botW = 60 and edit move_bot() to move to centroid of unvisited cells
# TODO resolve addition of x,y to coordinates at specific position
# TODO Add the grey area: 1/3 of bot length

from numpy import array, add, subtract
from os import listdir
from PIL import Image
from pygame_arena import colors
from merge_images import merge_BW
import pygame
import pygame.locals
from math import sin, cos, radians

specs = (500, 3, 1, 1, 300, 5)
# specs[0] - Frequency of LIDAR in Hz
# specs[1] - Time taken for servo motor to complete 1 revolution
# specs[2] - Bot Length (Y Axis, using standard cartesian Coordinates)
# specs[3] - Bot Width (X Axis)
# specs[4] - Max Distance LIDAR can measure, in cm. Turn it 40k cm later
# specs[5] - Min Distance for bot to turn back from an obstacle


# --------- Global variables start------------------
actual_map = []
all_coordinates = []  # Points which are detected as obstacle or Out Of Sensor Bounds
OOB = []  # Out of Bound Area
bot = []  # bot's coordinates
direction = -1


# --------- Global variables end ------------------


# Turns 'jpg' to actual_map array
# Initializes x,y,result_map to some values
def init():
    global actual_map, bot
    im = Image.open('map.jpg')
    actual_map = array(im)

    # TODO Starting Point Issue
    bot.append((actual_map.shape[1] / 2, actual_map.shape[0] / 2))


# --------------------------
#   Decides motion of the bot
# --------------------------
def move_bot():
    global actual_map, direction, bot, specs, direction
    (x, y) = bot[-1]

    # given code will update bot's position
    # note that i'm going in anticlockwise sense using 4 complex roots of 1 viz. 1,-1,i,-i
    delta = 1
    while delta < specs[4]:
        if specs[5] <= x < actual_map.shape[1] - specs[5] and specs[5] <= y < actual_map.shape[0] - specs[5] \
                and 0 == actual_map[y][x]:
            x += 1 * int(direction.real)
            y += 1 * int(direction.imag)
            delta += 1
        else:
            direction *= 1j
            break

    if specs[5] > x:
        x = specs[5]
    elif x >= actual_map.shape[1] - specs[5]:
        x = actual_map.shape[1] - specs[5] - 1

    if specs[5] > y:
        y = specs[5]
    elif y >= actual_map.shape[0] - specs[5]:
        y = actual_map.shape[0] - specs[5] - 1
    bot.append((x, y))


def get_readings():
    global bot, actual_map, all_coordinates, OOB
    n = specs[0] * specs[1]
    (x, y) = (int(bot[-1][0]), int(bot[-1][1]))

    for i in range(1, n):
        r = 1
        (_x, _y) = (x - int(cos(radians(i * 360 / n)) * r), y - int(sin(radians(i * 360 / n)) * r))

        while 0 <= _x < actual_map.shape[1] and 0 <= _y < actual_map.shape[0] and r < specs[4]:
            if actual_map[_y][_x] != 0:
                if (_x, _y) not in all_coordinates:  # to avoid same point being detected again in same session
                    all_coordinates.append((_x, _y))
                break
            r += 1
            (_x, _y) = (x - int(cos(radians(i * 360 / n)) * r), y - int(sin(radians(i * 360 / n)) * r))

        # TODO Out Of Bounds Issue
        # This is the situation when sensor's reading reach the max value
        (_x, _y) = (x - int(cos(radians(i * 360 / n)) * (r - 1)), y - int(sin(radians(i * 360 / n)) * (r - 1)))
        if r == specs[4]:
            OOB.append((_x, _y))
        if (_x, _y) not in all_coordinates:
            all_coordinates.append((_x, _y))


def update_image():
    global all_coordinates, OOB, bot, actual_map

    pygame.init()
    screen = pygame.display.set_mode((2 * specs[4], 2 * specs[4]))
    screen.fill(colors['BLACK'])

    for i in range(0, len(all_coordinates)):
        # TODO Image Merge Issue
        all_coordinates[i] = add(all_coordinates[i], subtract((specs[4], specs[4]), bot[-1]))
    pygame.draw.polygon(screen, colors['WHITE'], all_coordinates)

    for i in range(0, len(OOB)):
        # TODO Image Merge Issue
        OOB[i] = add(OOB[i], subtract((specs[4], specs[4]), bot[-1]))
        # TODO Grey Area Issue
        pygame.draw.circle(screen, colors['LGREY'], OOB[i], specs[5])

    pygame.image.save(screen, 'temp.jpg')
    pygame.quit()

    if len(bot) > 1:
        shift = tuple(subtract(bot[-2], bot[-1]))
        merge_BW('temp.jpg', 'result.jpg', shift)
    else:
        Image.open('temp.jpg').convert('L').save('result.jpg')

    all_coordinates = []  # this is important else u'll be using points of previous readings again
    OOB = []


# -------------------
#   Prints result map
# -------------------
def finish():
    if 'result.jpg' in listdir('.'):
        # TODO Invert Color Issue
        # ImageOps.invert(Image.open('result.jpg')).show()
        Image.open('result.jpg').show()


def main():
    init()
    start = raw_input("Press 'y' to start: ").startswith('y')

    while start:
        print "started scanning at ", bot[-1]
        get_readings()
        update_image()
        move_bot()
        start = raw_input("\nPress 'y' to continue scanning: ").startswith('y')

    finish()


if __name__ == '__main__':
    main()
