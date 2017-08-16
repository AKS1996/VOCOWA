import pygame
import pygame.locals
from random import uniform
from math import sin, cos, radians

colors = {'BLACK': (0, 0, 0),
          'LGREY': (172, 172, 172),
          'WHITE': (255, 255, 255)
          }

specs = (200, 1, 300, 5)
"""
specs[0] - Frequency of LIDAR in Hz
specs[1] - Time taken for servo motor to complete 1 revolution
specs[2] - Max Distance LIDAR can measure, in cm. Turn it 40k cm later
specs[3] - Grey Area - Min Distance for bot to turn back from an obstacle
"""

all_points = []
OOB = []


def f():
    values = []
    n = specs[0] * specs[1]
    for i in range(n):
        values.append(uniform(0.5 * specs[2], 2 * specs[2]))

    return values


def get_readings():
    n = specs[0] * specs[1]

    readings = f()

    if len(readings) == n:
        for i in range(n):
            theta = radians(i * 360 / n)
            if readings[i] <= specs[2]:
                (_x, _y) = (specs[2] - int(cos(theta) * readings[i]), specs[2] - int(sin(theta) * readings[i]))
            else:
                (_x, _y) = (int(specs[2] * (1 - cos(theta))), int(specs[2] * (1 - sin(theta))))
                OOB.append((_x, _y))

            all_points.append((_x, _y))
    else:
        raise ValueError("Size doesn't matches")


def update_image():
    pygame.init()
    screen = pygame.display.set_mode((2 * specs[2], 2 * specs[2]))
    screen.fill(colors['BLACK'])

    pygame.draw.polygon(screen, colors['WHITE'], all_points)

    for i in range(0, len(OOB)):
        pygame.draw.circle(screen, colors['LGREY'], OOB[i], specs[3])

    clock = pygame.time.Clock()
    pygame.display.update()

    done = False
    while not done:
        clock.tick(20)
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                done = True
        pygame.display.flip()  # to update screen. This must happen after all commands
    pygame.quit()


if __name__ == '__main__':
    get_readings()
    update_image()
