from math import *
import random

# the "world" has 4 landmarks.
# the robot's initial coordinates are somewhere in the square
# represented by the landmarks.
#
# NOTE: Landmark coordinates are given in (y, x)

landmarks = [[0.0, 100.0], [0.0, 0.0], [100.0, 0.0], [100.0, 100.0]]  # position of 4 landmarks
world_size = 100.0  # world is NOT cyclic. Robot is allowed to travel "out of bounds"
max_steering_angle = pi / 4  # but it is good to keep in mind the limitations of a real car.


class robot:
    def __init__(self, length=10.0):
        self.x = random.random() * world_size  # initial x position
        self.y = random.random() * world_size  # initial y position
        self.orientation = random.random() * 2.0 * pi  # initial orientation
        self.length = length  # length of robot
        self.bearing_noise = 0.0  # initialize bearing noise to zero
        self.steering_noise = 0.0  # initialize steering noise to zero
        self.distance_noise = 0.0  # initialize distance noise to zero

    def __repr__(self):
        return '[x=%.6s y=%.6s orient=%.6s]' % (str(self.x), str(self.y), str(self.orientation))

    def set(self, new_x, new_y, new_orientation):
        if new_orientation < 0 or new_orientation >= 2 * pi:
            raise ValueError, 'Orientation must be in [0..2pi]'
        self.x = float(new_x)
        self.y = float(new_y)
        self.orientation = float(new_orientation)

    def set_noise(self, new_b_noise, new_s_noise, new_d_noise):
        # makes it possible to change the noise parameters
        # this is often useful in particle filters
        self.bearing_noise = float(new_b_noise)
        self.steering_noise = float(new_s_noise)
        self.distance_noise = float(new_d_noise)

    def move(self, motion):
        [x, y, theta, l] = [self.x, self.y, self.orientation, self.length]
        forward = motion[1]  # d
        steering_angle = motion[0]  # alpha

        if steering_angle > max_steering_angle:
            raise (ValueError, "Steering angle exceeds max steering angle ")
        elif forward < 0:
            raise (ValueError, "Forward shud be positive")

        new_robot = robot(l)
        new_robot.set_noise(self.bearing_noise, self.steering_noise, self.distance_noise)

        # applying noise
        steering_angle += random.gauss(steering_angle, self.steering_noise)
        forward += random.gauss(forward, self.distance_noise)

        if steering_angle < 0.01:  # almost straight motion case
            x_new = x + forward * cos(theta)
            y_new = y + forward * sin(theta)
            theta_new = theta
        else:
            beta = forward * tan(steering_angle) / l  # turning angle
            r = l / tan(steering_angle)  # Turning radius

            x_new = x - r * sin(theta) + r * sin(theta + beta)
            y_new = y + r * cos(theta) - r * cos(theta + beta)
            theta_new = (theta + beta) % (2 * pi)

        new_robot.set(x_new, y_new, theta_new)
        return new_robot

## --------
## TEST CASE:
##
## 1) The following code should print:
##       Robot:     [x=0.0 y=0.0 orient=0.0]
##       Robot:     [x=10.0 y=0.0 orient=0.0]
##       Robot:     [x=19.861 y=1.4333 orient=0.2886]
##       Robot:     [x=39.034 y=7.1270 orient=0.2886]
##
##
# length = 20.
# bearing_noise = 0.0
# steering_noise = 0.0
# distance_noise = 0.0
#
# myrobot = robot(length)
# myrobot.set(0.0, 0.0, 0.0)
# myrobot.set_noise(bearing_noise, steering_noise, distance_noise)
#
# motions = [[0.0, 10.0], [pi / 6.0, 10], [0.0, 20.0]]
#
# T = len(motions)
#
# print 'Robot:    ', myrobot
# for t in range(T):
#    myrobot = myrobot.move(motions[t])
#    print 'Robot:    ', myrobot


# 2) The following code should print:
#      Robot:     [x=0.0 y=0.0 orient=0.0]
#      Robot:     [x=9.9828 y=0.5063 orient=0.1013]
#      Robot:     [x=19.863 y=2.0201 orient=0.2027]
#      Robot:     [x=29.539 y=4.5259 orient=0.3040]
#      Robot:     [x=38.913 y=7.9979 orient=0.4054]
#      Robot:     [x=47.887 y=12.400 orient=0.5067]
#      Robot:     [x=56.369 y=17.688 orient=0.6081]
#      Robot:     [x=64.273 y=23.807 orient=0.7094]
#      Robot:     [x=71.517 y=30.695 orient=0.8108]
#      Robot:     [x=78.027 y=38.280 orient=0.9121]
#      Robot:     [x=83.736 y=46.485 orient=1.0135]
#
# length = 20.
# bearing_noise = 0.0
# steering_noise = 0.0
# distance_noise = 0.0
#
# my_robot = robot(length)
# my_robot.set(0.0, 0.0, 0.0)
# my_robot.set_noise(bearing_noise, steering_noise, distance_noise)
#
# motions = [[0.2, 10.] for row in range(10)]
#
# T = len(motions)
#
# print 'Robot:    ', my_robot
# for t in range(T):
#     my_robot = my_robot.move(motions[t])
#     print 'Robot:    ', my_robot
