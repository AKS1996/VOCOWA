from math import pi, sqrt, cos, sin, pow, atan
from copy import deepcopy
from numpy import zeros, uint8, double
from random import randint

conv = pi / 180.0  # Convert to radians
MAXlandmarks = 30  # TODO Was 3k initially
MAXERROR = 0.5  # if a landmark is within 20 cm of another landmark its the same landmark
MINOBSERVATIONS = 15  # Number of times a landmark must be observed to be recognized as a landmark
LIFE = 40
MAX_RANGE = 1
MAXTRIALS = 1000  # RANSAC: max times to run algorithm
MAXSAMPLE = 10  # RANSAC: randomly select X points
MINLINEPOS = 30  # RANSAC: if less than 40 points left don't bother trying to find consensus(stop   algorithm)
RANSAC_TOLERANCE = 0.05  # RANSAC: if point is within x distance of line its part of line
RANSAC_CONSENSUS = 30  # RANSAC: at least 30 votes required to determine if a line


class landmark:
    points = 0  # landmarks (x,y) position relative to map
    lm_id = 0  # the landmarks unique ID
    life = 0  # a life counter used to determine whether to discard a landmark
    total_times_observed = 0  # the number of times we have seen landmark
    max_range = 0  # last observed max_range to landmark
    bearing = 0  # last observed bearing to landmark

    # RANSAC: Now store equation of a line
    a = 0
    b = 0
    range_error = 0  # distance from robot position to the wall we are using as a landmark(to calculate error)
    bearing_error = 0  # bearing from robot position to the wall we are using as a landmark (to calculate error)

    def __init__(self):
        self.total_times_observed = 0
        self.lm_id = -1
        self.life = LIFE
        self.points = [.0, .0]
        self.a = -1
        self.b = -1


def list_of_landmarks(n):
    my_list = []
    for i in range(n):
        my_list.append(landmark())
    return my_list


def distanceToLine(x, y, a, b):
    ao = -1.0 / a
    bo = y - ao * x
    # get intersection  between  y = ax + b and y = aox + bo

    px = (b - bo) / (ao - a)
    py = ((ao * (b - bo)) / (ao - a)) + bo

    return distance(x, y, px, py)


def distance_lm(lm1, lm2):
    return sqrt(pow(lm1.points[0] - lm2.points[0], 2) + pow(lm1.points[1] - lm2.points[1], 2))


def distance(x1, y1, x2, y2):
    return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2))


class landmarks:
    landmarkDB = []
    IDtoID = []
    DBsize = 0
    ekf_landmarks = 0
    dps = 0.5  # Degree Per Scan

    def __init__(self, dps):
        self.dps = dps
        for i in range(MAXlandmarks):
            self.landmarkDB.append(landmark())
            self.IDtoID.append([0, 0])

    def get_slam_id(self, lm_id):
        for i in range(self.ekf_landmarks):
            if self.IDtoID[i][0] == lm_id:
                return self.IDtoID[i][1]
        return -1

    def add_slam_id(self, landmark_id, slam_id):
        self.IDtoID[self.ekf_landmarks][0] = landmark_id
        self.IDtoID[self.ekf_landmarks][1] = slam_id
        self.ekf_landmarks += 1
        return 0

    def remove_bad_landmarks(self, laserdata, robot_position):
        maxrange = 0
        for i in range(len(laserdata) - 1):
            # distance further away than 8.1m we assume are failed returns
            # we get the laser data with max max_range
            if laserdata[i - 1] < 8.1 and laserdata[i + 1] < 8.1 and laserdata[i] > maxrange:
                maxrange = laserdata[i]

        maxrange = MAX_RANGE
        x_bounds = [.0, .0, .0, .0]
        y_bounds = [.0, .0, .0, .0]

        # get bounds of rectangular box to remove bad landmarks from

        x_bounds[0] = cos((1 * self.dps * conv) + (robot_position[2] * pi / 180)) * maxrange + robot_position[
            0]
        y_bounds[0] = sin((1 * self.dps * conv) + (robot_position[2] * pi / 180)) * maxrange + robot_position[
            1]
        x_bounds[1] = x_bounds[0] + cos((180 * self.dps * conv) + (robot_position[2] * pi / 180)) * maxrange
        y_bounds[1] = y_bounds[0] + sin((180 * self.dps * conv) + (robot_position[2] * pi / 180)) * maxrange
        x_bounds[2] = cos((359 * self.dps * conv) + (robot_position[2] * pi / 180)) * maxrange + robot_position[0]
        y_bounds[2] = sin((359 * self.dps * conv) + (robot_position[2] * pi / 180)) * maxrange + robot_position[1]
        x_bounds[3] = x_bounds[2] + cos((180 * self.dps * conv) + (robot_position[2] * pi / 180)) * maxrange
        y_bounds[3] = y_bounds[2] + sin((180 * self.dps * conv) + (robot_position[2] * pi / 180)) * maxrange

        # now check DB for landmarks that are within this box
        # decrease life of all landmarks in box. If the life reaches zero, remove landmark

        for k in range(self.DBsize + 1):

            pntx = self.landmarkDB[k].points[0]
            pnty = self.landmarkDB[k].points[1]
            i, j = 0, 0

            if robot_position[0] < 0 or robot_position[1] < 0:
                in_rectangle = False
            else:
                in_rectangle = True

            for i in range(4):
                if (y_bounds[i] <= pnty < y_bounds[j] or y_bounds[j] <= pnty < y_bounds[i]) and (
                            pntx < (x_bounds[j] - x_bounds[i]) * (pnty - y_bounds[i]) / (y_bounds[j] - y_bounds[i]) +
                            x_bounds[i]):
                    in_rectangle = not in_rectangle

                j = i
                i += 1

            if in_rectangle:
                # in rectangle so decrease life and maybe remove
                self.landmarkDB[k].life -= 1
                if self.landmarkDB[k].life <= 0:
                    for kk in range(k, self.DBsize):  # remove self.landmark by copying down rest of DB
                        if kk == self.DBsize - 1:
                            self.landmarkDB[kk].points[0] = self.landmarkDB[kk + 1].points[0]
                            self.landmarkDB[kk].points[1] = self.landmarkDB[kk + 1].points[1]
                            self.landmarkDB[kk].life = self.landmarkDB[kk + 1].life
                            self.landmarkDB[kk].lm_id = self.landmarkDB[kk + 1].lm_id
                            self.landmarkDB[kk].total_times_observed = self.landmarkDB[kk + 1].total_times_observed

                        else:
                            self.landmarkDB[kk + 1].lm_id -= 1
                            self.landmarkDB[kk].points[0] = self.landmarkDB[kk + 1].points[0]
                            self.landmarkDB[kk].points[1] = self.landmarkDB[kk + 1].points[1]
                            self.landmarkDB[kk].life = self.landmarkDB[kk + 1].life
                            self.landmarkDB[kk].lm_id = self.landmarkDB[kk + 1].lm_id
                            self.landmarkDB[kk].total_times_observed = self.landmarkDB[kk + 1].total_times_observed
                    self.DBsize -= 1
        return 0

    def update_landmark(self, matched, lm_id, distance, reading_no, robot_position):
        if matched:
            # EKF matched landmark so increase times landmark has been observed
            self.landmarkDB[lm_id].total_times_observed += 1
            lm = self.landmarkDB[lm_id]
        else:
            # EKF failed to match landmark so add it to DB as new landmark
            lm = landmark()
            # convert landmark to map coordinate
            lm.points[0] = cos((reading_no * self.dps * conv) + (robot_position[2] * pi / 180)) * distance
            lm.points[1] = sin((reading_no * self.dps * conv) + (robot_position[2] * pi / 180)) * distance
            lm.points[0] += robot_position[0]  # add robot position
            lm.points[1] += robot_position[1]  # add robot position
            lm.bearing = reading_no
            lm.max_range = distance
            lm_id = self.AddToDB(lm)
            lm.lm_id = lm_id

        # return landmarks
        return lm

    def update_landmark2(self, lm):
        # try to do data-association on landmark.
        lm_id = self.GetAssociation(lm)  # if we failed to associate landmark, then add it to DB
        if lm_id == -1:
            lm_id = self.AddToDB(lm)
        lm.lm_id = lm_id

        # return landmarks
        return lm

    def update_add_line_landmarks(self, extracted_landmarks):  # returns the found landmarks
        temp_landmarks = deepcopy(extracted_landmarks)
        for i in range(len(extracted_landmarks)):
            temp_landmarks[i] = self.update_landmark2(extracted_landmarks[i])
        return temp_landmarks

    def update_add_using_ekf_results(self, matched, lm_id, ranges, bearings, robot_position):
        foundlandmarks = []
        for i in range(len(matched)):
            foundlandmarks.append(landmark())
            foundlandmarks[i] = self.update_landmark(matched[i], lm_id[i], ranges[i], bearings[i], robot_position)
        return foundlandmarks

    def UpdateLineLandmark(self, lm):
        # try to do data-association on landmark.
        lm_id = self.GetAssociation(lm)
        # if we failed to associate landmark, then add it to DB

        if lm_id is -1:
            lm_id = self.AddToDB(lm)
        return lm_id

    def ExtractLinelandmarks(self, laserdata, robot_position):  # TODO Most important fn
        la = list(zeros((100, 1), dtype=double))
        lb = list(zeros((100, 1), dtype=double))
        total_lines = 0

        line_points = deepcopy(laserdata)
        total_line_points = 0

        temp_landmarks = list_of_landmarks(400)

        for i in range(len(laserdata) - 1):
            line_points[total_line_points] = i
            total_line_points += 1

        # RANSAC ALGORITHM

        no_trials = 0
        while no_trials < MAXTRIALS and total_line_points > MINLINEPOS:
            random_selected_points = list(zeros((MAXSAMPLE, 1), dtype=uint8))
            temp = 0

            # Randomly select a subset S1 of n data points and
            # compute the model M1
            # Initial version chooses entirely randomly. Now choose
            # one point randomly and then sample from neighbours within some defined
            # radius
            center_point = randint(MAXSAMPLE, total_line_points - 1)
            random_selected_points[0] = center_point

            for i in range(1, MAXSAMPLE):
                new_point = False

                while not new_point:
                    temp = (center_point + (randint(0, 3) - 1) * randint(0, MAXSAMPLE)) % len(laserdata)
                    # TODO check me later, 10**6 -> randint(0,2)
                    for j in range(i):
                        if random_selected_points[j] == temp:
                            break  # point has already been selected
                        if j >= i - 1:
                            new_point = True  # point has not already been selected

                random_selected_points[i] = temp

            # compute model M1: y = a+ bx

            a, b = self.LeastSquaresLineEstimate(laserdata, robot_position, random_selected_points, MAXSAMPLE)

            # Determine the consensus set S1* of points is P
            # compatible with M1 (within some error tolerance)

            consensus_points = deepcopy(laserdata)
            total_consensus_points = 0

            new_line_points = deepcopy(laserdata)
            total_new_line_points = 0

            for i in range(total_line_points):

                # convert ranges and bearing to coordinates
                x = (cos((line_points[i] * self.dps * conv) + robot_position[2] * conv) *
                     laserdata[line_points[i]]) + robot_position[0]
                y = (sin((line_points[i] * self.dps * conv) + robot_position[2] * conv) *
                     laserdata[line_points[i]]) + robot_position[1]
                # x =(cos((line_points[i] * dps * conv)) * laserdata[line_points[i]])  +robot_position[0]
                # y =(sin((line_points[i] * dps * conv)) *   laserdata[line_points[i]]) +robot_position[1]

                d = distanceToLine(x, y, a, b)

                if d < RANSAC_TOLERANCE:
                    # add points which are close to line
                    consensus_points[total_consensus_points] = line_points[i]
                    total_consensus_points += 1
                else:
                    # add points which are not close to line
                    new_line_points[total_new_line_points] = line_points[i]
                    total_new_line_points += 1

            # If #(S1*) > t, use S1* to compute (maybe using least squares) a new model M1*

            if total_consensus_points > RANSAC_CONSENSUS:

                # Calculate updated line equation based on consensus points
                a, b = self.LeastSquaresLineEstimate(laserdata, robot_position, consensus_points,
                                                     total_consensus_points)

                # for now add points associated to line as landmarks to see results
                for i in range(total_consensus_points):
                    # Remove points that have now been associated to this line
                    line_points = deepcopy(new_line_points)
                    total_line_points = total_new_line_points

                # add line to found lines
                la[total_lines] = a
                lb[total_lines] = b
                total_lines += 1
                # restart search since we found a line
                # no_trials = MAXTRIALS #when maxtrials = debugging
                no_trials = 0

            else:
                # DEBUG add point that we chose as middle value
                # temp_landmarks[centerPo] = GetLandmark(laserdata[centerPo], centerPo,robot_position)
                # If #(S1*) < t, randomly select another subset S2 and repeat
                # If, after some predetermined number of trials there is
                # no consensus set with t points, return with failure
                no_trials += 1

        # for each line we found:
        # calculate the point on line closest to origin (0,0)
        # add this point as a landmark
        for i in range(total_lines):
            temp_landmarks[i] = self.GetLineLandmark(la[i], lb[i], robot_position)
            # temp_landmarks[i+1] = GetLine(la[i], lb[i])

        # for debug add origin as landmark
        # temp_landmarks[total_lines+1] = GetOrigin()
        # temp_landmarks[i] = GetLandmark(laserdata[i], i, robot_position)
        # now return found landmarks in an array of correct dimensions

        foundlandmarks = []
        for i in range(total_lines):
            foundlandmarks.append(temp_landmarks[i])

        return foundlandmarks

    def LeastSquaresLineEstimate(self, laserdata, robot_position, selected_points, array_size):

        sum_y, sum_yy = 0, 0  # sum of y^2 for each coordinate
        sum_x, sum_xx = 0, 0
        sum_yx = 0  # sum of y * x for each point

        for i in range(array_size):
            # convert ranges and bearing to coordinates
            x = (cos((selected_points[i] * self.dps * conv) + robot_position[2] * conv) *
                 laserdata[selected_points[i]]) + robot_position[0]
            y = (sin((selected_points[i] * self.dps * conv) + robot_position[2] * conv) *
                 laserdata[selected_points[i]]) + robot_position[1]

            sum_y += y
            sum_yy += pow(y, 2)
            sum_x += x
            sum_xx += pow(x, 2)
            sum_yx += y * x

        b = (sum_y * sum_xx - sum_x * sum_yx) / (array_size * sum_xx - pow(sum_x, 2))
        a = (array_size * sum_yx - sum_x * sum_y) / (array_size * sum_xx - pow(sum_x, 2))

        return a, b

    def ExtractSpikelandmarks(self, laserdata, robot_position):  # have a large array to keep track of found landmarks
        temp_landmarks = []
        for i in range(400):
            temp_landmarks.append(landmark())

        for i in range(1, len(laserdata) - 1):
            # Check for error measurement in laser data
            if laserdata[i - 1] < 8.1 and laserdata[i + 1] < 8.1 and (laserdata[i - 1] - laserdata[i]) + (
                        laserdata[i + 1] - laserdata[i]) > 0.5:
                temp_landmarks[i] = self.GetLandmark(laserdata[i], i, robot_position)
            else:
                if (laserdata[i - 1] - laserdata[i]) > 0.3:
                    temp_landmarks[i] = self.GetLandmark(laserdata[i], i, robot_position)
                elif laserdata[i + 1] < 8.1 and (laserdata[i + 1] - laserdata[i]) > 0.3:
                    temp_landmarks[i] = self.GetLandmark(laserdata[i], i, robot_position)
                    # get total found landmarks so you can return array of correct dimensions

        # now return found landmarks in an array of correct dimensions
        foundlandmarks = []
        for i in range(len(temp_landmarks)):
            if temp_landmarks[i].lm_id is not -1:
                foundlandmarks.append(temp_landmarks[i])
        return foundlandmarks

    def GetLandmark(self, max_range, reading_no, robot_position):
        lm = landmark()
        # convert landmark to map coordinate
        lm.points[0] = cos((reading_no * self.dps * conv) + (robot_position[2] * pi / 180)) * max_range
        lm.points[1] = sin((reading_no * self.dps * conv) + (robot_position[2] * pi / 180)) * max_range
        lm.points[0] += robot_position[0]  # add robot position
        lm.points[1] += robot_position[1]  # add robot position
        lm.max_range = max_range
        lm.bearing = reading_no

        # associate landmark to closest landmark.
        total_times_observed = 0
        lm_id, total_times_observed = self.GetClosestAssociation(lm, total_times_observed)
        lm.lm_id = lm_id
        # return landmarks
        return lm

    def GetLineLandmark(self, a, b, robot_position):

        # our goal is to calculate point on line closest to origin (0,0)
        # calculate line perpendicular to input line. a*ao = -1
        ao = -1.0 / a
        # landmark position
        x = b / (ao - a)
        y = (ao * b) / (ao - a)
        max_range = sqrt(pow(x - robot_position[0], 2) + pow(y - robot_position[1], 2))
        bearing = atan((y - robot_position[1]) / (x - robot_position[0])) - robot_position[2]
        # now do same calculation but get point on wall closest to robot instead
        # y = aox + bo => bo = y - aox
        bo = robot_position[1] - ao * robot_position[0]  # get intersection between y = ax + b and y = aox + bo
        # so aox + bo = ax + b => aox - ax = b - bo => x = (b - bo)/(ao - a), y = ao*(b - bo)/(ao - a) + bo

        px = (b - bo) / (ao - a)
        py = ((ao * (b - bo)) / (ao - a)) + bo

        range_error = distance(robot_position[0], robot_position[1], px, py)
        bearing_error = atan((py - robot_position[1]) / (px - robot_position[0])) - robot_position[
            2]  # do you subtract or add robot bearing? I am not sure!

        lm = landmark()

        # convert landmark to map coordinate
        lm.points[0] = x
        lm.points[1] = y
        lm.max_range = max_range
        lm.bearing = bearing
        lm.a = a
        lm.b = b
        lm.range_error = range_error
        lm.bearing_error = bearing_error
        # associate landmark to closest landmark.

        total_times_observed = 0

        lm_id, total_times_observed = self.GetClosestAssociation(lm, total_times_observed)
        lm.lm_id = lm_id
        lm.total_times_observed = total_times_observed
        # return landmarks
        return lm

    def GetLine(self, a, b):

        # our goal is to calculate point on line closest to origin (0,0)
        # calculate line perpendicular to input line. a*ao = -1
        ao = -1.0 / a
        # get intersection between y = ax + b and y = aox
        # so aox = ax + b => aox - ax = b => x = b/(ao - a), y = ao*b/(ao - a)
        x = b / (ao - a)
        y = (ao * b) / (ao - a)

        lm = landmark()

        # convert landmark to map coordinate
        lm.points[0] = x
        lm.points[1] = y
        lm.max_range = -1
        lm.bearing = -1
        lm.a = a
        lm.b = b
        # associate landmark to closest landmark.

        total_times_observed = 0
        lm_id, total_times_observed = self.GetClosestAssociation(lm, total_times_observed)
        lm.lm_id = lm_id
        # return landmarks
        return lm

    def GetOrigin(self):
        lm = landmark()
        # convert landmark to map coordinate
        lm.points[0] = 0
        lm.points[1] = 0
        lm.max_range = -1
        lm.bearing = -1
        # associate landmark to closest landmark.

        total_times_observed = 0
        lm_id, total_times_observed = self.GetClosestAssociation(lm, total_times_observed)
        lm.lm_id = lm_id
        # return landmarks
        return lm

    def GetClosestAssociation(self, lm, total_times_observed):

        # given a landmark we find the closest landmark in DB
        closest_landmark = 0
        leastdistance = 99999  # 99999m is least initial distance, its big
        for i in range(self.DBsize):
            # only associate to landmarks we have seen more than MINOBSERVATIONS times
            if self.landmarkDB[i].total_times_observed > MINOBSERVATIONS:
                temp = distance_lm(lm, self.landmarkDB[i])
                if temp < leastdistance:
                    leastdistance = temp
                    closest_landmark = self.landmarkDB[i].lm_id

        if leastdistance == 99999:
            lm_id = -1
        else:
            lm_id = self.landmarkDB[closest_landmark].lm_id
            total_times_observed = self.landmarkDB[closest_landmark].total_times_observed

        return lm_id, total_times_observed

    def GetAssociation(self, lm):  # this method needs to be improved so we use innovation as a validation gate
        # currently we just check if a landmark is within some predetermined distance of a landmark in DB
        for i in range(self.DBsize):
            if distance_lm(lm, self.landmarkDB[i]) < MAXERROR and self.landmarkDB[i].lm_id is not -1:
                self.landmarkDB[i].life = LIFE  # landmark seen so reset its life counter
                self.landmarkDB[i].total_times_observed += 1  # increase number of times we seen landmark

                self.landmarkDB[i].bearing = lm.bearing  # set last bearing seen at
                self.landmarkDB[i].max_range = lm.max_range  # set last max_range seen at
                return self.landmarkDB[i].lm_id

        return -1

    def RemoveDoubles(self, extracted_landmarks):

        uniquelmrks = 0
        leastdistance = 99999

        uniquelandmarks = list_of_landmarks(100)
        for i in range(len(extracted_landmarks)):

            # remove landmarks that didn't get associated and also pass
            # landmarks through our temporary landmark validation gate
            if extracted_landmarks[i].lm_id is not -1 and self.GetAssociation(extracted_landmarks[i]) is not -1:

                leastdistance = 99999
                # remove s in extracted_landmarks
                # if two observations match same landmark, take closest landmark
                for j in range(len(extracted_landmarks)):
                    if extracted_landmarks[i].lm_id == extracted_landmarks[j].lm_id:
                        if j < i:
                            break
                        temp = distance_lm(extracted_landmarks[j], self.landmarkDB[extracted_landmarks[j].lm_id])
                        if temp < leastdistance:
                            leastdistance = temp
                            uniquelandmarks[uniquelmrks] = extracted_landmarks[j]

            if not leastdistance == 99999:
                uniquelmrks += 1

        # copy landmarks over o an array of correct dimensions
        extracted_landmarks = []
        for i in range(uniquelmrks):
            extracted_landmarks.append(uniquelandmarks[i])

        return extracted_landmarks

    def AlignLandmarkData(self, extracted_landmarks, matched, lm_id, ranges, bearings, lmrks, exlmrks):
        uniquelmrks = 0
        leastdistance = 99999
        uniquelandmarks = list_of_landmarks(100)

        for i in range(len(extracted_landmarks)):
            if extracted_landmarks[i].lm_id != -1:
                leastdistance = 99999
                # remove s in extracted_landmarks
                # if two observations match same landmark, take closest landmark
                for j in range(len(extracted_landmarks)):
                    if extracted_landmarks[i].lm_id == extracted_landmarks[j].lm_id:
                        if j < i:
                            break
                        temp = distance_lm(extracted_landmarks[j], self.landmarkDB[extracted_landmarks[j].lm_id])
                        if temp < leastdistance:
                            leastdistance = temp
                            uniquelandmarks[uniquelmrks] = extracted_landmarks[j]

            if leastdistance != 99999:
                uniquelmrks += 1

        for i in range(uniquelmrks):
            matched[i] = True
            lm_id[i] = uniquelandmarks[i].lm_id
            ranges[i] = uniquelandmarks[i].max_range
            bearings[i] = uniquelandmarks[i].bearing
            lmrks[i, 0] = self.landmarkDB[uniquelandmarks[i].lm_id].points[0]
            lmrks[i, 1] = self.landmarkDB[uniquelandmarks[i].lm_id].points[1]
            exlmrks[i, 0] = uniquelandmarks[i].points[0]
            exlmrks[i, 1] = uniquelandmarks[i].points[1]

        return 0

    def AddToDB(self, lm):
        if self.DBsize + 1 < len(self.landmarkDB):
            self.landmarkDB[self.DBsize].points[0] = lm.points[0]  # set landmark coordinates
            self.landmarkDB[self.DBsize].points[1] = lm.points[1]  # set landmark coordinates
            self.landmarkDB[self.DBsize].life = LIFE  # set landmark life counter
            self.landmarkDB[self.DBsize].lm_id = self.DBsize  # set landmark lm_id
            self.landmarkDB[self.DBsize].total_times_observed = 1  # initialise number of times we've seen landmark
            self.landmarkDB[self.DBsize].bearing = lm.bearing  # set last bearing was seen at
            self.landmarkDB[self.DBsize].max_range = lm.max_range  # set last max_range was seen at
            self.landmarkDB[self.DBsize].a = lm.a  # store landmarks wall equation
            self.landmarkDB[self.DBsize].b = lm.b  # store landmarks wall equation
            self.DBsize += 1

            return self.DBsize - 1

        return -1

    def GetDBsize(self):
        return self.DBsize

    def GetDB(self):
        temp = list_of_landmarks(self.DBsize)
        for i in range(self.DBsize):
            temp[i] = self.landmarkDB[i]
        return temp


def main():
    """
        For Typical laser data, see dummies pg 14
        Starts Reading from robot[2] degrees to Cartesian X Axis in anticlockwise sense

    """
    data = []

    for i in range(90):
        data.append(1 / (sin(i * conv) + cos(i * conv)))

    for i in range(90):
        data.append(1 / (sin(i * conv) + cos(i * conv)))

    robot = (0, 0, 0)  # X,Y, theta
    ls = landmarks(1)
    nls = ls.ExtractLinelandmarks(data, robot)
    print nls


if __name__ == '__main__':
    main()
