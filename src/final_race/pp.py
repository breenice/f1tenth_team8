#!/usr/bin/env python

import rospy
import math
from pp_config import *

from raceline_merchant import RacelineMerchant
from overtaker_config import *

from std_msgs.msg import Int32

WHEELBASE_LEN = 0.325


class PurePursuit:
    def __init__(self, lookahead_distance, velo_lookahead_distance, min_speed, max_speed):
        self.lookahead_distance = lookahead_distance
        self.velo_lookahead_distance = velo_lookahead_distance
        self.min_speed = min_speed
        self.max_speed = max_speed

        self.raceline_merchant = RacelineMerchant()
        self.plan = []
        self.speed_plan = []

        self.odom = None
        self.base_proj = None
        self.base_proj_index = None
        self.target = None

        self.heading = None

        self.min_plan_speed = None
        self.max_plan_speed = None

        self.sector = None
        rospy.Subscriber('/{}/sector'.format(CAR_NAME), Int32, self.set_sector)

    def set_sector(self, data):
        self.sector = data.data

    def construct_path(self, trajectory_name):
        """
        Function to construct the path from a CSV file
        """
        self.raceline_merchant.construct_path(trajectory_name)
        self.plan = self.raceline_merchant.plan
        self.speed_plan = self.raceline_merchant.speed_plan
        if self.speed_plan:
            self.min_plan_speed = min(self.speed_plan)
            self.max_plan_speed = max(self.speed_plan)

    def pure_pursuit(self, odom_x, odom_y, heading):
        self.odom = (odom_x, odom_y)
        self.heading = heading

        self.get_base_projection()
        self.get_lookahead_point()
        # test = self.get_steering_angle()
        # angle = abs(test)
        # #dynamic lookahead
        # self.lookahead_distance = MIN_LOOK_AHEAD_DISTANCE + (((100-angle) / 100) * (MAX_LOOK_AHEAD_DISTANCE - MIN_LOOK_AHEAD_DISTANCE))
        angle = self.get_steering_angle()
        return angle

    def get_base_projection(self):
        """
        Find closest point on the reference path to the car's current position
        """
        min_dist = float('inf')
        base_projection_index = 0  # index to store waypoint on the path
        for index, point in enumerate(self.plan):
            distance = self._get_distance(self.odom, point)
            if distance < min_dist:
                min_dist = distance
                base_projection_index = index
        # get the coordinates for base projection
        # print("IDX", base_projection_index)
        self.base_proj = (self.plan[base_projection_index][0], self.plan[base_projection_index][1])
        self.base_proj_index = base_projection_index

    def get_lookahead_point(self):
        """
        Follow path starting from base_projection and get first point that is lookahead_distance away
        """
        lookahead_indices = int(self.speed_plan[self.base_proj_index] * 1.8) # 8
        add_lookahead = 1
        if self.sector == Sectors.FREE:
            add_lookahead = 12
        elif self.sector == Sectors.MID:
            add_lookahead = 8
        elif self.sector == Sectors.DANGER:
            add_lookahead = 6
        target_index = (self.base_proj_index + add_lookahead) % len(self.plan)
        self.target = (self.plan[target_index - 1][0], self.plan[target_index - 1][1])
        return 0

        lookahead_indices = int(self.speed_plan[self.base_proj_index] * 1.8) # 8
        target_index = (self.base_proj_index + lookahead_indices) % len(self.plan)
        self.target = (self.plan[target_index - 1][0], self.plan[target_index - 1][1])
        return


        target_index = self.base_proj_index
        # loop through waypoints along our path until target point is at least lookahead_distance away and we havent
        # reached end
        while target_index < len(self.plan) - 1:
            if self._get_distance(self.odom, self.plan[target_index]) >= self.lookahead_distance:
                break
            target_index += 1

        # target coordiantes for where we want the car to go to next bc they're just far enough past lookahead dist
        # without overshooting
        self.target = (self.plan[target_index - 1][0], self.plan[target_index - 1][1])

    def get_curvature(self):
        # TODO use heading
        curv_len = 4
        point_base = self.heading
        point_1 = self.plan[(self.base_proj_index + curv_len + curv_len)%len(self.plan)]
        point_2 = self.plan[(self.base_proj_index + (2*curv_len) + curv_len)%len(self.plan)]
        x1,y1 = point_base
        x2,y2 = point_1
        x3,y3 = point_2

        det = 2 * ((x1 - x2) * (y2 - y3) - (x2 - x3) * (y1 - y2)) # determinant
        if det == 0:
            return 0
        
        center_x = ((x1**2 + y1**2) * (y2 - y3) + (x2**2 + y2**2) * (y3-y1)+(x3**2 + y3**2) * (y1-y2))/det
        center_y = ((x1**2 + y1**2) * (x3 - x2) + (x2**2 + y2**2) * (x1-x3)+(x3**2 + y3**2) * (x2-x1))/det

        radius = ((center_x - x1)**2 + (center_y-y1)**2)**0.5
        curvature = 1/radius

        return curvature

    def get_steering_angle(self):
        """
        Compute the steering angle given the pose of the car, target point, and lookahead distance
        """

        alpha = math.atan2(self.target[1] - self.odom[1], self.target[0] - self.odom[0]) - self.heading
        steering_angle = math.atan2(2 * WHEELBASE_LEN * math.sin(alpha), self.lookahead_distance)
        steering_angle = math.degrees(steering_angle)
        # print("Alpha:", alpha, "Steering Angle:", steering_angle)

        if self.sector == Sectors.FREE:
            return self.velo
        elif self.sector == Sectors.MID:
            return self.velo * 0.8
        elif self.sector == Sectors.DANGER:
            return self.velo * 0.6
        return 0
    
        return steering_angle

    def get_dynamic_velo(self, steering_angle):
        # dynamic lookahead based on curvature

        # curv = abs(self.get_curvature())
        # curv_speed = 1/(curv * CURVE_MULT)
        # velocity = min(MAX_SPEED, 50 * curv_speed)
        # return velocity

        # # Dynamic speed based on further lookahead point
        target_index = (self.base_proj_index + 20) % len(self.plan)
        vel_target = (self.plan[target_index - 1][0], self.plan[target_index - 1][1])

        distance = self._get_distance(self.base_proj, vel_target) - 1

        self.velo = max(15, distance * 20)

        if self.sector == Sectors.FREE:
            return self.velo
        elif self.sector == Sectors.MID:
            return self.velo * 0.8
        elif self.sector == Sectors.DANGER:
            return self.velo * 0.6
        return 0

        if not self.speed_plan:
            # Dynamic speed from ftg algo
            abs_steering_angle = abs(steering_angle)
            if abs_steering_angle >= 100:
                return MIN_SPEED
            else:
                return MAX_SPEED - ((abs_steering_angle / 100.0) * (MAX_SPEED - MIN_SPEED))
        else:
            # Trying dynamic speed based on raceline
            intended_speed = self.speed_plan[self.base_proj_index]
            return min(70, intended_speed * 10)
        
            # Trying dynamic speed based on raceline with interpolation
            x1, y1 = self.min_plan_speed, MIN_SPEED
            x2, y2 = self.max_plan_speed, MAX_SPEED
            x_new = intended_speed
            return y1 + (y2 - y1) / (x2 - x1) * (x_new - x1)

    def _get_distance(self, p1, p2):
        return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)
