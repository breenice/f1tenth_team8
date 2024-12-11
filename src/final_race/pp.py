#!/usr/bin/env python

import rospy
import math
from pp_config import *

from raceline_merchant import RacelineMerchant
from overtaker_config import *

from std_msgs.msg import Int32

WHEELBASE_LEN = 0.325


class PurePursuit:
    def __init__(self, lookahead_distance, min_speed, max_speed):
        self.lookahead_distance = lookahead_distance
        self.lookahead_distance_index = 0
        self.velo_lookahead_distance_index = 0
        self.sector_velo_mult = 0
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

        self.sector = None
        rospy.Subscriber('/{}/sector'.format(CAR_NAME), Int32, self.set_sector)

    def set_sector(self, data):
        self.sector = data.data

        if self.sector == Sectors.FREE:
            self.lookahead_distance_index = FREE_LOOKAHEAD
            self.velo_lookahead_distance_index = 2 * FREE_LOOKAHEAD
            self.sector_velo_mult = FREE_VELO_MULT
        elif self.sector == Sectors.MID:
            self.lookahead_distance_index = MID_LOOKAHEAD
            self.velo_lookahead_distance_index = 2 * MID_LOOKAHEAD
            self.sector_velo_mult = MID_VELO_MULT
        elif self.sector == Sectors.DANGER:
            self.lookahead_distance_index = DANGER_LOOKAHEAD
            self.velo_lookahead_distance_index = 2 * DANGER_LOOKAHEAD
            self.sector_velo_mult = DANGER_VELO_MULT

    def construct_path(self, trajectory_name):
        """
        Function to construct the path from a CSV file
        """
        self.raceline_merchant.construct_path(trajectory_name)
        self.plan = self.raceline_merchant.plan
        self.speed_plan = self.raceline_merchant.speed_plan

    def pure_pursuit(self, odom_x, odom_y, heading):
        self.odom = (odom_x, odom_y)
        self.heading = heading

        self.get_base_projection()
        self.get_lookahead_point()
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
        target_index = (self.base_proj_index + self.lookahead_distance_index) % len(self.plan)
        self.target = (self.plan[target_index][0], self.plan[target_index][1])
        return 0

    def get_steering_angle(self):
        """
        Compute the steering angle given the pose of the car, target point, and lookahead distance
        """

        alpha = math.atan2(self.target[1] - self.odom[1], self.target[0] - self.odom[0]) - self.heading
        steering_angle = math.atan2(2 * WHEELBASE_LEN * math.sin(alpha), self.lookahead_distance)
        steering_angle = math.degrees(steering_angle)
        # print("Alpha:", alpha, "Steering Angle:", steering_angle)
        return steering_angle

    def get_dynamic_velo(self):
        # # Dynamic speed based on further lookahead point
        target_index = (self.base_proj_index + self.velo_lookahead_distance_index) % len(self.plan)
        vel_target = (self.plan[target_index - 1][0], self.plan[target_index - 1][1])

        distance = self._get_distance(self.base_proj, vel_target)

        velo = distance * VELO_MULT * self.sector_velo_mult
        velo = max(self.min_speed, min(self.max_speed, velo))
        return velo

    def _get_distance(self, p1, p2):
        return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)
