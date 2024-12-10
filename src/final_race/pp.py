#!/usr/bin/env python

import math
from pp_config import *
from raceline_merchant import RacelineMerchant

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
        self.get_lookahead_distance()
        self.get_lookahead_point()
        # test = self.get_steering_angle()
        # angle = abs(test)
        # #dynamic lookahead
        # self.lookahead_distance = MIN_LOOK_AHEAD_DISTANCE + (((100-angle) / 100) * (MAX_LOOK_AHEAD_DISTANCE - MIN_LOOK_AHEAD_DISTANCE))
        

        
        #if angle >= 90:
            #self.lookahead_distance = MAX_LOOK_AHEAD_DISTANCE
        #else:
            #self.lookahead_distance = MIN_LOOK_AHEAD_DISTANCE

        # print("lookahead:")
        # print(self.lookahead_distance)
        # print("angle: ")
        # print(angle)

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
        print("IDX", base_projection_index)
        self.base_proj = (self.plan[base_projection_index][0], self.plan[base_projection_index][1])
        self.base_proj_index = base_projection_index

    def get_lookahead_distance(self):
        """
        Set the lookahead distance based on curvature of the path ahead. Starting from self.base_projection_index,
        go along the path for LOOKAHEAD_METERS and calculate the total curvature of the path.
        """
        total_curvature = 0
        current_index = self.base_proj_index
        distance_covered = 0
        
        while current_index < len(self.plan) - 2 and distance_covered < self.velo_lookahead_distance:
            # Get three consecutive points to calculate curvature
            p1 = self.plan[current_index]
            p2 = self.plan[current_index + 1]
            p3 = self.plan[current_index + 2]
            
            # Calculate vectors between points
            v1 = (p2[0] - p1[0], p2[1] - p1[1])
            v2 = (p3[0] - p2[0], p3[1] - p2[1])
            
            # Calculate angle between vectors
            dot_product = v1[0]*v2[0] + v1[1]*v2[1]
            v1_mag = math.sqrt(v1[0]**2 + v1[1]**2)
            v2_mag = math.sqrt(v2[0]**2 + v2[1]**2)
            
            # Avoid division by zero
            if v1_mag * v2_mag == 0:
                angle = 0
            else:
                angle = math.acos(max(-1, min(1, dot_product/(v1_mag * v2_mag))))
            
            total_curvature += abs(angle)
            distance_covered += self._get_distance(p1, p2)
            current_index += 1
            
        # Adjust lookahead distance based on curvature
        curvature_factor = min(1, total_curvature / math.pi)
        self.lookahead_distance = MIN_LOOK_AHEAD_DISTANCE + curvature_factor * (MAX_LOOK_AHEAD_DISTANCE - MIN_LOOK_AHEAD_DISTANCE)
        print(total_curvature, curvature_factor, self.lookahead_distance)


    def get_lookahead_point(self):
        """
        Follow path starting from base_projection and get first point that is lookahead_distance away
        """
        # TODO 3: Utilizing the base projection found in TODO 1, your next task is to identify the goal or target point for the car.
        target_index = (self.base_proj_index + 10) % len(self.plan)
        # loop through waypoints along our path until target point is at least lookahead_distance away and we havent
        # reached end
        # while target_index < len(self.plan) - 1:
        #     if self._get_distance(self.odom, self.plan[target_index]) >= self.lookahead_distance:
        #         break
        #     target_index += 1

        # TODO: Make this interpolate if target point is beyond lookahead distance
        # target coordiantes for where we want the car to go to next bc they're just far enough past lookahead dist
        # without overshooting
        self.target = (self.plan[target_index-1][0], self.plan[target_index-1][1])

    def get_steering_angle(self):
        """
        Compute the steering angle given the pose of the car, target point, and lookahead distance
        """
        alpha = math.atan2(self.target[1] - self.odom[1] ,  self.target[0] - self.odom[0]) - self.heading
        steering_angle = math.atan2(2 * WHEELBASE_LEN * math.sin(alpha), self.lookahead_distance)
        steering_angle = math.degrees(steering_angle)
        # print("Alpha:", alpha, "Steering Angle:", steering_angle)
        return steering_angle
    
    def get_dynamic_velo(self, steering_angle):
        target_index = (self.base_proj_index + 20) % len(self.plan)
        vel_target = (self.plan[target_index-1][0], self.plan[target_index-1][1])

        distance = self._get_distance(self.base_proj, vel_target) - 1.5

        return max(30, distance * 25)

        if not self.speed_plan:
            # Dynamic speed from ftg algo
            abs_steering_angle = abs(steering_angle)
            if abs_steering_angle >= 100:
                return MIN_SPEED
            else:
                return MAX_SPEED - ((abs_steering_angle / 100.0) * (MAX_SPEED - MIN_SPEED))
        else:
            intended_speed = self.speed_plan[self.base_proj_index]
            return min(70, intended_speed * 12) 

            x1, y1 = self.min_plan_speed, MIN_SPEED
            x2, y2 = self.max_plan_speed, MAX_SPEED
            x_new = intended_speed
            return y1 + (y2 - y1) / (x2 - x1) * (x_new - x1)

    def _get_distance(self, p1, p2):
        return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)
