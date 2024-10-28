#!/usr/bin/env python

import rospy
import numpy as np
import math
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDrive
from race.msg import pid_input


### Notes from lecture - can delete later 
# Step 1: Convert LIDAR data to a numpy array and handle NaNs
# Step 2: Identify disparities 
# Step 3: Apply safety bubble 
# Step 4: Filter the valid front-facing data (-90 to +90 degrees)
# Step 5: Calculate the steering angle towards the gap’s center
# Step 6: Publish the AckermannDrive message with dynamic speed control


def __init__(self):
    return 0

def lidar_callback(self, data):
    """
    callback function to process LIDAR data
    - identify disparities and applies safety measures
    - search for the best gap to navigate
    - publish steering and velocity commands based on the identified gap
    """
    return 0

def apply_safety_bubble(self, ranges, disparities, angle_increment):
    """
    apply a safety bubble around disparities to avoid collisions
    """

def find_best_gap(self, ranges, data):
    """
    find widest gap
    """

def calculate_steering_angle(self, best_gap_idx, angle_increment):
    """
    calculate steering angle towards the identified gap's center
    """

def publish_drive(self, angle, distance):
    """
    """

def dynamic_velocity(self, distance):
    """
    adjust speed based on the distance to the nearest obstacle
    """

if __name__=='__main__':
    """
    """
