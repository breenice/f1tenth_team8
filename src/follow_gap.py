#!/usr/bin/env python

import rospy
import numpy as np
import math
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDrive

### Notes from lecture - can delete later 
# Step 1: Convert LIDAR data to a numpy array and handle NaNs
# Step 2: Identify disparities 
# Step 3: Apply safety bubble 
# Step 4: Filter the valid front-facing data (-90 to +90 degrees)
# Step 5: Calculate the steering angle towards the gap’s center
# Step 6: Publish the AckermannDrive message with dynamic speed control


# def __init__(self):
#     # topics 
#     lidar_topic = '/car_8/scan'
#     drive_topic = '/car_8/offboard/command'

#     # parameters 
#     self.safety_radius = 0.5 
#     self.gap_detection_threshold = 0.1 
#     self.max_lidar_distance = 10.0 
#     self.velocity = 15
#     self.turn_speed = 5.0


#     # ROS publisher and subscriber
#     self.drive_pub = rospy.Publisher(drive_topic, AckermannDrive, queue_size=10)
#     rospy.Subscriber(lidar_topic, LaserScan, self.lidar_callback)

# parameters 
SAFETY_RADIUS = 0.5 
GAP_DETECTION_THRESHOLD = 0.1  
MAX_LIDAR_DISTANCE = 10.0 
DEFAULT_VELOCITY = 15.0
TURN_SPEED = 5.0 

# ROS publisher
drive_pub = None

def lidar_callback(data):
    """
    callback function to process LIDAR data
    - identify disparities and applies safety measures
    - search for the best gap to navigate
    - publish steering and velocity commands based on the identified gap
    """
    # preprocess LIDAR data by converting LIDAR data to numpy array and handle NaNs (replace NaNs w/ max range)
    ranges = np.array(data.ranges) 
    ranges = np.where(np.isnan(ranges), MAX_LIDAR_DISTANCE, ranges)

    # apply safety bubble around closest obstacle 
    closest_idx = np.argmin(ranges) # index of closest point
    safety_bubble(ranges, closest_idx, data.angle_increment)

    # find largest gap and select the best point from that gap
    start_i, end_i = find_largest_gap(ranges)
    best_point = find_best_point(start_i, end_i, ranges)

    # calculate steering angle towards best point
    steering_angle = (best_point - len(ranges) // 2) * data.angle_increment

    # publish 
    publish_drive(steering_angle)


def safety_bubble(ranges, closest_idx, angle_increment):
    """
    apply a safety bubble around disparities to avoid collisions
    """
    # calculate the start and end indices for the safety bubble
    start = max(0, closest_idx - int(SAFETY_RADIUS / angle_increment))
    end = min(len(ranges), closest_idx + int(SAFETY_RADIUS / angle_increment))
    
    # mark all points within the safety bubble as obstacles (set them to 0)
    ranges[start:end] = 0


def find_largest_gap(ranges):
    """
    find widest gap
    """
    # create a mask for free space points (non-zero values)
    free_space = ranges > 0
    # split gaps 
    gaps = np.split(np.arange(len(ranges)), np.where(np.diff(free_space) != 0)[0] + 1)
    
    # filter out small gaps based on threshold
    valid_gaps = [gap for gap in gaps if len(gap) * ranges[0] > GAP_DETECTION_THRESHOLD]

    if not valid_gaps:
        rospy.logwarn("No valid gaps detected")
        return 0, len(ranges) - 1  # default to full range if no valid gaps
    
    # find largest gap based on its length, then return start and end indices of the largest gap
    largest_gap = max(valid_gaps, key=len)
    return largest_gap[0], largest_gap[-1]


def find_best_point(start_i, end_i, ranges):
    """
    pick furthest point in the identified gap ^ from there 
    """
     # return index of the best point
    return np.argmax(ranges[start_i:end_i + 1]) + start_i

def publish_drive(steering_angle):
    """
    publish to AckermannDrive with our calculated steering angle 
    """
    # create a new AckermannDrive message
    command = AckermannDrive()
    # clamp steering angle between [-90, 90] degrees
    command.steering_angle = max(min(steering_angle, math.radians(90)), math.radians(-90))
    # set speed based on steering angle
    command.speed = dynamic_velocity(steering_angle)
    # publish drive command 
    drive_pub.publish(command)


def dynamic_velocity(steering_angle):
    """
    adjust speed based on the distance to the nearest obstacle
    """
    # if turn is sharp (> 30 degrees) then slow down, if not keep default speed 
    if abs(steering_angle) > math.radians(30):
        return 5.0
    else:
        return DEFAULT_VELOCITY 


    # uhhh attempt to slow speed graudally but not actually sure if it'l work
    # # calculate absolute value of the steering angle to determine severity of the turn
    # abs_angle = abs(steering_angle)

    # # gradually slow speed as the steering angle increases (sharp turn)
    # speed = np.interp(abs_angle, [0, math.radians(90)], [DEFAULT_VELOCITY, TURN_SPEED])

    # return speed


if __name__=='__main__':
    """
    init ros node and keep running 
    """
    rospy.init_node('follow_gap', anonymous=True)
    rospy.spin()
    
