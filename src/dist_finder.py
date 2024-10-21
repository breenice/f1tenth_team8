#!/usr/bin/env python

import rospy
import math
import numpy as np
from sensor_msgs.msg import LaserScan
from race.msg import pid_input

# Some useful variable declarations.
angle_range = 240         # Hokuyo 4LX has 240 degrees FoV for scan
forward_projection = 0.2  # distance (in m) that we project the car forward for correcting the error.
desired_distance = 0.9    # desired distance from the wall (in m).
car_length = 0.50         # Traxxas Rally is 20 inches or 0.5 meters.

prev_readings = {}

# Publisher that will publish on the 'error' topic messages of type 'pid_input'
pub = rospy.Publisher('error', pid_input, queue_size=10)

def getRange(data, angle):
    """
    Retrieve the LIDAR range at a specific angle.
    """
    # Convert angle to radians
    angle_rad = math.radians(angle)
    angle_min = data.angle_min
    angle_increment = data.angle_increment
    num_ranges = len(data.ranges)

    # Calculate index
    index = int((angle_rad - angle_min) / angle_increment)
    index = max(0, min(num_ranges - 1, index))  # Ensure index is within bounds

    range_value = data.ranges[index]

    # Handle NaNs and invalid readings
    if np.isnan(range_value) or range_value == 0:
        range_value = prev_readings.get(angle, 2)  # Default to 2 meters if no previous reading
    else:
        prev_readings[angle] = range_value

    return range_value

def callback(data):
    """
    Callback function to process LIDAR data and publish errors for PID control.
    """
    global forward_projection

    thetas = [45, 60, 70, 75, 80]  # Angles to use for error calculation

    steering_error = 0
    for theta in thetas:
        a = getRange(data, theta)  # Distance at angle theta
        b = getRange(data, 0)      # Distance directly to the right
        swing = math.radians(theta)

        # Compute Alpha, AB, and CD
        numerator = a * math.cos(swing) - b
        denominator = a * math.sin(swing)
        if denominator != 0:
            alpha = math.atan2(numerator, denominator)
        else:
            alpha = 0  # Avoid division by zero

        AB = b * math.cos(alpha)
        CD = AB + forward_projection * math.sin(alpha)
        steering_error += desired_distance - CD

        # Debugging Prints
        print("Angle " + str(theta) + ": " + str(a))
        print("Angle " + str(0) + ": " + str(b))
        print("alpha: " + str(alpha))
        print("AB: " + str(AB) + ", CD: " + str(CD))    
        print("Steering Error (before averaging): " + str(steering_error))

    steering_error /= len(thetas)

    # Get distance straight ahead (90 degrees)
    dist_ahead = getRange(data, 90)
    print("Distance ahead: " + str(dist_ahead) + " meters")

    # Adjust velocity based on distance ahead using linear mapping
    min_dist = 0.5   # meters
    max_dist = 2.0   # meters
    min_speed = 15   # speed units
    max_speed = 25   # speed units
    desired_speed = 20        # desired cruising speed

    if dist_ahead <= min_dist:
        desired_vel = min_speed
        velError = desired_vel - desired_speed
        print("Close to obstacle, set velocity error to achieve minimum speed.")
    elif dist_ahead >= max_dist:
        desired_vel = max_speed
        velError = desired_vel - desired_speed
        print("Path is clear, set velocity error to achieve maximum speed.")
    else:
        # Linear interpolation between min_speed and max_speed
        desired_vel = min_speed + (dist_ahead - min_dist) * (max_speed - min_speed) / (max_dist - min_dist)
        velError = desired_vel - desired_speed
        print("Adjusting speed based on distance ahead. Desired Velocity: " + str(desired_vel))

    print("Velocity Error: " + str(velError))
    print("Steering Error (after averaging): " + str(steering_error))

    # ----------------------------------------------

    # Create and publish the pid_input message
    msg = pid_input()  # An empty msg is created of the type pid_input
    msg.pid_error = steering_error  # Steering error for PID
    msg.pid_vel = velError          # Velocity error for PID
    pub.publish(msg)

if __name__ == '__main__':
    print("Hokuyo LIDAR node started")
    rospy.init_node('dist_finder', anonymous=True)
    # Subscribe to the correct scan topic
    rospy.Subscriber("/car_8/scan", LaserScan, callback)
    rospy.spin()

'''
    # Adjust speed using if-else statement based on error magnitude
    threshold_error = 5  # Adjust this threshold value as needed
    high_speed = 25      # Speed when going straight
    low_speed = 15       # Speed when about to turn

    if abs(error) < threshold_error:
        command.speed = high_speed
        print("Going straight. Speeding up.")
    else:
        command.speed = low_speed
        print("Approaching a turn. Slowing down.")

    # Ensure speed is within bounds [0, 100]
    command.speed = max(0, min(100, command.speed))
'''
