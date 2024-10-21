#!/usr/bin/env python

import rospy
import math
import numpy as np
from sensor_msgs.msg import LaserScan
from race.msg import pid_input

# Some useful variable declarations.
angle_range = 240	# Hokuyo 4LX has 240 degrees FoV for scan
forward_projection = 1	# distance (in m) that we project the car forward for correcting the error. You have to adjust this.
desired_distance = 1.1 # distance from the wall (in m). (defaults to right wall). You need to change this for the track
vel = 15 		# this vel variable is not really used here.
error = 0.0		# initialize the error
car_length = 0.50 # Traxxas Rally is 20 inches or 0.5 meters. Useful variable.

prev_readings = {}

# Publisher that will publish on the 'error' topic messages of type 'pid_input'
pub = rospy.Publisher('error', pid_input, queue_size=10)

def getRange(data, angle):
    # data: single message from topic /scan
    # angle: between -30 to 210 degrees, where 0 degrees is directly to the right, and 90 degrees is directly in front
    # Outputs length in meters to object with angle in lidar scan field of view
    # Make sure to take care of NaNs etc.
     #TODO: implement

    #  if not -30 <= angle <= 210:
    #     raise ValueError "Angle " + str(angle) + " out of bounds"

     # Convert to radians because LaserScan is in radians
    angle_rad = math.radians(angle)

    angle_min = -(data.angle_min % math.pi)
    index = int((angle_rad - angle_min) / data.angle_increment)
    
    range_value = data.ranges[index]

    print(angle_rad, angle_min, data.angle_max, data.angle_increment)

    if np.isnan(range_value):
        range_value = prev_readings.get(angle, 2)
    else:
        prev_readings[angle] = range_value
    
    return range_value


def callback(data):
    """
    Callback function to process LIDAR data and publish errors for PID control.
    """
    global forward_projection

    thetas = [45, 60, 65, 70, 72, 75, 80]  # Angles to use for error calculation

    error = 0
    for theta in thetas:
        a = getRange(data,theta) # obtain the ray distance for theta
        b = getRange(data,0)	# obtain the ray distance for 0 degrees (i.e. directly to the right of the car)
        swing = math.radians(theta)

        # ----------------------------------------------
        
        ## Your code goes here to determine the projected error as per the alrorithm
        # Compute Alpha, AB, and CD..and finally the error.
        alpha = math.atan((a * math.cos(swing) - b)/ (a * math.sin(swing)))
        AB = b * math.cos(alpha)

        CD = AB + (forward_projection) * math.sin(alpha) # forward projection is AC
        # try w/ (forward_projection + car_length) * math.sin(alpha) if bad results
        error += desired_distance - CD
    error /= len(thetas)
    # Get distance straight ahead (90 degrees)
    dist_ahead = getRange(data, 90)
    print("Distance ahead: " + str(dist_ahead) + " meters")

    # Adjust speed using if-else statement based on error magnitude
    threshold_error = 2  # Adjust this threshold value as needed
    high_speed = 30      # Speed when going straight
    low_speed = 15       # Speed when about to turn

    if dist_ahead > threshold_error:
        vel_error = high_speed
        print("Going straight. Speeding up.")
    else:
        vel_error = low_speed
        print("Approaching a turn. Slowing down.")

    # Ensure speed is within bounds [0, 100]
    vel_error = max(0, min(100, vel_error))


    # Adjust velocity based on distance ahead using linear mapping
    # min_dist = 0.5   # meters
    # max_dist = 2.0   # meters
    # min_speed = 15   # speed units
    # max_speed = 40   # speed units
    # desired_speed = 20        # desired cruising speed

    # if dist_ahead <= min_dist:
    #     desired_vel = min_speed
    #     velError = desired_vel - desired_speed
    #     print("Close to obstacle, set velocity error to achieve minimum speed.")
    # elif dist_ahead >= max_dist:
    #     desired_vel = max_speed
    #     velError = desired_vel - desired_speed
    #     print("Path is clear, set velocity error to achieve maximum speed.")
    # else:
    #     # Linear interpolation between min_speed and max_speed
    #     desired_vel = min_speed + (dist_ahead - min_dist) * (max_speed - min_speed) / (max_dist - min_dist)
    #     velError = desired_vel - desired_speed
    #     print("Adjusting speed based on distance ahead. Desired Velocity: " + str(desired_vel))

    # print("Velocity Error: " + str(velError))
    # print("Steering Error (after averaging): " + str(error))

    

    # ----------------------------------------------

    # Create and publish the pid_input message
    msg = pid_input()  # An empty msg is created of the type pid_input
    msg.pid_error = error  # Steering error for PID
    msg.pid_vel = vel_error          # Velocity error for PID
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
