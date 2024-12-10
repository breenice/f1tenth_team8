import cv2
import rospy
import numpy as np
import math
from std_msgs.msg import Int32, String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from overtaker_config import *

class DriveModeSelector:

    # set mode
    def __init__(self):
        self.drive_mode_pub = rospy.Publisher('/{}/drive_mode'.format(CAR_NAME), Int32, queue_size=1)
        self.raceline_pub = rospy.Publisher('/{}/select_raceline'.format(CAR_NAME), String, queue_size=1)
        
        self.map = cv2.imread('base_map.pgm', cv2.IMREAD_GRAYSCALE)
        
        self.current_pose = None
        rospy.Subscriber('/{}/particle_filter/viz/inferred_pose'.format(CAR_NAME), 
                        PoseStamped, self.pose_callback)
        
        rospy.Subscriber("/car_8/scan", LaserScan, self.scan_callback)
        self.wall_threshold = 10

    def pose_callback(self, msg):
        self.current_pose = msg.pose

    def scan_callback(self, scan):
        """
        TODO: Based on scan and pose, determine what coordinates have obstacles
            This can be done by checking if the points on scan + pose are close to walls on the map
            ie. if the distance between the point and the nearest wall is greater than some threshold, then it is an obstacle
        Based on these coordinates...
        Choose a drive mode
        Or, iterate over racelines and choose raceline that does not have obstacle
        
        logic to switch in here
        """

        ## this most likely wrong lel 


        
        if self.current_pose is None:
            return

        return
        # Convert scan data to cartesian coordinates relative to car
        angle_min = -(scan.angle_min % math.pi)
        angles = np.arange(angle_min, scan.angle_max + scan.angle_increment, scan.angle_increment)
        xs = scan.ranges * np.cos(angles)
        ys = scan.ranges * np.sin(angles)
        points = np.vstack((xs, ys))

        # Filter out invalid readings
        print("Min Scan Range: ", scan.range_min)
        print("Max Scan Range: ", scan.range_max)
        valid_mask = np.logical_and(scan.ranges > scan.range_min, scan.ranges < scan.range_max)
        points = points[:, valid_mask]

        obstacles = []
        for point in points:
            map_point = self.world_to_map(point)
            if self.close_to_wall(map_point):
                obstacles.append(point)

        for raceline in RACELINES:
            pass
            # Check each raceline and choose one

    def set_mode_stop(self):
        self.drive_mode_pub.publish(DriveMode.STOP)

    def set_mode_ftg(self):
        self.drive_mode_pub.publish(DriveMode.FTG)

    def set_mode_pp(self):
        self.drive_mode_pub.publish(DriveMode.PP)

    def set_raceline(self, raceline):
        if raceline not in RACELINES:
            raise ValueError("Invalid raceline")
        self.set_mode_pp()
        self.raceline_pub.publish(raceline)
