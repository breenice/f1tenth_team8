import rospy
import cv2
import numpy as np
from std_msgs.msg import Int32, String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from overtaker_config import *

class DriveModeSelector:
    def __init__(self):
        self.drive_mode_pub = rospy.Publisher('/{}/drive_mode'.format(CAR_NAME), Int32, queue_size=1)
        self.raceline_pub = rospy.Publisher('/{}/raceline'.format(CAR_NAME), String, queue_size=1)
        
        # self.map = cv2.imread('base_map.pgm', cv2.IMREAD_GRAYSCALE)
        
        self.current_pose = None
        rospy.Subscriber('/{}/particle_filter/viz/inferred_pose'.format(CAR_NAME), 
                        PoseStamped, self.pose_callback)
        
        rospy.Subscriber("/car_8/scan", LaserScan, self.scan_callback)

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
        """
        pass

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
