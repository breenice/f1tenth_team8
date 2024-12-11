import cv2
import rospy
import numpy as np
import math
from std_msgs.msg import Int32, String, Float32
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from overtaker_config import *
from ftg_config import MAX_LIDAR_DISTANCE

from ftg_raceline_control import FTGRacelineControl

class DriveModeSelector:

    # set mode
    def __init__(self):
        self.drive_mode_pub = rospy.Publisher('/{}/drive_mode'.format(CAR_NAME), Int32, queue_size=1)
        self.raceline_pub = rospy.Publisher('/{}/select_raceline'.format(CAR_NAME), String, queue_size=1)
        self.cc_pub = rospy.Publisher('/{}/cruise_mult'.format(CAR_NAME), Float32, queue_size=1)
        
        self.map = cv2.imread('base_map.pgm', cv2.IMREAD_GRAYSCALE)
        
        self.current_pose = None
        rospy.Subscriber('/{}/particle_filter/viz/inferred_pose'.format(CAR_NAME), 
                        PoseStamped, self.pose_callback)
        
        rospy.Subscriber("/car_8/scan", LaserScan, self.scan_callback)

        self.ftg_raceline_control = FTGRacelineControl()


    def pose_callback(self, msg):
        self.current_pose = msg.pose

    def scan_callback(self, scan):
        """
        TODO: use disparity extender to ID obstacles & gap finder. pick between racelines based on
        1. larger gaps to cetner 2. deeper gaps to the outer raceline 3. smaller gaps to the inner raceline 
        4. no gaps = cc
        
        """
        #NEED TO ADD PARAMETERs FOR GAP FINDING AND CHANGE THIS IF NEEDED

        if self.current_pose is None:
            return

        raceline = self.ftg_raceline_control.ftg_control(scan)
        print("Choosing raceline: ", raceline)

        if raceline:
            self.set_raceline(raceline)
        else:
            self.set_raceline(RACELINES_IN_ORDER[0])
            self.set_mode_cc(scan)

    def set_mode_stop(self):
        self.drive_mode_pub.publish(DriveMode.STOP)
        
    def set_mode_cc(self, data):
        # maintain a distance of 1.5 meters from the closest object ahead.        
        closest_distance = self.get_closest_object(data) # closest point (in front)

        if closest_distance < CC_DISTANCE: 
            self.cc_pub.publish(0.5)
        else:
            self.cc_pub.publish(1.0)
        
        self.drive_mode_pub.publish(DriveMode.CC)

    def get_closest_object(self, data):
        # get the index of the closest object in the scan between 30 and 150 degrees
        angle_min = -(data.angle_min % math.pi)
        ranges = np.array(data.ranges)
        
        # Convert angles to indices
        angle_30 = int((math.radians(30) - angle_min) / data.angle_increment)
        angle_150 = int((math.radians(150) - angle_min) / data.angle_increment)
        
        # Get ranges between 30 and 150 degrees
        front_ranges = ranges[angle_30:angle_150]
        
        # Replace inf, nan and small values with max range
        front_ranges = np.where(np.isnan(front_ranges), MAX_LIDAR_DISTANCE, front_ranges)
        front_ranges = np.where(np.isinf(front_ranges), MAX_LIDAR_DISTANCE, front_ranges)
        front_ranges[front_ranges < 0.05] = MAX_LIDAR_DISTANCE
        
        # Return minimum distance
        return np.min(front_ranges)

    def set_mode_pp(self):
        self.drive_mode_pub.publish(DriveMode.PP)

    def set_raceline(self, raceline):
        if raceline not in RACELINES:
            raise ValueError("Invalid raceline")
        self.set_mode_pp()
        self.raceline_pub.publish(raceline)
