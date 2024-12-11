import cv2
import rospy
import numpy as np
import math
from std_msgs.msg import Int32, String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from overtaker_config import *
from ftg_config import MAXIMUM_SPEED 

class DriveModeSelector:

    # set mode
    def __init__(self):
        self.drive_mode_pub = rospy.Publisher('/{}/drive_mode'.format(CAR_NAME), Int32, queue_size=1)
        self.raceline_pub = rospy.Publisher('/{}/select_raceline'.format(CAR_NAME), String, queue_size=1)
        
        self.map = cv2.imread('base_map.pgm', cv2.IMREAD_GRAYSCALE)
        
        self.current_pose = None
        rospy.Subscriber('/{}/particle_filter/viz/inferred_pose'.format(CAR_NAME), 
                        PoseStamped, self.pose_callback)
        
        # rospy.Subscriber("/car_8/scan", LaserScan, self.scan_callback)
        self.wall_threshold = 10

        # added disparity extender and gap finder
        # self.disparity_extender = DisparityExtender(DISPARITY_DISTANCE, SAFETY_EXTENSION, MAX_LIDAR_DISTANCE)
        # self.gap_finder = GapFinder(GAP_SELECTION, POINT_SELECTION, MIN_GAP_SIZE, MIN_GAP_DISTANCE, CORNERING_DISTANCE)


    def pose_callback(self, msg):
        self.current_pose = msg.pose

    # def scan_callback(self, scan):
    #     """
    #     TODO: use disparity extender to ID obstacles & gap finder. pick between racelines based on
    #     1. larger gaps to cetner 2. deeper gaps to the outer raceline 3. smaller gaps to the inner raceline 
    #     4. no gaps = cc
        
    #     """
    #     #NEED TO ADD PARAMETERs FOR GAP FINDING AND CHANGE THIS IF NEEDED

    #     if self.current_pose is None:
    #         return

    #     # convert scan to ranges array and handle NaNs
    #     ranges = np.array(scan.ranges)
    #     ranges = np.where(np.isnan(ranges), MAX_LIDAR_DISTANCE, ranges)
    #     ranges[ranges < 0.05] = MAX_LIDAR_DISTANCE

    #     # process scan with disparity extender
    #     disparity_extender = DisparityExtender(DISPARITY_DISTANCE, SAFETY_EXTENSION, MAX_LIDAR_DISTANCE)
    #     processed_ranges = disparity_extender.extend_disparities(ranges, scan.angle_increment)
        
    #     # find gaps
    #     gap_finder = GapFinder(GAP_SELECTION, POINT_SELECTION, MIN_GAP_SIZE, MIN_GAP_DISTANCE, CORNERING_DISTANCE)
    #     gap_finder.update_data(processed_ranges, scan)
        
    #     try:
    #         # largest gap
    #         start_i, end_i = gap_finder.get_gap()
    #         gap_width = end_i - start_i
    #         gap_depth = min(processed_ranges[start_i:end_i])
            
            # which raceline to use based on gap
            if gap_width > MIN_GAP_SIZE * 2:  # large gap - use center
                self.set_raceline('mindist')
            elif gap_depth > MAX_LIDAR_DISTANCE * 0.8:  # deep gap - use outer
                self.set_raceline('mindist_boundry')
            elif gap_width > MIN_GAP_SIZE:  # smaller gap - use inner
                self.set_raceline('mincurve')
            else:  # cc
                self.set_mode_cc()
                
        except:
            # if gap finding fails, switch to cc
            self.set_mode_cc()

    def set_mode_stop(self):
        self.drive_mode_pub.publish(DriveMode.STOP)
        
    def set_mode_cc(self):
        #maintain a distance of 1.5 meters from the closest object ahead.
        
        if self.current_pose is None:
            return
        
        closest_index = self.get_closest_index() # cloestes point index
        closest_distance = self.get_distance_to_object(closest_index)  

        if closest_distance < CC_DISTANCE: 
            speed = self.dynamic_velocity(closest_distance) 
        else:
            speed = MAXIMUM_SPEED 
        
        self.drive_mode_pub.publish(DriveMode.CC)

    def set_mode_ftg(self):
        self.drive_mode_pub.publish(DriveMode.FTG)

    def set_mode_pp(self):
        self.drive_mode_pub.publish(DriveMode.PP)

    def set_raceline(self, raceline):
        if raceline not in RACELINES:
            raise ValueError("Invalid raceline")
        self.set_mode_pp()
        self.raceline_pub.publish(raceline)
