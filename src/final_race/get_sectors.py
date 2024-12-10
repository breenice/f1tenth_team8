import rospy
import os
import csv
import math
import overtaker_config
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDrive
from std_msgs.msg import Int32, String

from multi_pp_control import MultiPPControl
# from ftg_control import FTGControl
from overtaker_config import *
PATH_FOLDER = '/home/volta/depend_ws/src/F1tenth_car_workspace/wallfollow/src/final_race/racelines'

class GetSectors():
    def __init__(self):
        rospy.Subscriber('/{}/particle_filter/viz/inferred_pose'.format(CAR_NAME), PoseStamped,self.mpp_control)
        self.command_pub = rospy.Publisher('/{}/drive_mode'.format(CAR_NAME), Int32, queue_size=1)

        self.plan = self.construct_path()

    def construct_path(self):
        """
        Function to construct the path from a CSV file
        """
        self.raceline_merchant.construct_path("mindist")
        return self.raceline_merchant.plan

    def mpp_control(self, data):
        self.pose = data.pose

    def get_sector(self):
        self.get_closest_idx()

        file_path = os.path.expanduser(
            '{}/{}.csv'.format(PATH_FOLDER, "sectors"))
        
        sec_names = {
            "FREE": Sectors.FREE,
            "MID": Sectors.MID,
            "DANGER": Sectors.DANGER
        }

        with open(file_path) as csv_file:
            csv_reader = csv.reader(csv_file, delimiter=',')
            for start, sector in csv_reader:
                if self.closest_idx >= int(start):
                    mapped_type = string_to_type["int"]
                    val = mapped_type(sector) 
                    self.sector = val

    
    def get_closest_idx(self):
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
        self.base_proj = (self.plan[base_projection_index][0], self.plan[base_projection_index][1])
        self.closest_idx = base_projection_index

    def _get_distance(self, p1, p2):
        return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)

    def publish_command(self):
        command = Sectors()
        command = self.sector
        self.command_pub.publish(command)