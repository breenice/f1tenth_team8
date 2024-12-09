import os
import csv
import rospy
from geometry_msgs.msg import Point32
from geometry_msgs.msg import PolygonStamped
from overtaker_config import *

PATH_FOLDER = '/home/volta/depend_ws/src/F1tenth_car_workspace/wallfollow/src/final_race'

class RacelineMerchant:
    _instance = None
    _cache = {}

    def __new__(cls):
        if cls._instance is None:
            cls._instance = super(RacelineMerchant, cls).__new__(cls)
        return cls._instance

    def __init__(self):
        if not hasattr(self, 'initialized'):
            self.initialized = True
            self.plan = []
            self.raceline_pub = rospy.Publisher('/{}/raceline'.format(CAR_NAME), PolygonStamped, queue_size=1)

    def construct_path(self, trajectory_name):
        """
        Function to construct the path from a CSV file. Uses caching to avoid re-reading files.
        """
        if trajectory_name in self._cache:
            self.plan = self._cache[trajectory_name]
            self.publish_raceline()
            return

        self.plan = []
        
        file_path = os.path.expanduser(
            '{}/{}.csv'.format(PATH_FOLDER, trajectory_name))

        with open(file_path) as csv_file:
            csv_reader = csv.reader(csv_file, delimiter=',')
            for waypoint in csv_reader:
                self.plan.append(waypoint)

        # Convert string coordinates to floats
        for index in range(0, len(self.plan)):
            for point in range(0, len(self.plan[index])):
                self.plan[index][point] = float(self.plan[index][point])
                
        # Cache the processed path
        self._cache[trajectory_name] = self.plan[:]

        # Publish the new raceline
        self.publish_raceline()

    def publish_raceline(self):
        """
        Publishes the current raceline as a polygon for visualization in RViz
        """
        polygon = PolygonStamped()
        polygon.header.stamp = rospy.Time.now()
        polygon.header.frame_id = "map"

        for point in self.plan:
            p = Point32()
            p.x = point[0]
            p.y = point[1]
            p.z = 0.0
            polygon.polygon.points.append(p)

        for _ in range(3):
            self.raceline_pub.publish(polygon)
            rospy.sleep(0.1)
