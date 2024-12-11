import cv2
import rospy
import numpy as np
import math
from std_msgs.msg import Int32, String, Float32
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from overtaker_config import *

# Map metadata from the YAML file
origin_x, origin_y = -6.977912, -3.423147  # Origin of the map
resolution = 0.025000  # Resolution of the map


class DriveModeSelector:

    # set mode
    def __init__(self):
        self.drive_mode_pub = rospy.Publisher('/{}/drive_mode'.format(CAR_NAME), Int32, queue_size=1)
        self.raceline_pub = rospy.Publisher('/{}/select_raceline'.format(CAR_NAME), String, queue_size=1)
        self.cc_pub = rospy.Publisher('/{}/cruise_mult'.format(CAR_NAME), Float32, queue_size=1)
        self.obstacle_pub = rospy.Publisher('/{}/obstacles'.format(CAR_NAME), MarkerArray, queue_size=1)
        
        self.map = cv2.imread('base_map.pgm', cv2.IMREAD_GRAYSCALE)
        
        self.current_pose = None
        rospy.Subscriber('/{}/particle_filter/viz/inferred_pose'.format(CAR_NAME), 
                        PoseStamped, self.pose_callback)
        
        self.obstacle_points = []
        rospy.Subscriber("/car_8/scan", LaserScan, self.scan_callback)


    def pose_callback(self, msg):
        self.current_pose = msg.pose

    def scan_callback(self, scan):
        if self.current_pose is None:
            return
        
        x_r, y_r = self.current_pose.position.x, self.current_pose.position.y
        theta_r = self.current_pose.orientation.z

        angle_min = -(self.data.angle_min % math.pi)

        global_points = []
        for i in range(len(scan.ranges)):
            angle = angle_min + i * scan.angle_increment
            x_local = scan.ranges[i] * math.cos(angle)
            y_local = scan.ranges[i] * math.sin(angle)
        
            x_global = x_r + x_local * math.cos(theta_r) - y_local * math.sin(theta_r)
            y_global = y_r + x_local * math.sin(theta_r) + y_local * math.cos(theta_r)

            global_points.append((x_global, y_global))

        obstacle_points = []
        for px, py in global_points:
            map_x = int((px - origin_x) / resolution)
            map_y = int((py - origin_y) / resolution)
            if self.is_wall(map_x, map_y):
                obstacle_points.append((px, py))

        self.visualize_obstacles(obstacle_points)
        self.obstacle_points = obstacle_points

    def visualize_obstacles(self, obstacle_points):
        # Create marker array for visualization
        marker_array = MarkerArray()
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "obstacles"
        marker.id = 0
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.color.r = 1.0
        marker.color.a = 1.0

        for px, py in obstacle_points:
            # Convert back to global coordinates
            x = px * resolution + origin_x
            y = py * resolution + origin_y
            point = Point()
            point.x = x
            point.y = y
            point.z = 0.0
            marker.points.append(point)

        marker_array.markers.append(marker)
        self.obstacle_pub.publish(marker_array)

    def is_wall(self, px, py):
        # Check surrounding points (OBSTACLE_WINDOW x OBSTACLE_WINDOW)
        for dx in range(-OBSTACLE_WINDOW, OBSTACLE_WINDOW):
            for dy in range(-OBSTACLE_WINDOW, OBSTACLE_WINDOW):
                # Verify indices are within map bounds before accessing
                if (0 <= py + dy < self.map.shape[0] and 
                    0 <= px + dx < self.map.shape[1] and
                    self.map[py + dy, px + dx] < 10): # Walls are 0
                    return True
        return False


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
        # If no obstacle points, return max distance
        if not self.obstacle_points:
            return float('inf')
            
        # Get car's current position from data
        car_x = data.pose.position.x 
        car_y = data.pose.position.y

        min_distance = float('inf')
        
        # Check distance to each obstacle point
        for x, y in self.obstacle_points:
            distance = math.sqrt((x - car_x)**2 + (y - car_y)**2)
            min_distance = min(min_distance, distance)
            
        return min_distance

    def set_mode_pp(self):
        self.drive_mode_pub.publish(DriveMode.PP)

    def set_raceline(self, raceline):
        if raceline not in RACELINES:
            raise ValueError("Invalid raceline")
        self.set_mode_pp()
        self.raceline_pub.publish(raceline)
