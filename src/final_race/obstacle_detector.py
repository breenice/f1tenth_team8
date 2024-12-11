import rospy
import cv2
import numpy as np
import math
import tf
import tf2_ros
import tf2_sensor_msgs.tf2_sensor_msgs as tf2_sensor_msgs
from sensor_msgs.msg import LaserScan, PointCloud2
from geometry_msgs.msg import TransformStamped, PoseStamped
from std_msgs.msg import Header
from laser_geometry import LaserProjection
from sensor_msgs.point_cloud2 import read_points, create_cloud_xyz32
from overtaker_config import *

class ObstacleDetector:
    def __init__(self):
        self.obstacle_pub = rospy.Publisher('/{}/obstacles'.format(CAR_NAME), PointCloud2, queue_size=1)
        self.map = cv2.imread("/home/volta/depend_ws/src/F1tenth_car_workspace/wallfollow/src/final_race/map/base_map.pgm", cv2.IMREAD_GRAYSCALE)
        
        # Map metadata from the YAML file
        self.origin_x, self.origin_y = -6.977912, -3.423147  # Origin of the map
        self.resolution = 0.025000  # Resolution of the map
        
        self.current_pose = None
        self.obstacle_points = []
        
        rospy.Subscriber('/{}/particle_filter/viz/inferred_pose'.format(CAR_NAME), 
                        PoseStamped, self.pose_callback)
        rospy.Subscriber("/car_8/scan", LaserScan, self.scan_callback)

    def pose_callback(self, msg):
        self.current_pose = msg.pose

    def scan_callback(self, scan):
        if self.current_pose is None:
            return
        
        projector = LaserProjection()
        point_cloud = projector.projectLaser(scan)

        x_r, y_r = self.current_pose.position.x, self.current_pose.position.y
        transform_stamped = self.make_transform(x_r, y_r, self.current_pose.orientation)

        point_cloud_map = tf2_sensor_msgs.do_transform_cloud(point_cloud, transform_stamped)

        point_generator = read_points(
            point_cloud_map,
            field_names=("x","y"),
            skip_nans=True
        )

        global_points = []
        for x, y in point_generator:
            global_points.append((y, x))

        obstacle_points = []
        for px, py in global_points:
            map_x = int((py - self.origin_x) / self.resolution)
            map_y = self.map.shape[0] - int((px - self.origin_y) / self.resolution)
            if not self.is_wall(map_x, map_y):
                obstacle_points.append((px, py))

        self.visualize_obstacles(obstacle_points)
        self.obstacle_points = obstacle_points

    def make_transform(self, x, y, rot):
        trans = TransformStamped()
        trans.header.stamp = rospy.Time.now()
        trans.header.frame_id = "car_8_laser"
        trans.child_frame_id = "map"

        trans.transform.translation.x = x
        trans.transform.translation.y = y
        trans.transform.translation.z = 0

        trans.transform.rotation.x = rot.x
        trans.transform.rotation.y = rot.y
        trans.transform.rotation.z = rot.z
        trans.transform.rotation.w = rot.w

        return trans

    def visualize_obstacles(self, obstacle_points):
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "map"

        points = []
        for px, py in obstacle_points:
            points.append([py, px, 0.0])

        pc2 = create_cloud_xyz32(header, points)
        self.obstacle_pub.publish(pc2)

    def is_wall(self, px, py):
        for dx in range(-OBSTACLE_WINDOW, OBSTACLE_WINDOW):
            for dy in range(-OBSTACLE_WINDOW, OBSTACLE_WINDOW):
                if (0 <= py + dy < self.map.shape[0] and 
                    0 <= px + dx < self.map.shape[1] and
                    self.map[py + dy, px + dx] < 10):
                    return True
        return False

    def get_obstacle_points(self):
        return self.obstacle_points