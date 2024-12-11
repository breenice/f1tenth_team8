import cv2
import rospy
import numpy as np
import math
from std_msgs.msg import Int32, String, Float32, Header, Int16
from sensor_msgs.msg import LaserScan, PointCloud2
from geometry_msgs.msg import PoseStamped, Point, TransformStamped
from visualization_msgs.msg import Marker, MarkerArray
import tf
from overtaker_config import *

from laser_geometry import LaserProjection
import tf2_ros
import tf2_sensor_msgs.tf2_sensor_msgs as tf2_sensor_msgs
from sensor_msgs.point_cloud2 import read_points, create_cloud_xyz32
from obstacle_detector import ObstacleDetector
from raceline_merchant import RacelineMerchant

# from now viewing gaps
#from ftg_config import MAX_LIDAR_DISTANCE, MAXIMUM_SPEED
#from ftg_raceline_control import FTGRacelineControl
#

# Map metadata from the YAML file
origin_x, origin_y = -6.977912, -3.423147  # Origin of the map
resolution = 0.025000  # Resolution of the map


class DriveModeSelector:

    # set mode
    def __init__(self):
        self.obstacle_detector = ObstacleDetector()

        self.drive_mode_pub = rospy.Publisher('/{}/drive_mode'.format(CAR_NAME), Int32, queue_size=1)
        self.raceline_pub = rospy.Publisher('/{}/select_raceline'.format(CAR_NAME), String, queue_size=1)
        self.drive_mult_pub = rospy.Publisher('/{}/speed_mult'.format(CAR_NAME), Float32, queue_size=1)
        self.obstacle_pub = rospy.Publisher('/{}/obstacles'.format(CAR_NAME), PointCloud2, queue_size=1)
        
        self.map = cv2.imread("/home/volta/depend_ws/src/F1tenth_car_workspace/wallfollow/src/final_race/map/base_map.pgm", cv2.IMREAD_GRAYSCALE)
        
        self.current_pose = None
        rospy.Subscriber('/{}/particle_filter/viz/inferred_pose'.format(CAR_NAME), 
                        PoseStamped, self.pose_callback)
        
        self.raceline_merchant = RacelineMerchant()
        self.counter = 0
        self.every = 1
        
        # from now viewing gaps
        # self.ftg_raceline_control = FTGRacelineControl()
        #

        self.current_sector = None
        rospy.Subscriber('/{}/current_sector'.format(CAR_NAME), Int16, self.sector_callback)


    def sector_callback(self, msg):
        self.current_sector = msg.data
    
    def pose_callback(self, msg):
        self.current_pose = msg.pose

        obstacles = self.obstacle_detector.get_obstacle_points()

        raceline = None
        for raceline_name in RACELINES_IN_ORDER:
            raceline_points = self.raceline_merchant.construct_path(raceline_name)
            if self.is_path_clear(raceline_points, obstacles):
                raceline = raceline_name
                break

        print("raceline:", raceline)


        if self.current_sector == Sectors.FREE and raceline is not None:
            self.set_raceline(raceline)
        else:
            self.set_mode_cc()

    def is_path_clear(self, raceline_points, obstacles):
        total_dist = 0
        prev_point = raceline_points[0]
        
        for point in raceline_points[1:]:
            # Calculate distance along raceline
            total_dist += math.sqrt((point[0] - prev_point[0])**2 + (point[1] - prev_point[1])**2)
            
            # Stop checking if we've looked far enough ahead
            if total_dist > RACELINE_LOOKAHEAD:
                return True
                
            # Check if any obstacles are too close to this point
            for obstacle in obstacles:
                dist = math.sqrt((point[0] - obstacle[0])**2 + (point[1] - obstacle[1])**2)
                if dist < SAFETY_DISTANCE:
                    return False
                    
            prev_point = point
            
        return True


    def scan_callback(self, scan):
        if self.current_pose is None:
            return

        
        # from now viewing gaps
         
        #  ranges = np.array(scan.ranges)
        #  ranges = np.where(np.isnan(ranges), MAX_LIDAR_DISTANCE, ranges)
        #  ranges[ranges < 0.05] = MAX_LIDAR_DISTANCE

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
        #     if gap_width > MIN_GAP_SIZE * 2:  # large gap - use center
        #         self.set_raceline('mindist')
        #     elif gap_depth > MAX_LIDAR_DISTANCE * 0.8:  # deep gap - use outer
        #         self.set_raceline('mindist_boundry')
        #     elif gap_width > MIN_GAP_SIZE:  # smaller gap - use inner
        #         self.set_raceline('mincurve')
        #     else:  # cc
        #         self.set_mode_cc()
                
        # except:
        #     # if gap finding fails, switch to cc
        #     self.set_mode_cc()
        
        #this is what you edited after my code
        #  raceline = self.ftg_raceline_control.ftg_control(scan)
        #  print("Choosing raceline: ", raceline)
        #  if raceline:
        #      self.set_raceline(raceline)
        #  else:
        #      self.set_raceline(RACELINES_IN_ORDER[0])
        #      self.set_mode_cc(scan)
        # #
        
                
        # projector = LaserProjection()
        # point_cloud = projector.projectLaser(scan)
        # x_r, y_r = self.current_pose.position.x, self.current_pose.position.y
        # rot = tf.transformations.euler_from_quaternion((self.current_pose.orientation.x,
        #                                             self.current_pose.orientation.y,
        #                                             self.current_pose.orientation.z,
        #                                             self.current_pose.orientation.w))
        # transform_stamped = self.make_transform(x_r, y_r, self.current_pose.orientation)


        # print(x_r, y_r)
        # print(self.current_pose.orientation)
        # print(transform_stamped)

        # theta_r = rot[2]


        # point_cloud_map = tf2_sensor_msgs.do_transform_cloud(point_cloud, transform_stamped)

        # point_generator = read_points(
        #     point_cloud_map,
        #     field_names=("x","y"),
        #     skip_nans=True
        # )

        # global_points = []
        # for x, y in point_generator:
        #     global_points.append((y, x))

        # obstacle_points = []
        # for px, py in global_points:
        #     map_x = int((py - origin_x) / resolution)
        #     map_y = self.map.shape[0] - int((px - origin_y) / resolution)
        #     if not self.is_wall(map_x, map_y):
        #         obstacle_points.append((px, py))

        # obstacle_points.append((0, 0))
        # obstacle_points.append((1, 0))

        # self.visualize_obstacles(obstacle_points)
        # self.obstacle_points = obstacle_points


    def set_mode_stop(self):
        self.drive_mode_pub.publish(DriveMode.STOP)

    def set_drive_mult(self, mult):
        self.drive_mult_pub.publish(mult)
        
    def set_mode_cc(self):
        self.raceline_pub.publish(RACELINES_IN_ORDER[0])

        # maintain a distance of 1.5 meters from the closest object ahead.        
        closest_distance = self.get_closest_object() # closest point (in front)

        if closest_distance < CC_DISTANCE: 
            self.set_drive_mult(0.0)
        else:
            self.set_drive_mult(1.0)

    def get_closest_object(self):
        # If no obstacle points, return max distance
        if not self.obstacle_detector.obstacle_points:
            return float('inf')
            
        car_x, car_y = self.current_pose.position.x, self.current_pose.position.y
        min_distance = float('inf')

        for x, y in self.obstacle_detector.obstacle_points:
            distance = math.sqrt((x - car_x)**2 + (y - car_y)**2)
            min_distance = min(min_distance, distance)
            
        return min_distance


        # from now viewing gaps
        # get the index of the closest object in the scan between 30 and 150 degrees
        #angle_min = -(data.angle_min % math.pi)
        #ranges = np.array(data.ranges)
        
        # Convert angles to indices
        #angle_30 = int((math.radians(30) - angle_min) / data.angle_increment)
        #angle_150 = int((math.radians(150) - angle_min) / data.angle_increment)
        
        # Get ranges between 30 and 150 degrees
        #front_ranges = ranges[angle_30:angle_150]
        
        # Replace inf, nan and small values with max range
        #front_ranges = np.where(np.isnan(front_ranges), MAX_LIDAR_DISTANCE, front_ranges)
        #front_ranges = np.where(np.isinf(front_ranges), MAX_LIDAR_DISTANCE, front_ranges)
        #front_ranges[front_ranges < 0.05] = MAX_LIDAR_DISTANCE
        
        # Return minimum distance
        #return np.min(front_ranges)
        #
        
    def set_mode_pp(self):
        self.drive_mode_pub.publish(DriveMode.PP)

    def set_raceline(self, raceline):
        if raceline not in RACELINES:
            raise ValueError("Invalid raceline")
        self.set_mode_pp()
        self.raceline_pub.publish(raceline, publish=True)