#!/usr/bin/env python

import rospy
import numpy as np
import math
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from ackermann_msgs.msg import AckermannDrive
from sensor_msgs import point_cloud2
from std_msgs.msg import Header
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point


from disparity_extension import DisparityExtender
from gap_finder import GapFinder



# parameters 
SAFETY_RADIUS = 0.5 
GAP_DETECTION_THRESHOLD = 0.1  
MAX_LIDAR_DISTANCE = 10.0 
DEFAULT_VELOCITY = 20.0
TURN_SPEED = 5.0 


class FTGControl:
    def __init__(self):
        self.drive_pub = rospy.Publisher("/car_8/offboard/command", AckermannDrive, queue_size=10)
        self.lidar_pub = rospy.Publisher("/car_8/lidar/processed", PointCloud2, queue_size=2)
        self.marker1_pub = rospy.Publisher('marker1', Marker, queue_size=10)
        self.marker2_pub = rospy.Publisher('marker2', Marker, queue_size=10)
        rospy.Subscriber("/car_8/scan", LaserScan, self.lidar_callback)

        self.disparity_extender = DisparityExtender(safety_radius=SAFETY_RADIUS)
        self.gap_finder = GapFinder()

    # def preprocess_lidar(self, ranges):
    #     """
    #     preprocess LIDAR data by capping max range and handling NaNs
    #     """
    #     processed_ranges = []
    #     for r in ranges:
    #         if math.isnan(r) or r > MAX_LIDAR_DISTANCE:
    #             processed_ranges.append(MAX_LIDAR_DISTANCE)
    #         else:
    #             processed_ranges.append(r)
    #     return processed_ranges
    
    def lidar_callback(self, data):
        """
        callback function to process LIDAR data
        - identify disparities and applies safety measures
        - search for the best gap to navigate
        - publish steering and velocity commands based on the identified gap
        """
        # preprocess LIDAR data by converting LIDAR data to numpy array and handle NaNs (replace NaNs w/ max range)
        ranges = np.array(data.ranges)
        # TODO: We can also try linearly interpolating NaNs instead of setting to max distance
        ranges = np.where(np.isnan(ranges), MAX_LIDAR_DISTANCE, ranges)

        # if we dont want to use numpy?
        # ranges = self.preprocess_lidar(list(data.ranges))

        # TODO: FTG Tweak 4: Set a maximum distance for the LIDAR sensor (2m or 3m)

        # disparity extension
        self.disparity_extender.extend_disparities(ranges, data.angle_increment)

        # find largest gap and select the best point from that gap
        start_i, end_i = self.gap_finder.get_gap(ranges)
        best_point = self.gap_finder.get_point(start_i, end_i, ranges, data)
        # best_point = self.gap_finder.get_point_to_go_to(data) # This does the same as above, but in one line

        # calculate steering angle towards best point
        steering_angle = self.get_steering_angle(best_point, data)

        # publish
        self.publish_drive(steering_angle)
        self.publish_gap_points(data, start_i, end_i)
        self.publish_lidar(data, ranges)

    # TODO: This probably needs tuning
    def get_steering_angle(self, best_point, data):
        steering_direction = self.get_angle_of_lidar_idx(best_point, data)
        steering_offset = steering_direction - 90
        steering_angle = steering_offset * 2.5
        return steering_angle
    
    def get_angle_of_lidar_idx(self, idx, data):
        # angle_min = data.angle_min
        angle_min = -(data.angle_min % math.pi)
        angle = idx * data.angle_increment + angle_min
        return math.degrees(angle)

    def publish_drive(self, steering_angle):
        """
        publish to AckermannDrive with our calculated steering angle
        """
        # create a new AckermannDrive message
        command = AckermannDrive()
        # clamp steering angle between [-100, 100] degrees
        command.steering_angle = min(max(steering_angle, -100), 100)
        # set speed based on steering angle
        command.speed = self.dynamic_velocity(steering_angle)
        # publish drive command
        self.drive_pub.publish(command)

    def dynamic_velocity(self, steering_angle):
        """
        adjust speed based on the distance to the nearest obstacle
        """
        # if turn is sharp (> 30 degrees) then slow down, if not keep default speed
        if abs(steering_angle) > math.radians(30):
            return 15.0
        #    return TURN_SPEED  # if the turn are too sharp
        else:
            return DEFAULT_VELOCITY

        # uhhh attempt to slow speed graudally but not actually sure if it'l work
        # # calculate absolute value of the steering angle to determine severity of the turn
        # abs_angle = abs(steering_angle)

        # # gradually slow speed as the steering angle increases (sharp turn)
        # speed = np.interp(abs_angle, [0, math.radians(90)], [DEFAULT_VELOCITY, TURN_SPEED])

        # return speed

    def publish_lidar(self, data, ranges):
        """
        publish LIDAR data as PointCloud2, add color to borders of gap
        """
        # Setup header for PointCloud2
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = data.header.frame_id
        fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
            PointField('rgb', 12, PointField.UINT32, 1)
        ]

        angle = data.angle_min
        points = []
        for r, r_ext in enumerate(zip(data.ranges, ranges)):
            if r == r_ext:
                x = r * math.cos(angle)
                y = r * math.sin(angle)
                z = 0

                point = [x, y, z, 0XFFFFFF]  # white
                points.append(point)
            else:  # this point was disparity extended
                x = r * math.cos(angle)
                y = r * math.sin(angle)
                z = 0

                point = [x, y, z, 0XFFFFFF]  # white
                points.append(point)

                x_ext = r_ext * math.cos(angle)
                y_ext = r_ext * math.sin(angle)
                z_ext = 0

                point_ext = [x_ext, y_ext, z_ext, 0XFF0000]  # red
                points.append(point_ext)

            angle += data.angle_increment

        # create PointCloud2 message
        pc2_msg = point_cloud2.create_cloud(header, fields, points)

        resize = 200
        pc2_msg.height = resize
        pc2_msg.width = resize
        pc2_msg.row_step = resize * pc2_msg.point_step

        new_size = pc2_msg * resize * pc2_msg.point_step
        pc2_msg.data = bytearray(new_size)
        self.lidar_pub.publish(pc2_msg)

    def publish_gap_points(self, data, start_i, end_i):
        # add all laser scan points, but set start_i and end_i to red (edge of gap)
        r1 = data.ranges[start_i]
        angle = data.angle_min + start_i * data.angle_increment
        x1 = r1 * math.cos(angle)
        y1 = r1 * math.sin(angle)
        z1 = 0
        m1 = self.generate_marker(x1, y1, z1, [0.0, 1.0, 0.0, 1.0])
        self.marker1_pub.publish(m1)

        r2 = data.ranges[end_i]
        angle = data.angle_min + end_i * data.angle_increment
        x2 = r2 * math.cos(angle)
        y2 = r2 * math.sin(angle)
        z2 = 0
        m2 = self.generate_marker(x2, y2, z2, [1.0, 0.0, 0.0, 1.0])
        self.marker2_pub.publish(m2)

    def generate_marker(self, x, y, z, color):
        marker = Marker()
        marker.header.frame_id = "car_8_laser"
        marker.header.stamp = rospy.Time.now()
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]
        return marker


if __name__ == '__main__':
    """
    init ros node and keep running 
    """
    rospy.init_node('follow_gap', anonymous=True)
    FTGControl()
    rospy.spin()
