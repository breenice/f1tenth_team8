#!/usr/bin/env python

import rospy
import numpy as np
import math
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from ackermann_msgs.msg import AckermannDrive
from sensor_msgs import point_cloud2
from std_msgs.msg import Header


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

        # apply safety bubble around closest obstacle
        # TODO: FTG Tweak 2: Disparity Extension should apply to every disparity, not just the closest point
        closest_idx = np.argmin(ranges)  # index of closest point
        # self.disparity_extender.safety_bubble(ranges, closest_idx, data.angle_increment)

        # find largest gap and select the best point from that gap
        start_i, end_i = self.gap_finder.get_gap(ranges)
        best_point = self.gap_finder.get_point(start_i, end_i, ranges)
        # best_point = self.gap_finder.get_point_to_go_to(ranges) # This does the same as above, but in one line

        # calculate steering angle towards best point
        steering_angle = self.get_steering_angle(best_point, data)

        # publish
        self.publish_drive(steering_angle)
        self.publish_lidar(data, start_i, end_i)

    # TODO: This probably needs tuning
    def get_steering_angle(self, best_point, data):
        steering_direction = self.get_angle_of_lidar_idx(best_point, data)
        steering_offset = steering_direction - 90
        steering_angle = steering_offset * 2
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

    def publish_lidar(self, data, start_i, end_i):
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

        # add all laser scan points, but set start_i and end_i to red (edge of gap)
        angle = data.angle_min
        points = []
        for i, r in enumerate(data.ranges):
            if i == start_i or i == end_i:
                color = 0xFF0000
            else:
                color = 0x00FF00

            if data.range_min <= r <= data.range_max:
                x = r * math.cos(angle)
                y = r * math.sin(angle)
                z = 0

                point = [x, y, z, color]
                points.append(point)

            angle += data.angle_increment

        pc2_msg = point_cloud2.create_cloud(header, fields, points)
        # pc2_msg.resize(200)
        # int scale = 3
        #pc2_msg->width = cloud->width * scale;
        #pc2_msg->height = cloud->height * scale;
        #pc2_msg->points.resize(pc2_msg->width * pc2_msg->height);
        #for  size_t i = 0, ii = 0; i < pc2_msg->height; ii += scale, i++{
        #    for size_t j = 0, jj = 0; j < pc2_msg->width; jj += scale, j++{
        #        pc2_msg->at(j, i) = cloud->at(jj, ii)
        #    }
        #}
        self.lidar_pub.publish(pc2_msg)


if __name__ == '__main__':
    """
    init ros node and keep running 
    """
    rospy.init_node('follow_gap', anonymous=True)
    FTGControl()
    rospy.spin()
