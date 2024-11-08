import numpy as np
import math
import rospy


# TODO: We should also consider other ways to choosing gaps/choosing point in gap
class GapFinder:
    def __init__(self, gap_selection, point_selection, min_gap_size, min_gap_distance):
        gap_selection_functions = {
            "deepest" : self.find_deepest_gap,
            "widest": self.find_widest_gap,
            "least_steering": self.find_least_steering_gap
        }

        point_selection_functions = {
            "deepest": self.find_deepest_point,
            "middle": self.find_middle_point,
            "least_steering": self.find_least_steering_point
        }

        self.gap_selection = gap_selection_functions[gap_selection]
        self.point_selection = point_selection_functions[point_selection]
        self.min_gap_size = min_gap_size
        self.min_gap_distance = min_gap_distance

        self.ranges = None
        self.data = None

    def update_data(self, ranges, data):
        self.ranges = ranges
        self.data = data

    def get_gap(self):
        return self.gap_selection()

    def get_point(self, start_i, end_i):
        return self.point_selection(start_i, end_i)

    def get_point_to_go_to(self):
        start_i, end_i = self.get_gap()
        best_point = self.get_point(start_i, end_i)
        return best_point
    
    def find_middle_point(self, start_i, end_i):
        """
        pick the middle point in the identified gap
        """
        return (start_i + end_i) // 2


    def find_least_steering_point(self, start_i, end_i):
        """
        Find the point in the gap that requires the least steering
        (closest to straight ahead)
        """
        angle_rad = math.radians(90)  # 90 degrees is directly in front
        angle_min = -(self.data.angle_min % math.pi)
        center_index = int((angle_rad - angle_min) / self.data.angle_increment)

        # If gap contains center, return center
        if start_i <= center_index <= end_i:
            return center_index
        
        # If gap is to the lrft of center, return leftmost point
        if start_i > center_index:
            return start_i
            
        # If gap is to the right of center, return leftmost point
        return end_i


    def find_deepest_point(self, start_i, end_i):
        """
        pick furthest point in the identified gap ^ from there
        """
        # return index of the best point
        return np.argmax(self.ranges[start_i:end_i + 1]) + start_i



    def find_widest_gap(self):
        """
        find widest gap
        """
        valid_gaps = self.get_gaps()

        if not valid_gaps:
            rospy.logwarn("No valid gaps detected")
            return 0, len(self.ranges) - 1  # default to full range if no valid gaps

        # find largest gap based on its length, then return start and end indices of the largest gap
        largest_gap = max(valid_gaps, key=len)
        return largest_gap[0], largest_gap[-1]


    def find_deepest_gap(self):
        """
        find deepest gap
        """
        valid_gaps = self.get_gaps()

        if not valid_gaps:
            rospy.logwarn("No valid gaps detected")
            return 0, len(self.ranges) - 1  # default to full range if no valid gaps

        # find largest gap based on its length, then return start and end indices of the largest gap
        largest_gap = max(valid_gaps, key=lambda x : sum(self.ranges[x[0]: x[-1]]))
        return largest_gap[0], largest_gap[-1]



    def filter_gaps(self, gaps):
        return [gap for gap in gaps if len(gap) > self.min_gap_size]


    def find_least_steering_gap(self):
        """
        Find the gap that requires the least steering (most aligned with car's forward direction)
        """
        valid_gaps = self.get_gaps()

        if not valid_gaps:
            rospy.logwarn("No valid gaps detected")
            return 0, len(self.ranges) - 1  # default to full range if no valid gaps

        
        # Calculate the center index (represents straight ahead)
        angle_rad = math.radians(90)  # 90 degrees is directly in front
        angle_min = -(self.data.angle_min % math.pi)
        center_index = int((angle_rad - angle_min) / self.data.angle_increment)
        
        # Find the gap with center closest to the car's forward direction
        min_distance_to_center = float('inf')
        best_gap = valid_gaps[0]
        
        for gap in valid_gaps:
            gap_center = (gap[0] + gap[-1]) // 2
            distance_to_center = abs(gap_center - center_index)
            
            if distance_to_center < min_distance_to_center:
                min_distance_to_center = distance_to_center
                best_gap = gap
        
        return best_gap[0], best_gap[-1]
    
    def get_gaps(self):
        too_close = np.where(self.ranges < self.min_gap_distance)[0]
        
        # Split into gaps
        gaps = np.split(np.arange(len(self.ranges)), too_close)
        
        # Filter out small gaps
        valid_gaps = self.filter_gaps(gaps)

        return valid_gaps
