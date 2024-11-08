import numpy as np
import math
import rospy

GAP_DETECTION_THRESHOLD = 30
GAP_TOO_CLOSE_THRESHOLD = 2
GAP_ANGLE_CHANGE_THRESHOLD = 10 # max change allowed

def find_middle_point(start_i, end_i, ranges, data):
    """
    pick the middle point in the identified gap
    """
    return (start_i + end_i) // 2


def find_least_steering_point(start_i, end_i, ranges, data):
    """
    Find the point in the gap that requires the least steering
    (closest to straight ahead)
    """
    angle_rad = math.radians(90)  # 90 degrees is directly in front
    angle_min = -(data.angle_min % math.pi)
    center_index = int((angle_rad - angle_min) / data.angle_increment)

    # If gap contains center, return center
    if start_i <= center_index <= end_i:
        return center_index
    
    # If gap is to the right of center, return leftmost point
    if start_i > center_index:
        return start_i
        
    # If gap is to the left of center, return rightmost point
    return end_i


def find_deepest_point(start_i, end_i, ranges, data):
    """
    pick furthest point in the identified gap ^ from there
    """
    # return index of the best point
    return np.argmax(ranges[start_i:end_i + 1]) + start_i



def find_widest_gap(ranges, data):
    """
    find widest gap
    """
    # create a mask for free space points (non-zero values)
    free_space = ranges > 1
    # split gaps
    # gaps = np.split(np.arange(len(ranges)), np.where(np.diff(free_space) != 0)[0] + 1)
    too_close = np.where(ranges < GAP_TOO_CLOSE_THRESHOLD)[0]
    gaps = np.split(np.arange(len(ranges)), too_close)

    # filter out small gaps based on threshold
    valid_gaps = filter_gaps(gaps, ranges)

    if not valid_gaps:
        rospy.logwarn("No valid gaps detected")
        return 0, len(ranges) - 1  # default to full range if no valid gaps

    # find largest gap based on its length, then return start and end indices of the largest gap
    largest_gap = max(valid_gaps, key=len)
    return largest_gap[0], largest_gap[-1]


def filter_gaps(gaps, ranges):
    return [gap for gap in gaps if len(gap) > GAP_DETECTION_THRESHOLD]


def find_least_steering_gap(ranges, data):
    """
    Find the gap that requires the least steering (most aligned with car's forward direction)
    """
    # create a mask for free space points (non-zero values)
    free_space = ranges > 1
    too_close = np.where(ranges < GAP_TOO_CLOSE_THRESHOLD)[0]
    
    # Split into gaps
    gaps = np.split(np.arange(len(ranges)), too_close)
    
    # Filter out small gaps
    valid_gaps = filter_gaps(gaps, ranges)
    
    # Calculate the center index (represents straight ahead)
    angle_rad = math.radians(90)  # 90 degrees is directly in front
    angle_min = -(data.angle_min % math.pi)
    center_index = int((angle_rad - angle_min) / data.angle_increment)
    
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


# TODO: We should also consider other ways to choosing gaps/choosing point in gap
class GapFinder:
    def __init__(self, gap_selection="widest", point_selection="middle"):
        gap_selection_functions = {
            "widest": find_widest_gap,
            "least_steering": find_least_steering_gap
        }

        point_selection_functions = {
            "deepest": find_deepest_point,
            "middle": find_middle_point,
            "least_steering": find_least_steering_point
        }

        self.gap_selection = gap_selection_functions[gap_selection]
        self.point_selection = point_selection_functions[point_selection]

    def get_gap(self, ranges, data):
        return self.gap_selection(ranges, data)

    def get_point(self, start_i, end_i, ranges, data):
        return self.point_selection(start_i, end_i, ranges, data)

    def get_point_to_go_to(self, ranges, data):
        start_i, end_i = self.get_gap(ranges)
        best_point = self.get_point(start_i, end_i, ranges, data)

        print("Length: " + str(len(data.ranges)))
        print("Gap: " + str((start_i, end_i)))
        print("Best Point: " + str(best_point))

        return best_point
