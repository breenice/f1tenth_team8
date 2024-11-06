import numpy as np
import math
import rospy

GAP_DETECTION_THRESHOLD = 10
GAP_TOO_CLOSE_THRESHOLD = 2


def find_middle_point(start_i, end_i, ranges, data):
    """
    pick the middle point in the identified gap
    """
    return (start_i + end_i) // 2


def find_least_steering_point(start_i, end_i, ranges, data):
    angle_rad = math.radians(90)  # 90 degrees is directly in front
    angle_min = -(data.angle_min % math.pi)
    center_index = int((angle_rad - angle_min) / data.angle_increment)

    print(start_i, end_i, center_index)
    if start_i > center_index:
        return start_i
    elif end_i < center_index:
        return end_i
    else:
        return center_index


def find_deepest_point(start_i, end_i, ranges, data):
    """
    pick furthest point in the identified gap ^ from there
    """
    # return index of the best point
    return np.argmax(ranges[start_i:end_i + 1]) + start_i
    # if not using numpy
    #max_value = float('-inf')  # negative inf for larger range
    #max_index = start_i  

    #for i in range(start_i, end_i + 1):
    #    if ranges[i] > max_value: 
    #        max_value = ranges[i]  #
    #        max_index = i  # max_index to the current index

    #return max_index


def find_widest_gap(ranges):
    """
    find widest gap
    """
    # create a mask for free space points (non-zero values)
    free_space = ranges > 1
    # split gaps
    # gaps = np.split(np.arange(len(ranges)), np.where(np.diff(free_space) != 0)[0] + 1)
    too_close = np.where(ranges < GAP_TOO_CLOSE_THRESHOLD)[0]

    # areas w no obs since distance in >1 clear checked
    #free_space = [i for i, r in enumerate(ranges) if r > 1]
    # distance less htan the threshold & signals that the car is too close to wall/block
    #too_close = [i for i, r in enumerate(ranges) if r < GAP_TOO_CLOSE_THRESHOLD]
    #if not too_close:  # No obstacles detected
    #    return 0, len(ranges) - 1  # Default to full range

    #gaps = [] # store gaps
    #start = 0

    #for i in range(len(ranges)):
    #    if i in too_close:
    #        if start < i:  # There was a gap before this obstacle
    #            gaps.append((start, i - 1))
    #        start = i + 1  # Move past the obstacle

    #if start < len(ranges):  # Closing the last gap if it exists
    #    gaps.append((start, len(ranges) - 1)) 

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


# TODO: We should also consider other ways to choosing gaps/choosing point in gap
class GapFinder:
    def __init__(self, gap_selection="widest", point_selection="least_steering"):
        gap_selection_functions = {
            "widest": find_widest_gap,
        }

        point_selection_functions = {
            "deepest": find_deepest_point,
            "middle": find_middle_point,
            "least_steering": find_least_steering_point
        }

        self.gap_selection = gap_selection_functions[gap_selection]
        self.point_selection = point_selection_functions[point_selection]

    def get_gap(self, ranges):
        return self.gap_selection(ranges)

    def get_point(self, start_i, end_i, ranges, data):
        return self.point_selection(start_i, end_i, ranges, data)

    def get_point_to_go_to(self, ranges, data):
        start_i, end_i = self.get_gap(ranges)
        best_point = self.get_point(start_i, end_i, ranges, data)

        print("Length: " + str(len(data.ranges)))
        print("Gap: " + str((start_i, end_i)))
        print("Best Point: " + str(best_point))

        return best_point
