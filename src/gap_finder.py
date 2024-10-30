import numpy as np
import rospy

GAP_DETECTION_THRESHOLD = 0.1


def find_middle_point(start_i, end_i, ranges):
    """
    pick the middle point in the identified gap
    """
    return (start_i + end_i) // 2


def find_deepest_point(start_i, end_i, ranges):
    """
    pick furthest point in the identified gap ^ from there
    """
    # return index of the best point
    return np.argmax(ranges[start_i:end_i + 1]) + start_i


def find_widest_gap(ranges):
    """
    find widest gap
    """
    # create a mask for free space points (non-zero values)
    free_space = ranges > 0
    # split gaps
    gaps = np.split(np.arange(len(ranges)), np.where(np.diff(free_space) != 0)[0] + 1)

    # filter out small gaps based on threshold
    valid_gaps = filter_gaps(gaps, ranges)

    if not valid_gaps:
        rospy.logwarn("No valid gaps detected")
        return 0, len(ranges) - 1  # default to full range if no valid gaps

    # find largest gap based on its length, then return start and end indices of the largest gap
    largest_gap = max(valid_gaps, key=len)
    return largest_gap[0], largest_gap[-1]


def filter_gaps(gaps, ranges):
    return [gap for gap in gaps if len(gap) * ranges[0] > GAP_DETECTION_THRESHOLD]

# TODO: We should also consider other ways to choosing gaps/choosing point in gap
class GapFinder:
    def __init__(self, gap_selection="deepest", point_selection="middle"):
        gap_selection_functions = {
            "widest": find_widest_gap,
        }

        point_selection_functions = {
            "deepest": find_deepest_point,
            "middle": find_middle_point
        }

        self.gap_selection = gap_selection_functions[gap_selection]
        self.point_selection = point_selection_functions[point_selection]

    def get_gap(self, ranges):
        return self.gap_selection(ranges)

    def get_point(self, start_i, end_i, ranges):
        return self.point_selection(start_i, end_i, ranges)

    def get_point_to_go_to(self, ranges):
        start_i, end_i = self.get_gap(ranges)
        return self.get_point(start_i, end_i, ranges)
