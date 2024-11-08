import numpy as np


class DisparityExtender:
    def __init__(self, disparity_distance, safety_extension, max_range):
        self.disparity = disparity_distance
        self._safety_safety_extension = safety_extension
        self._max_range = max_range

    def preprocess_lidar(self, ranges):
        # just to make sure all ranges capped to max
        return np.clip(ranges, 0, self._max_range)

    def extend_disparities(self, original_ranges, angle_increment):
        """
        apply a safety bubble around all obstacles detected'
        -- instead of extending from both directions, we now process range twice: 
        right (extending left) and left (extending right)
        """
        original_ranges = self.preprocess_lidar(original_ranges)
        ranges = original_ranges.copy()
        # calculate how many indices to include in the safety bubble

        # process from right, extend bubble to left
        for i in range(len(ranges) - 2, -1, -1):  # to avoid index error
            # detect disparity by checking if the distance change is greater than the safety radius
            if original_ranges[i] - original_ranges[i + 1] > self.disparity:
                start_idx = max(0, i - self._safety_safety_extension)
                obstacle_distance = original_ranges[i+1]
                # extend the bubble left by setting points within range to the obstacle distance
                for j in range(i, start_idx - 1, -1):
                    ranges[j] = min(original_ranges[j], obstacle_distance)

        # process from left, extend bubble to right
        for i in range(1, len(original_ranges)):
            # detect a disparity ^ same way
            if original_ranges[i] - original_ranges[i - 1] > self.disparity:
                end_idx = min(len(ranges), i + self._safety_safety_extension)
                obstacle_distance = original_ranges[i-1]
                # extend right 
                for j in range(i, end_idx):
                    ranges[j] = min(original_ranges[j], obstacle_distance)

        return ranges
