import numpy as np


class DisparityExtender:
    def __init__(self, safety_radius=0.5, max_range=3):
        self._safety_radius = safety_radius
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
        ranges = original_ranges.copy()
        ranges = self.preprocess_lidar(ranges)
        # calculate how many indices to include in the safety bubble
        bubble_range = int(self._safety_radius / angle_increment)

        # process from right, extend bubble to left
        for i in range(len(ranges) - 2, -1, -1):  # to avoid index error
            # detect disparity by checking if the distance change is greater than the safety radius
            if ranges[i] - ranges[i + 1] > self._safety_radius:
                start_idx = max(0, i - bubble_range)
                obstacle_distance = ranges[i+1]
                # extend the bubble left by setting points within range to the obstacle distance
                for j in range(i, start_idx - 1, -1):
                    ranges[j] = min(ranges[j], obstacle_distance)

        # process from left, extend bubble to right
        for i in range(1, len(ranges)):
            # detect a disparity ^ same way
            if ranges[i] - ranges[i - 1] > self._safety_radius:
                end_idx = min(len(ranges), i + bubble_range)
                obstacle_distance = ranges[i-1]
                # extend right 
                for j in range(i, end_idx):
                    ranges[j] = min(ranges[j], obstacle_distance)

        return ranges
