
class DisparityExtender:
    def __init__(self, safety_radius=0.5):
        self._safety_radius = safety_radius

    # TODO: Safety bubble should extend outwards in one direction from the disparity, and the value should be the
    #  distance of the obstacle, not 0
    def safety_bubble(self, ranges, closest_idx, angle_increment):
        """
        apply a safety bubble around all obstacles detected
        """
        bubble_range = int(self.safety_radius / angle_increment)

        for i in range(1, len(ranges)):
            if abs(ranges[i] - ranges[i - 1]) > self.safety_radius:
                start_idx = max(0, i - bubble_range)
                end_idx = min(len(ranges), i + bubble_range)

                #extend bubble area

                obstacle_distance = ranges[i] 
                for j in range(start_idx, end_idx):
                    ranges[j] = min(ranges[j], obstacle_distance)
