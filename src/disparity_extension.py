

class DisparityExtender:
    def __init__(self, safety_radius=0.5):
        self._safety_radius = safety_radius

    # TODO: Safety bubble should extend outwards in one direction from the disparity, and the value should be the
    #  distance of the obstacle, not 0
    def safety_bubble(self, ranges, closest_idx, angle_increment):
        """
        apply a safety bubble around disparities to avoid collisions
        """
        # calculate the start and end indices for the safety bubble
        start = max(0, closest_idx - int(self._safety_radius / angle_increment))
        end = min(len(ranges), closest_idx + int(self._safety_radius / angle_increment))

        # mark all points within the safety bubble as obstacles (set them to 0)
        ranges[start:end] = 0
