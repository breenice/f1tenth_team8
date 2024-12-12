CAR_NAME = "car_8"


class DriveMode:
    STOP = 0
    PP = 1
    FTG = 2

class Sectors:
    FREE = 0
    MID = 1
    DANGER = 2


# Racelines are the paths that the overtaker can choose between
RACELINES = ["mincurve", "mincurve", "mindist_boundary", "mindist", "mincurve", "demoline", "center", "wide"]

# Racelines in the order the overtaker can choose between
RACELINES_IN_ORDER = ["mincurve", "mindist", "center", "wide"]

OBSTACLE_WINDOW = 10
RACELINE_LOOKAHEAD = 2.0
SAFETY_DISTANCE = 0.3

POINT_GROUP_DISTANCE = 0.1
MIN_POINT_GROUP_SIZE = 5

CC_DISTANCE = 1
