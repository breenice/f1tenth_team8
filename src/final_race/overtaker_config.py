CAR_NAME = "car_8"


class DriveMode:
    STOP = 0
    PP = 1


class SpeedMode:
    STOP = 0
    PP = 1
    CC = 2  # Cruise Control
    OT = 3  # Overtake


class Sectors:
    FREE = 0
    MID = 1
    DANGER = 2


# Racelines are the paths that the overtaker can choose between
RACELINES = ["mincurve", "mindist_boundary", "mindist", "mincurve", "demoline"]

# Racelines in the order the overtaker can choose between
RACELINES_IN_ORDER = ["mincurve"]

OBSTACLE_WINDOW = 10

CC_DISTANCE = 1.5
