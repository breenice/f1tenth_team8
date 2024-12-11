CAR_NAME = "car_8"

class DriveMode:
    STOP = 0
    FTG = 1
    PP = 2

class SpeedMode:
    STOP = 0
    FTG = 1
    PP = 2
    CC = 3 # Cruise Control

class Sectors:
    FREE = 0
    MID = 1
    DANGER = 2

# Racelines are the paths that the overtaker can choose between
RACELINES = ["edited_raceline", "mindist_boundary", "mindist", "mincurve", "drawn_w_speed", "demoline"]