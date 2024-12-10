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

# Racelines are the paths that the overtaker can choose between
RACELINES = ["mincurve", "mindist_boundary", "mindist", "mincurve", "demoline"]