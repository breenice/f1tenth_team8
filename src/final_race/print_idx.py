#!/usr/bin/env python

import os
import csv
import math
from pp import *
from pp_config import *
import numpy as np

class PrintIdx(PurePursuit):
    def __init__(self):
        super().__init__(LOOKAHEAD_DISTANCE, VELOCITY_LOOKAHEAD_DISTANCE, MIN_SPEED, MAX_SPEED)
        print_idx()


    def print_idx(self, data):
        curr_x = data.pose.position.x
        curr_y = data.pose.position.y
        distances = [np.sqrt((x - curr_x)**2 + (y - curr_y)**2) for x, y in self.plan]
        closest_index = np.argmin(distances)
        print("index: ", closest_index)
        return closest_index
