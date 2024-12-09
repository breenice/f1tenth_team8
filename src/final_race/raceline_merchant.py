import os
import csv

PATH_FOLDER = '/home/volta/depend_ws/src/f1tenth_purepursuit/path'

class RacelineMerchant:
    _instance = None
    _cache = {}

    def __new__(cls):
        if cls._instance is None:
            cls._instance = super(RacelineMerchant, cls).__new__(cls)
        return cls._instance

    def __init__(self):
        if not hasattr(self, 'initialized'):
            self.initialized = True
            self.plan = []

    def construct_path(self, trajectory_name):
        """
        Function to construct the path from a CSV file. Uses caching to avoid re-reading files.
        """
        if trajectory_name in self._cache:
            self.plan = self._cache[trajectory_name]
            return

        self.plan = []
        
        # TODO: Modify this path to match the folder where the csv file containing the path is located.
        file_path = os.path.expanduser(
            '{}/{}.csv'.format(PATH_FOLDER, trajectory_name))

        with open(file_path) as csv_file:
            csv_reader = csv.reader(csv_file, delimiter=',')
            for waypoint in csv_reader:
                self.plan.append(waypoint)

        # Convert string coordinates to floats and calculate path resolution
        for index in range(0, len(self.plan)):
            for point in range(0, len(self.plan[index])):
                self.plan[index][point] = float(self.plan[index][point])
                
        # Cache the processed path
        self._cache[trajectory_name] = self.plan.copy()
