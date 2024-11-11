import numpy as np
import math
import rospy


# TODO: We should also consider other ways to choosing gaps/choosing point in gap
class GapFinder:
    def __init__(self, gap_selection, point_selection, min_gap_size, min_gap_distance, cornering_distance):
        gap_selection_functions = {
            "deepest" : self.find_deepest_gap,
            "widest": self.find_widest_gap,
            "least_steering": self.find_least_steering_gap,
            "largest_integral": self.find_largest_integral_gap
        }

        point_selection_functions = {
            "deepest": self.find_deepest_point,
            "middle": self.find_middle_point,
            "least_steering": self.find_least_steering_point
        }

        self.gap_selection = gap_selection_functions[gap_selection]
        self.point_selection = point_selection_functions[point_selection]
        self.min_gap_size = min_gap_size
        self.min_gap_distance = min_gap_distance
        self.cornering_distance = cornering_distance

        self.ranges = None
        self.data = None

        # NEW CODE TO FIX GAP SWITCHING; Initialize gap history
        self.previous_gaps = []
        self.gap_history_size = 6 # change

        self.og_min_gap_distance = min_gap_distance  # store orginial min gap distance



        # NEW CODE TO FIX TURNING/CORNERING ISSUE ; track vehicle speed for dynamic cornering
        self.current_speed = 0
        self.prev_steering_angle = 90 # start straight
        self.steering_smoothing = .3 # smoother = lower value


    def update_data(self, ranges, data):
        self.ranges = ranges
        self.data = data

    def get_gap(self):
        valid_gaps = self.get_gaps() # for if no gaps found 
        # ORIGINAL CODE
        # return self.gap_selection()

        # NEW CODE

        # ----- 
        # increase min_gap_distance if no valid gaps found
        if not valid_gaps:
            self.min_gap_distance += 0.5  # increase min gap distance in increments
            if self.min_gap_distance > 2 * self.og_min_gap_distance:
                self.min_gap_distance = self.og_min_gap_distance  # reset if maxed out
            return 0, len(self.ranges) - 1  # default
        
        # reset min_gap_distance if valid gaps are found
        self.min_gap_distance = self.og_min_gap_distance


        # -----
    
        # ADD GAPS TO GAP HISTORY, ENSURE IT FOLLOWS GAP HISTORY SIZE FOR MEMORY
        current_gap = self.gap_selection()
        self.previous_gaps.append(current_gap)
        if len(self.previous_gaps) > self.gap_history_size:
            self.previous_gaps.pop(0)
        
        # if we have atleast 2 gaps in history
        if len(self.previous_gaps) > 1:
            # average center of previous gaps in history (p[0] and p[1] start and end indicies)
            prev_gap_center = sum((p[0] + p[1]) for p in self.previous_gaps[:-1]) / (2 * (len(self.previous_gaps)-1))
            # center of current gap
            current_gap_center = (current_gap[0] + current_gap[1]) / 2
            # if the current gap center differ to much from history 
            threshold = len(self.ranges) * .35 # 35% OF LIDAR READING INDEX IS THRESHOLD  
            if abs(current_gap_center - prev_gap_center) > threshold:
                return self.previous_gaps[-2] # last stable gap we were using
        return current_gap
    




    def get_point(self, start_i, end_i):

        # OLD COOOODE ----

        # Check cornering
        min_index = self.get_index_of(0)
        max_index = self.get_index_of(180)

        close = 0
        for i in range(min_index, max_index):
            if self.ranges[i] < self.cornering_distance:
                close += 1
        if close > 10:
            self.get_index_of(90)

        # ----


        # NEW COOOOOODE -0---- (cornering/turning)

        # center_index = self.get_index_of(90)

        # CORNER_DETECTION_THRESHOLD = 15
        # CORNER_SEARCH_RANGE = 30

        # # count points that are too close
        # close_points = sum(1 for i in range(min_index, max_index)
        #                    if self.ranges[i] < self.cornering_distance)
        # # modified: only consider it a corner if we have more close points
        # if close_points > CORNER_DETECTION_THRESHOLD: # threshold = 20
        #     # calculate a weighted point instead of jumping to 90 degree
        #     center_index = self.get_index_of(90)
        #     # find the furthest point within a reasonable range
        #     search_start = max(start_i, center_index - CORNER_SEARCH_RANGE)          
        #     search_end = min(end_i, center_index + CORNER_SEARCH_RANGE)
            
        #     # get the point with maximum distance in our research range

        #     # smooth the transition
        #     if search_start < search_end:
                
        #         max_dist_index = search_start + np.argmax(self.ranges[search_start::search_end])
        #         target_angle = self.get_angle_from_index(max_dist_index)
        #         smoothed_angle = self.smooth_steering(target_angle)

        #         return self.get_index_of(smoothed_angle)
    
        # # if not cornering, use normal point selection
        # selected_point = self.point_selection(start_i, end_i)

        # # smooth the tansition even in normal driving
        # target_angle = self.get_angle_from_index(selected_point)                                                                                                                                                                     
        # smoothed_angle = self.smooth_steering(target_angle)

        # return self.get_index_of(smoothed_angle)
    

    #---------

        # OLD CODE RETURN
        return self.point_selection(start_i, end_i) 
    
    # NEW CODE HELPER METHOD (cornering/turning)
    def get_angle_from_index(self, index):
        ''' convert lidar to angle in degree'''
        angle_rad = index * self.data.angle_increment + self.data.angle_min
        return math.degrees(angle_rad)
    
    # NEW CODE HELPER METHOD (cornering/turning)
    def smooth_steering(self, target_angle):
        ''' smoothly transition between current steering angle and target angle'''
        ''' formula: new angle = prev angle * (1 - smooth) + target_angle * smooth'''
        smoothed_angle = (self.prev_steering_angle * (1 - self.steering_smoothing) + target_angle * self.steering_smoothing)
        self.prev_steering_angle = smoothed_angle
        return smoothed_angle

    

    def get_point_to_go_to(self):
        start_i, end_i = self.get_gap()
        best_point = self.get_point(start_i, end_i)
        return best_point
    
    def find_middle_point(self, start_i, end_i):
        """
        pick the middle point in the identified gap
        """
        return (start_i + end_i) // 2


    def find_least_steering_point(self, start_i, end_i):
        """
        Find the point in the gap that requires the least steering
        (closest to straight ahead)
        """
        center_index = self.get_index_of(90)

        # If gap contains center, return center
        if start_i <= center_index <= end_i:
            return center_index
        
        # If gap is to the lrft of center, return leftmost point
        if start_i > center_index:
            return start_i
            
        # If gap is to the right of center, return leftmost point
        return end_i


    def find_deepest_point(self, start_i, end_i):
        """
        pick furthest point in the identified gap ^ from there
        """
        # return index of the best point
        return np.argmax(self.ranges[start_i:end_i + 1]) + start_i



    def find_widest_gap(self):
        """
        find widest gap
        """
        valid_gaps = self.get_gaps()

        if not valid_gaps:
            rospy.logwarn("No valid gaps detected")
            return 0, len(self.ranges) - 1  # default to full range if no valid gaps

        # find largest gap based on its length, then return start and end indices of the largest gap
        largest_gap = max(valid_gaps, key=lambda x : x[-1] - x[0])

        return largest_gap[0], largest_gap[-1]


    def find_deepest_gap(self):
        """
        find deepest gap
        """
        valid_gaps = self.get_gaps()

        if not valid_gaps:
            rospy.logwarn("No valid gaps detected")
            return 0, len(self.ranges) - 1  # default to full range if no valid gaps

        # find largest gap based on its length, then return start and end indices of the largest gap
        largest_gap = max(valid_gaps, key=lambda x : max(self.ranges[x[0]: x[-1]]))
        return largest_gap[0], largest_gap[-1]
    
    def find_deepest_favor_left_gap(self):
        """
        find deepest gap
        """
        valid_gaps = self.get_gaps()

        if not valid_gaps:
            rospy.logwarn("No valid gaps detected")
            return 0, len(self.ranges) - 1  # default to full range if no valid gaps

        # find largest gap based on its length, then return start and end indices of the largest gap
        deepest_gap = max(valid_gaps, key=lambda x : max(self.ranges[x[0]: x[-1]]))
        deepest_point = max(self.ranges[deepest_gap[0]:deepest_gap[-1]])
        # Filter to only gaps that have a deepest point within 0.2 meters of the deepest point
        potential_gaps = list(filter(lambda x : deepest_point - max(self.ranges[x[0]: x[-1]]) < 0.2, valid_gaps))
        largest_gap = max(potential_gaps, key=lambda x : x[-1])
        return largest_gap[0], largest_gap[-1]

    
    def find_largest_integral_gap(self):
        """
        find deepest gap
        """
        valid_gaps = self.get_gaps()

        if not valid_gaps:
            rospy.logwarn("No valid gaps detected")
            return 0, len(self.ranges) - 1  # default to full range if no valid gaps

        # find largest gap based on its length, then return start and end indices of the largest gap
        largest_gap = max(valid_gaps, key=lambda x : sum(self.ranges[x[0]: x[-1]]))
        return largest_gap[0], largest_gap[-1]


    def filter_gaps(self, gaps):
        min_index = self.get_index_of(0)
        max_index = self.get_index_of(180)
        valid_gaps = []
        for gap in gaps:
            if len(gap) <= self.min_gap_size or gap[-1] < min_index or gap[0] > max_index:
                continue
            if gap[0] < min_index:
                gap[0] = min_index
            if gap[-1] > max_index:
                gap[-1] = max_index
            valid_gaps.append(gap)

        # print(min_index, max_index)
        # print([(gap[0], gap[-1]) for gap in gaps if len(gap) > self.min_gap_size])
        # print([(gap[0], gap[-1]) for gap in valid_gaps])

        return valid_gaps


    def find_least_steering_gap(self):
        """
        Find the gap that requires the least steering (most aligned with car's forward direction)
        """
        valid_gaps = self.get_gaps()

        if not valid_gaps:
            rospy.logwarn("No valid gaps detected")
            return 0, len(self.ranges) - 1  # default to full range if no valid gaps

        
        # Calculate the center index (represents straight ahead)
        center_index = self.get_index_of(90)
        
        # Find the gap with center closest to the car's forward direction
        min_distance_to_center = float('inf')
        best_gap = valid_gaps[0]
        
        for gap in valid_gaps:
            gap_center = (gap[0] + gap[-1]) // 2
            distance_to_center = abs(gap_center - center_index)
            
            if distance_to_center < min_distance_to_center:
                min_distance_to_center = distance_to_center
                best_gap = gap
        
        return best_gap[0], best_gap[-1]
    
    def get_gaps(self):
        too_close = np.where(self.ranges < self.min_gap_distance)[0]
        
        # Split into gaps
        gaps = np.split(np.arange(len(self.ranges)), too_close)
        
        # Filter out small gaps
        valid_gaps = self.filter_gaps(gaps)

        return valid_gaps
    
    def get_index_of(self, degrees):
        angle_rad = math.radians(degrees)
        angle_min = -(self.data.angle_min % math.pi)
        index = int((angle_rad - angle_min) / self.data.angle_increment)
        return index

