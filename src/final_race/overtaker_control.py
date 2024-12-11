import rospy
from geometry_msgs.msg import PoseStamped
from ackermann_msgs.msg import AckermannDrive
from std_msgs.msg import Int32, String, Float32

from multi_pp_control import MultiPPControl
from overtaker_config import *

class OvertakerControl:
    def __init__(self):
        self.drive_mode = DriveMode.PP
        self.speed_mode = SpeedMode.PP
        rospy.Subscriber('/{}/drive_mode'.format(CAR_NAME), Int32, self.set_drive_mode)
        rospy.Subscriber('/{}/speed_mode'.format(CAR_NAME), Int32, self.set_speed_mode)

        self.racelines = RACELINES
        rospy.Subscriber('/{}/select_raceline'.format(CAR_NAME), String, self.set_raceline)
        
        self.command_pub = rospy.Publisher('/{}/offboard/command'.format(CAR_NAME), AckermannDrive, queue_size=1)

        self.pp_control = MultiPPControl()
        self.init_pp()

        self.current_cc_mult = 1.0
        self.target_cc_mult = 1.0

        rospy.Subscriber('/{}/cruise_mult'.format(CAR_NAME), Float32, self.cc_mult_callback)

    def set_raceline(self, raceline):
        self.pp_control.select_raceline(raceline)

    def set_drive_mode(self, msg):
        self.drive_mode = msg.data

    def set_speed_mode(self, msg):
        self.speed_mode = msg.data

    def cc_mult_callback(self, msg):
        self.target_cc_mult = msg.data

    def init_pp(self):
        rospy.Subscriber('/{}/particle_filter/viz/inferred_pose'.format(CAR_NAME), PoseStamped,
                         self.mpp_control)

    def mpp_control(self, data):
        if self.drive_mode == DriveMode.PP:
            command = self.pp_control.pp_control(data)
            steering_angle = command.steering_angle
            speed = command.speed

            self.target_speed = speed
            self.publish_command(steering_angle)
            print("steering:", steering_angle, "speed:", speed)

    def get_speed(self):
        if self.speed_mode == SpeedMode.STOP:
            return 0.0
        
        if self.speed_mode == SpeedMode.CC:
            if self.target_cc_mult < self.current_cc_mult:
                self.current_cc_mult = max(self.target_cc_mult, self.current_cc_mult - 0.05)
            else:
                self.current_cc_mult = self.target_cc_mult
        
        return self.target_speed * self.current_cc_mult

    def publish_command(self, steering_angle):
        command = AckermannDrive()
        command.steering_angle = steering_angle
        command.speed = self.get_speed()
        self.command_pub.publish(command)
