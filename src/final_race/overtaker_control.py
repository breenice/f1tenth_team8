import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDrive
from std_msgs.msg import Int32, String

from pp.multi_pp_control import MultiPPControl
from ftg.ftg_control import FTGControl
from overtaker_config import *

class OvertakerControl:
    def __init__(self):
        self.drive_mode = DriveMode.STOP
        rospy.Subscriber('/{}/drive_mode'.format(CAR_NAME), Int32, self.set_drive_mode)

        self.racelines = RACELINES
        rospy.Subscriber('/{}/raceline'.format(CAR_NAME), String, self.raceline_callback)
        
        self.command_pub = rospy.Publisher('/{}/offboard/command'.format(CAR_NAME), AckermannDrive, queue_size=1)

        self.pp_control = MultiPPControl()
        self.init_pp()

        self.ftg_control = FTGControl()
        self.init_ftg()

    def set_raceline(self, raceline):
        self.pp_control.select_raceline(raceline)

    def set_drive_mode(self, msg):
        self.drive_mode = msg.data

    def init_pp(self):
        rospy.Subscriber('/{}/particle_filter/viz/inferred_pose'.format(CAR_NAME), PoseStamped,
                         self.pp_control.pp_control)

    def mpp_control(self, data):
        if self.drive_mode == DriveMode.PP:
            command = self.pp_control.pp_control(data)
            self.command_pub.publish(command)

    def init_ftg(self):
        rospy.Subscriber("/car_8/scan", LaserScan, self.ftg_control)

    def ftg_control(self, data):
        if self.drive_mode == DriveMode.FTG:
            command = self.ftg_control.ftg_control(data)
            self.command_pub.publish(command)
