import rospy
from std_msgs.msg import Int32, String
from overtaker_config import *

class DriveModeSelector:
    def __init__(self):
        self.drive_mode_pub = rospy.Publisher('/{}/drive_mode'.format(CAR_NAME), Int32, queue_size=1)
        self.raceline_pub = rospy.Publisher('/{}/raceline'.format(CAR_NAME), String, queue_size=1)
        # TODO : Subscribe to lidar or other sensors to determine drive mode

    def set_mode_stop(self):
        self.drive_mode_pub.publish(DriveMode.STOP)

    def set_mode_ftg(self):
        self.drive_mode_pub.publish(DriveMode.FTG)

    def set_mode_pp(self):
        self.drive_mode_pub.publish(DriveMode.PP)

    def set_raceline(self, raceline):
        if raceline not in RACELINES:
            raise ValueError("Invalid raceline")
        self.set_mode_pp()
        self.raceline_pub.publish(raceline)
