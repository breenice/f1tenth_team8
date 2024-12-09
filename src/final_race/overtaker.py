#!/usr/bin/env python

import rospy
from overtaker_control import OvertakerControl
from drive_mode_selector import DriveModeSelector


if __name__ == "__main__":
    try:
        rospy.init_node("overtaker", anonymous=True)
        overtaker = OvertakerControl()
        dms = DriveModeSelector()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("An error occurred: {}", rospy.ROSInterruptException)
        pass
