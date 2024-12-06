import rospy
from overtaker_control import OvertakerControl


if __name__ == "__main__":
    try:
        rospy.init_node("overtaker")
        overtaker = OvertakerControl()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("An error occurred: {}", rospy.ROSInterruptException)
        pass
