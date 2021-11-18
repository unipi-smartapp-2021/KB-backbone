import rospy
from std_msgs.msg import String


def talker():
    rospy.init_node("talker", anonymous=True)
    pub = rospy.Publisher("test", String, queue_size=10)
    r = rospy.Rate(10)  # 10hz
    while not rospy.is_shutdown():
        str = "hello world %s" % rospy.get_time()
        rospy.loginfo(str)
        pub.publish(str)
        r.sleep()


if __name__ == "__main__":
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
