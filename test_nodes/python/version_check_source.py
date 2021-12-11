import rospy
from backbone.msg import TestMsg
def main() -> int:
    rospy.init_node(name="name")

    pub = rospy.Publisher("test",TestMsg)
    rate = rospy.Rate(10)
    msg = TestMsg()
    msg.version_header.version = 2
    msg.data = "hello"
    while not rospy.is_shutdown():
        pub.publish(msg)
        rate.sleep()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())