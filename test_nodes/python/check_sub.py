import rospy
from backbone.msg import TestMsg
def cb(msg):
    print(msg.data)

def main() -> int:
    rospy.init_node("check2")
    rospy.Subscriber("test", TestMsg, callback=cb, queue_size=1)
    rospy.spin()


if __name__ == "__main__":
    raise SystemExit(main())