#!/usr/bin/env python3

import rospy
from std_msgs.msg import Duration

def debug(msg: Duration) -> None:
    rospy.loginfo(f"(DEBUGGER) Delay is :{msg.data.to_nsec() / 1000} microseconds")

def main() -> int:
    rospy.init_node("debugger")
    rospy.Subscriber("test_topicRatedDelay",Duration,callback=debug, queue_size=1)
    rospy.spin()

if __name__ == "__main__":
    raise SystemExit(main())