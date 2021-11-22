#!/usr/bin/env python3

import rospy
import std_msgs.msg


def main() -> int:
    rospy.init_node("source")
    pub: rospy.Publisher = rospy.Publisher(
        "test_topic", data_class=std_msgs.msg.String, queue_size=1
    )
    rate = rospy.Rate(20)
    sent = 0
    while not rospy.is_shutdown():
        pub.publish(f"message {sent}")
        sent += 1
        rate.sleep()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
