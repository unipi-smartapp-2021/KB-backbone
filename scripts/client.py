#!/usr/bin/env python3


import rospy
from backbone.srv import ThrottleTopic


def main() -> int:
    while not rospy.is_shutdown():
        rospy.wait_for_service("test_topicMultiplexer")
        try:
            throttletopic = rospy.ServiceProxy("test_topicMultiplexer", ThrottleTopic)
            resp = throttletopic(10)
            print(f"got {resp.topic_name}")
            rospy.sleep(1)
        except rospy.ServiceException as e:
            print(f"Failed service call: {e}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
