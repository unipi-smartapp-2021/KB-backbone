#!/usr/bin/env python3


from sys import executable
import rospy
from backbone.srv import RateTopic, RateTopicResponse
import std_msgs.msg
import roslaunch

def callback(message: std_msgs.msg.String) -> None:
    rospy.loginfo(f"client2: got {message.data}")


def main() -> int:
    rospy.init_node("client_test2")
    rospy.wait_for_service("test_topicRated")
    try:
        throttletopic = rospy.ServiceProxy("test_topicRated", RateTopic)
        resp: RateTopicResponse = throttletopic(11)
        print(f"CLIENT 2) got new topic {resp.topic}")
        topic = resp.topic
        rospy.Subscriber(topic, data_class=std_msgs.msg.String, callback=callback)
        rospy.loginfo(f"listening on topic {topic}")
        rospy.spin()

    except rospy.ServiceException as e:
        print(f"Failed service call: {e}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
