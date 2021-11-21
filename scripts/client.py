#!/usr/bin/env python3


from sys import executable
import rospy
from backbone.srv import ThrottleTopic, ThrottleTopicResponse
import std_msgs.msg
import roslaunch

def callback(message: std_msgs.msg.String):
    rospy.loginfo(f"client1: got {message.data}")


def main() -> int:
    rospy.init_node("client_test")
    rospy.wait_for_service("test_topicMultiplexer")
    try:
        throttletopic = rospy.ServiceProxy("test_topicMultiplexer", ThrottleTopic)
        resp: ThrottleTopicResponse = throttletopic(2)
        print(f"got new topic {resp.topic_name}")
        topic = resp.topic_name
        rospy.Subscriber(topic, data_class=std_msgs.msg.String, callback=callback)
        rospy.loginfo(f"listening on topic {topic}")
        rospy.spin()

    except rospy.ServiceException as e:
        print(f"Failed service call: {e}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
