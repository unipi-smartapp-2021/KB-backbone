from typing import List, TypeVar

import rospy
from backbone.rated_publisher import RatedPublisher
from backbone.srv import ThrottleTopic, ThrottleTopicRequest, ThrottleTopicResponse

T = TypeVar("T")


class FrequencyMultiplexer:
    def __init__(self, topic: str, msg: T) -> None:
        rospy.init_node("freq_multiplexer")
        self.data_class: T = msg
        self.topic: str = topic
        self.name: str = f"{topic}Multiplexer"
        self.service: rospy.Service = rospy.Service(
            self.name, ThrottleTopic, self._registerToNode
        )
        self.input: rospy.Subscriber = rospy.Subscriber(
            topic, self.data_class, callback=self._forwardMessages
        )
        self.publishers: List[RatedPublisher] = []
        self.subscribed: int = 0

    def run(self) -> None:
        rospy.spin()

    def _registerToNode(self, req: ThrottleTopicRequest) -> ThrottleTopicResponse:
        new_topic = f"{self.topic}_{self.subscribed}"
        self.publishers.append(
            RatedPublisher(
                rospy.Publisher(new_topic, self.data_class, queue_size=1), req.rate
            ),
        )
        self.subscribed += 1
        res = ThrottleTopicResponse(new_topic)
        return res

    def _forwardMessages(self, msg: T) -> None:
        for pub in self.publishers:
            pub.publish(msg)
