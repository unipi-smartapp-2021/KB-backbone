"""! @package backbone.frequency_multiplexer

@author backbone_team
"""


from typing import List, TypeVar

import rospy
from backbone.rated_publisher import RatedPublisher
from backbone.srv import ThrottleTopic, ThrottleTopicRequest, ThrottleTopicResponse

T = TypeVar("T")


class FrequencyMultiplexer:
    """! Class that define a Multiplexxing Publisher that send at a given frequency

    """
    def __init__(self, topic: str, msg: T) -> None:
        """! Construction of the class

        @param topic the topic name of the publisher
        @param msg the message type we are going to send
        @return None
        """
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
        """! Base function that start the thread

        """
        rospy.spin()

    def _registerToNode(self, req: ThrottleTopicRequest) -> ThrottleTopicResponse:
        """! Function that register the node to the publisher

        @param req the request of the Node

        @return the responce of the Node
        """
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
        """! Function that forward the message to every subscribed Node

        @param msg the message to send
        """
        for pub in self.publishers:
            pub.publish(msg)
