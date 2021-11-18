from typing import List, Tuple
import rospy
from backbone.srv import ThrottleTopic
from backbone.srv import ThrottleTopicRequest
from backbone.srv import ThrottleTopicResponse


class FrequencyMultiplexer:
    def __init__(self, topic: str, msg) -> None:

        self.topic: str = topic
        self.name: str = f"{topic}Multiplexer"
        self.service: rospy.Service = rospy.Service(
            self.name, ThrottleTopic, self._registerToNode
        )
        self.input: rospy.Subscriber = rospy.Subscriber(
            topic, msg, callback=self._forwardMessages
        )
        self.outputs: List[Tuple[rospy.Publisher, rospy.Time, rospy.Duration]] = []
        self.subscribed: int = 0

    def run(self) -> None:
        rospy.spin()

    def _registerToNode(
        self, req: ThrottleTopicRequest, res: ThrottleTopicResponse
    ) -> bool:
        publish_interval = rospy.Duration(1.0 / req.rate)
        new_topic = f"{self.topic}_{self.subscribed}"
        self.outputs.append(
            (
                rospy.Publisher(new_topic, queue_size=1),
                rospy.Time.now(),
                publish_interval,
            )
        )
        self.subscribed += 1
        res.topic_name = new_topic
        return True

    def _forwardMessage(self, msg) -> None:
        for pub, time_guard, duration in self.outputs:
            if rospy.Time.now() >= time_guard:
                pub.publish(msg)
                time_guard = rospy.Time.now() + duration
