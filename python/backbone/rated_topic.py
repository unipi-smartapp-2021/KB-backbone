from typing import Dict, List, Optional, TypeVar
from std_msgs.msg import Duration
import rospy
from backbone.rated_publisher import RatedPublisher
from backbone.srv import RateTopic, RateTopicRequest, RateTopicResponse

T = TypeVar("T")


class RatedTopic:
    """! Class that define a Multiplexxing Publisher that send at a given frequency"""

    def __init__(self, topic: str, data_class: T, rates: List[int]) -> None:
        """! Construction of the class

        @param topic the topic name of the publisher
        @param msg the message type we are going to send
        @return None
        """
        self.data_class: T = data_class
        self.name: str = f"{topic}Rated"
        self.service: rospy.Service = rospy.Service(
            self.name, RateTopic, self._registerToNode
        )
        self.input: rospy.Subscriber = rospy.Subscriber(
            topic, self.data_class, callback=self._forwardMessages
        )
        self.debugger = rospy.Publisher(
            f"{self.name}Delay", Duration, queue_size=1
        )
        if len(rates) == 0:
            pass
        elif not all(rates[i] < rates[i + 1] for i in range(len(rates) - 1)):
            pass
        else:

            self.publishers: Dict[RatedPublisher] = {
                rate:RatedPublisher(rospy.Publisher(f"{self.name}{rate}Hz",data_class=self.data_class,queue_size=1),rate)
                for rate in rates
            }

    def run(self) -> None:
        """! Base function that start the thread"""
        rospy.spin()

    def _registerToNode(self, req: RateTopicRequest) -> Optional[RateTopicResponse]:
        """! Function that register the node to the publisher

        @param req the request of the Node

        @return the response of the Node
        """
        if req.rate > max(self.publishers.keys()):
            return None
        else:
            new_topic = self.publishers[req.rate].get_topic()
            res = RateTopicResponse(new_topic)
        return res

    def _forwardMessages(self, msg: T) -> None:
        """! Function that forward the message to every subscribed Node and computes the delay 
        in message delivery if there are debugging nodes
        @param msg the message to send
        """
        if self.debugger.get_num_connections() == 0:
            for _, pub in self.publishers.items():
                pub.publish(msg)
        else:
            total_delay = rospy.Duration(0,0)
            forwarded = 0
            for _,pub in self.publishers.items():
                delay = pub.publish_and_delay(msg)
                if delay and delay.to_nsec() >= 0:
                    total_delay += delay
                    forwarded += 1
                
                if forwarded > 0:
                    dbg = Duration()
                    dbg.data = rospy.Duration(nsecs=total_delay.to_nsec() / forwarded)
                    self.debugger.publish(dbg)