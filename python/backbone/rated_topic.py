from typing import Dict, List, Optional, TypeVar
import bisect
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
        self.original_topic = topic
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
        assert len(rates) > 0, "Please provide a non empty list"
        assert all(rates[i] < rates[i + 1] for i in range(len(rates) - 1)), "Please provide a sorted list"
        self.publishers: Dict[RatedPublisher] = {
                rate:RatedPublisher(rospy.Publisher(f"{self.name}{rate}Hz",data_class=self.data_class,queue_size=1),rate)
                for rate in rates
            }

    def _registerToNode(self, req: RateTopicRequest) -> Optional[RateTopicResponse]:
        """! Function that register the node to the publisher

        @param req the request of the Node

        @return the response of the Node
        """
        print(self.publishers.keys())
        rates = list(self.publishers.keys())
        print(rates)
        max_rate =  max(rates)
        res = RateTopicResponse()
        if req.rate <= 0:
            rospy.logerr(f"Got request rate {req.rate} Hz, please send a positive rate")
            return None
        elif req.rate < rates[0]:
            rospy.logerr(f"Got request rate {req.rate} Hz, the minimum frequency supported is {rates[0]} Hz")
            return None
        elif req.rate >= rates[-1]:
            res.topic  = self.publishers[rates[-1]].get_topic()
            return res
        elif req.rate in rates:
            res.topic = self.publishers[req.rate].get_topic()
            return res
        else:
            idx = bisect.bisect_left(rates, req.rate)
            rate = rates[idx-1]
            res.topic = self.publishers[rate].get_topic()
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