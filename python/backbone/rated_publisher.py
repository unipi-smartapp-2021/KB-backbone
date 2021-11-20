from typing import TypeVar

import rospy

T = TypeVar("T")


class RatedPublisher:
    def __init__(self, publisher: rospy.Publisher, rate: int) -> None:
        self.publisher = publisher
        self.interval = 1.0 / rate
        self.next_update = rospy.get_time()

    def get_num_subscribers(self) -> int:
        return self.publisher.get_num_connections()

    def get_topic(self) -> str:
        return self.publisher.name

    def publish(self, msg: T) -> bool:
        now = rospy.get_time()
        if self.get_num_subscribers() == 0 or now < self.next_update:
            return False
        else:
            self.publisher.publish(msg)
            self.next_update = now + self.interval
            return True
