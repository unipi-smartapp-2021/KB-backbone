from typing import Optional, TypeVar

import rospy

T = TypeVar("T")


class RatedPublisher:
    """! The class of the Publisher that use a frequency rate

    """
    def __init__(self, publisher: rospy.Publisher, rate: int) -> None:
        """! Constructor of the class

        @param publisher the publisher we are trasforming in a rated one
        @param rate the frequency rate at wich we want to update our subscribers
        """
        self.publisher = publisher
        self.interval = rospy.Duration(1.0 / rate)
        self.next_update = rospy.Time()

    def get_num_subscribers(self) -> int:
        """! Function that tell how many subscribers we have

        @return the number of subcribers that are connected to us
        """
        return self.publisher.get_num_connections()

    def get_topic(self) -> str:
        """! Function that show the name of the topic we publish

        @return the name of the topic we publish
        """
        return self.publisher.name

    def publish(self, msg: T) -> bool:
        """! Function that publish the message at a given rate

        @param msg the message we want to publish

        @return True if we publish with success
        @return False if we don't have any subscribers
        @return False if it's not time for the update yet
        """
        now = rospy.Time.now()
        if self.get_num_subscribers() == 0 or now < self.next_update:
            return False
        else:
            self.publisher.publish(msg)
            self.next_update = now + self.interval
            return True
   
    def publish_and_delay(self,msg: T) -> Optional[rospy.Duration]:
        now = rospy.Time.now()
        if self.get_num_subscribers() == 0 or now < self.next_update:
            return None
        else:
            self.publisher.publish(msg)
            delay = rospy.Duration(0,0) if self.next_update.to_nsec() == 0 else now -self.next_update
            self.next_update = now + self.interval
            return delay