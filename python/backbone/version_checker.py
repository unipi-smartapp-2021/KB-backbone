import rospy
import roslib.message


class VersionChecker:
    def __init__(self, topic, msg_type,max_msgs_without_err = 1) -> None:
        self.topic = topic
        self.msg_type = msg_type
        self.msg_class = roslib.message.get_message_class(self.msg_type)
        self.sub = rospy.Subscriber(
            f"{self.topic}", self.msg_class, callback=self.check_version, queue_size=1
        )
        self.version = rospy.get_param(f"{self.topic}_version")
        self.count_flag: int = 0
        self.max_msgs_without_err: int = max_msgs_without_err
    
    def check_version(self, msg) -> None:
        if "version" not in msg.__slots__:
            raise NoVersionFieldException(
                f"Field version not present in msg fields, please remember to include it"
            )
        elif msg.version != self.version:
            if self.count_flag == 0:
                rospy.logerr(f"Got version {msg.version} of {self.msg_type}, expected version {self.version}")
            self.count_flag  = (self.count_flag + 1) % self.max_msgs_without_err
            

        else:
            return None


class NoVersionFieldException(Exception):
    pass
