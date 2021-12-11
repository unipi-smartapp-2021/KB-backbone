import rospy
import roslib.message


class VersionChecker:
    def __init__(self, topic, msg_type) -> None:
        self.topic = topic
        self.msg_type = msg_type
        self.msg_class = roslib.message.get_message_class(self.msg_type)
        self.sub = rospy.Subscriber(
            f"/{self.topic}", self.msg_class, callback=self.check_version, queue_size=1
        )
        self.version = rospy.get_param(f"{self.topic}_version")
    def check_version(self, msg) -> None:
        if "version" not in msg.__slots__:
            raise WrongVersionException(
                f"Field version not present in msg fields, please remember to include it"
            )
        elif msg.version != self.version:
            raise WrongVersionException(
                f"Got version {msg.version}, expected {self.version}"
            )
        else:
            return None


class WrongVersionException(Exception):
    pass
