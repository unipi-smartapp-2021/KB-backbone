import rospy
import roslib.message


class VersionChecker:
    def __init__(self, topic, msg_type) -> None:
        self.topic = topic
        self.msg_type = msg_type
        self.msg_class = roslib.message.get_message_class(self.msg_type)
        self.sub = rospy.Subscriber(
            self.topic, self.msg_class, callback=self.check_version, queue_size=1
        )
        self.version = rospy.get_param("version")
        self.version_field = rospy.get_param("VERSION_FIELD")

    def check_version(self, msg) -> None:
        if self.version_field not in msg.__slots__:
            raise WrongVersionException(
                f"backbone/CustomHeader field {self.version_field} not present in msg fields, please remember to include it"
            )
        elif getattr(msg, self.version_field) != self.version:
            raise WrongVersionException(
                f"Got version {getattr(msg, self.version_field)}, expected {self.version}"
            )
        else:
            return None


class WrongVersionException(Exception):
    pass
