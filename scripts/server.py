from topic_control.frequency_multiplexer import FrequencyMultiplexer
import rospy
from std_msgs.msg import String


def main() -> int:
    m = FrequencyMultiplexer("test_topic", String)
    m.run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
