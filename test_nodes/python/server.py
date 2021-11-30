#!/usr/bin/env python3

from backbone.rated_topic import RatedTopic
import rospy
from std_msgs.msg import String


def main() -> int:
    rospy.init_node("freq_multiplexer")

    m = RatedTopic("test_topic", String, rates=[6,10])
    m.run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
