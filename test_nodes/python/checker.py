#!/usr/bin/env python3

import rospy
from backbone.version_checker import VersionChecker

def main() -> int:
    rospy.init_node("check")
    check = VersionChecker("test","backbone/TestMsg")
    rospy.spin()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())