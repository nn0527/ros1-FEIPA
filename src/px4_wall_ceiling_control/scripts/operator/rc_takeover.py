#!/usr/bin/env python3
"""One-shot request; flight control continues independently of this process."""
import sys
import rospy
from std_srvs.srv import Trigger


def main():
    rospy.init_node('request_rc_takeover', anonymous=True)
    try:
        rospy.wait_for_service('/attachment/rc_takeover', timeout=3.)
        reply = rospy.ServiceProxy('/attachment/rc_takeover', Trigger)()
        print(reply.message)
        return 0 if reply.success else 1
    except (rospy.ROSException, rospy.ServiceException) as error:
        print('接管请求失败：' + str(error), file=sys.stderr)
        return 1


if __name__ == '__main__':
    sys.exit(main())
