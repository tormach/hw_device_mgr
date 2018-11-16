#!/usr/bin/env python
# coding=utf-8

import rospy
from hal_402_mgr import hal_402_mgr


def call_cleanup():
    # need to unload the userland component here?
    rospy.loginfo("%s: Stopping ..." % hal_402_drives_mgr.compname)
    rospy.loginfo("%s: Stopped" % hal_402_drives_mgr.compname)


if __name__ == '__main__':
    # Create and name node
    hal_402_drives_mgr = hal_402_mgr()
    rospy.on_shutdown(call_cleanup)

    try:
        hal_402_drives_mgr.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("%s: ROSInterruptException"
                      % hal_402_drives_mgr.compname)
