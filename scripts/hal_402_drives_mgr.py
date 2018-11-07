#!/usr/bin/env python
# coding=utf-8

import rospy


class drive_402(object):
    # bitmask and value
    states_402 = {
        'NOT READY TO SWITCH ON':   [0x4F, 0x00],
        'SWITCH ON DISABLED':       [0x4F, 0x40],
        'READY TO SWITCH ON':       [0x6F, 0x21],
        'SWITCHED ON':              [0x6F, 0x23],
        'OPERATION ENABLED':        [0x6F, 0x27],
        'FAULT':                    [0x4F, 0x08],
        'FAULT REACTION ACTIVE':    [0x4F, 0x0F],
        'QUICK STOP ACTIVE':        [0x6F, 0x07]
    }

    def __init__(self, drive_name):
        pass

    def create_pins():
        pass

    def create_topics():
        pass

    def get_halpins():
        pass

    def set_halpins():
        pass

    def enable_drive():
        pass

    def disable_drive():
        pass

    def publish_drive_status():
        pass

    def publish_drive_error():
        pass


class hal_402_drives_mgr(object):
    compname = 'hal_402_drives_mgr'
    drives = dict()

    def __init__(self):
        pass

    def run(self):
        while not rospy.is_shutdown():
            pass


if __name__ == '__main__':
    # Create and name node
    hal_402_drives_mgr = hal_402_drives_mgr()
    try:
        hal_402_drives_mgr.run()
    except rospy.ROSInterruptException:
        pass
