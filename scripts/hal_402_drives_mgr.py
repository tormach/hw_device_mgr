#!/usr/bin/env python
# coding=utf-8

import rospy
import hal


class drive_402(object):

    def __init__(self, drive_name):
        # bitmask and value
        self.drive_name = drive_name
        self.states_402 = {
            'NOT READY TO SWITCH ON':   [0x4F, 0x00],
            'SWITCH ON DISABLED':       [0x4F, 0x40],
            'READY TO SWITCH ON':       [0x6F, 0x21],
            'SWITCHED ON':              [0x6F, 0x23],
            'OPERATION ENABLED':        [0x6F, 0x27],
            'FAULT':                    [0x4F, 0x08],
            'FAULT REACTION ACTIVE':    [0x4F, 0x0F],
            'QUICK STOP ACTIVE':        [0x6F, 0x07]
        }

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

    def __init__(self):
        self.compname = 'hal_402_drives_mgr'
        self.drives = dict()

        # create ROS node
        rospy.init_node(self.compname)
        rospy.loginfo("%s: Node started" % self.compname)

        # create HAL userland component
        self.halcomp = hal.component(self.compname)
        rospy.loginfo("%s: HAL component created" % self.compname)

        # create drives
        self.create_drives()
        self.create_service()
        self.create_publisher()

    def create_drives(self):
        for i in range(0, 6):
            # create 6 drives, later do this from ROS parameters
            drivename = "drive_%s" % (i + 1)
            self.drives[drivename] = drive_402(drivename)
            rospy.loginfo("%s: %s created" % (self.compname, drivename))

    def create_publisher(self):
        pass

    def create_service(self):
        pass

    def inspect_hal_pins(self):
        pass

    def run(self):
        while not rospy.is_shutdown():
            self.inspect_hal_pins()


def call_cleanup():
    rospy.loginfo("%s: Stopping ..." % hal_402_drives_mgr.compname)
    rospy.loginfo("%s: Stopped" % hal_402_drives_mgr.compname)


if __name__ == '__main__':
    # Create and name node
    hal_402_drives_mgr = hal_402_drives_mgr()
    rospy.on_shutdown(call_cleanup)

    try:
        hal_402_drives_mgr.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("%s: ROSInterruptException"
                      % hal_402_drives_mgr.compname)
