#!/usr/bin/env python
# coding=utf-8

import rospy
import hal
from hal_402_device_mgr.msg import msg_error, msg_status
from hal_402_device_mgr.srv import srv_robot_state


class pin_402(object):

    def __init__(self, name, dir, type, bit_pos):
        self.name = name
        self.dir = dir
        self.type = type
        self.bit_pos = bit_pos
        self.halpin = None

    def set_parent_comp(self, component):
        self.parent_comp = component

    def create_halpin(self):
        self.halpin = self.parent_comp.newpin(self.name, self.type, self.dir)


class drive_402(object):

    def __init__(self, drive_name, parent):
        # hal_402_drives_mgr
        self.parent = parent
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
        self.pins_402 = {
            # bits 0-3 and 7 and 8 of the controlword, bit 4-6 and
            # 9 - 15 intentionally not implemented yest
            'switch_on':            pin_402('%s.switch_on'
                                            % self.drive_name,
                                            hal.HAL_OUT,
                                            hal.HAL_BIT,
                                            0),
            'enable_voltage':       pin_402('%s.enable_voltage'
                                            % self.drive_name,
                                            hal.HAL_OUT,
                                            hal.HAL_BIT,
                                            1),
            'quick_stop':           pin_402('%s.quick_stop'
                                            % self.drive_name,
                                            hal.HAL_OUT,
                                            hal.HAL_BIT,
                                            2),
            'enable_operation':     pin_402('%s.enable_operation'
                                            % self.drive_name,
                                            hal.HAL_OUT,
                                            hal.HAL_BIT,
                                            3),
            'fault_reset':          pin_402('%s.fault_reset'
                                            % self.drive_name,
                                            hal.HAL_OUT,
                                            hal.HAL_BIT,
                                            7),
            'halt':                 pin_402('%s.halt'
                                            % self.drive_name,
                                            hal.HAL_OUT,
                                            hal.HAL_BIT,
                                            8),
            # bits in the status word, bit 8 - 15 intentionally
            # not implemented yet
            'ready_to_switch_on':   pin_402('%s.ready_to_switch_on'
                                            % self.drive_name,
                                            hal.HAL_IN,
                                            hal.HAL_BIT,
                                            0),
            'switched_on':          pin_402('%s.switched_on'
                                            % self.drive_name,
                                            hal.HAL_IN,
                                            hal.HAL_BIT,
                                            1),
            'operation_enabled':    pin_402('%s.operation_enabled'
                                            % self.drive_name,
                                            hal.HAL_IN,
                                            hal.HAL_BIT,
                                            2),
            'fault':                pin_402('%s.fault'
                                            % self.drive_name,
                                            hal.HAL_IN,
                                            hal.HAL_BIT,
                                            3),
            'voltage_enabled':      pin_402('%s.voltage_enabled'
                                            % self.drive_name,
                                            hal.HAL_IN,
                                            hal.HAL_BIT,
                                            4),
            # because of duplicity of pin 'quick_stop' of control word
            # this pin is called quick_stop_active
            'quick_stop_active':    pin_402('%s.quick_stop_active'
                                            % self.drive_name,
                                            hal.HAL_IN,
                                            hal.HAL_BIT,
                                            5),
            'switch_on_disabled':   pin_402('%s.switch_on_disabled'
                                            % self.drive_name,
                                            hal.HAL_IN,
                                            hal.HAL_BIT,
                                            6),
            'warning':              pin_402('%s.warning'
                                            % self.drive_name,
                                            hal.HAL_IN,
                                            hal.HAL_BIT,
                                            7)
        }
        self.create_pins()

    def create_pins(self):
        for key, pin in self.pins_402.items():
            pin.set_parent_comp(self.parent.halcomp)
            pin.create_halpin()

    def create_topics(self):
        # for each drive, an error and status topic are created
        # messages are defined in msg/ directory of this package
        self.topics = {
            'error': rospy.Publisher('%s/%s_error' %
                                     (self.parent.compname, self.drive_name),
                                     msg_error,
                                     queue_size=1,
                                     latch=True),
            'status': rospy.Publisher('%s/%s_status' %
                                      (self.parent.compname, self.drive_name),
                                      msg_status,
                                      queue_size=1,
                                      latch=True)
        }

    def test_publisher(self):
        # iterate dict and send a test message
        for key, topic in self.topics.items():
            print(topic)
            message = \
                'This is a testmessage for the {} channel of {}'.format(
                    key, self.drive_name)
            if key == 'error':
                topic.publish(msg_error(message, 0))
            if key == 'status':
                topic.publish(msg_status(message, 'unknown'))

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
        self.compname = 'hal_402_mgr'
        self.drives = dict()

        # create ROS node
        rospy.init_node(self.compname)
        rospy.loginfo("%s: Node started" % self.compname)

        # create HAL userland component
        self.halcomp = hal.component(self.compname)
        rospy.loginfo("%s: HAL component created" % self.compname)

        # create drives which create pins
        self.create_drives()
        self.halcomp.ready()

        self.create_service()
        self.create_publisher()

    def create_drives(self):
        for i in range(0, 6):
            # create 6 drives, later do this from ROS parameters
            drivename = "drive_%s" % (i + 1)
            self.drives[drivename] = drive_402(drivename, self)
            rospy.loginfo("%s: %s created" % (self.compname, drivename))

    def create_publisher(self):
        # todo, read from ROS param server
        self.update_rate = 1  # Hz
        self.rate = rospy.Rate(self.update_rate)
        # create publishers for topics and send out a test message
        for key, drive in self.drives.items():
            drive.create_topics()
            drive.test_publisher()

    def cb_robot_state_service(self, req):
        # the value of the requested state is in req.req_state
        pass

    def cb_test_service_cb(self, req):
        print(req.req_state)  # show abcd in the terminsal
        rospy.loginfo("%s: cb_test_service" % hal_402_drives_mgr.compname)
        # the response that's returned is the return value of the callback
        return "test service return string"

    def create_service(self):
        # $ rosservice call /drives_mgr abcd
        # will call the function callback, that will receive the
        # service message as an argument
        # testin routine:
        # self.service = rospy.Service('drives_mgr',
        #                             srv_robot_state,
        #                             self.cb_test_service)
        self.service = rospy.Service('/%s/drives_mgr' %
                                     self.compname,
                                     srv_robot_state,
                                     self.cb_robot_state_service)
        rospy.loginfo("%s: service %s created" %
                      (self.compname,
                       self.service.resolved_name))

    def inspect_hal_pins(self):
        pass

    def run(self):
        while not rospy.is_shutdown():
            self.inspect_hal_pins()
            self.rate.sleep()


def call_cleanup():
    # need to unload the userland component here?
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
