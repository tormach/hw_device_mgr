import rospy
import hal
# import messages from the ROS node
from hal_402_device_mgr.msg import msg_error, msg_status


class pin_402(object):

    def __init__(self, name, dir, type, bit_pos):
        self.name = name
        self.dir = dir
        self.type = type
        self.bit_pos = bit_pos
        self.halpin = None
        self.local_pin_value = None

    def set_parent_comp(self, component):
        self.parent_comp = component

    def create_halpin(self):
        self.halpin = self.parent_comp.newpin(self.name, self.type, self.dir)

    def set_local_value(self):
        # put HAL pin value in lacal_pin_value
        self.local_pin_value = self.halpin.get()


class drive_402(object):

    def __init__(self, drive_name, parent):
        # hal_402_drives_mgr
        self.parent = parent
        # bitmask and value
        self.drive_name = drive_name
        self.prev_state = 'unknown'
        self.curr_state = 'unknown'
        self.prev_status_word = 0
        self.curr_status_word = 0
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
            message = \
                'This is a testmessage for the {} channel of {}'.format(
                    key, self.drive_name)
            if key == 'error':
                topic.publish(msg_error(message, 0))
            if key == 'status':
                topic.publish(msg_status(message, 'unknown'))

    def read_halpins(self):
        # get all the status pins, and save their value locally
        for key, pin in self.pins_402.items():
            if pin.dir == hal.HAL_IN:
                pin.set_local_value()

    def calculate_status_word(self):
        # traverse dict and for the local values do some
        # bitwise operation so that these input pins build
        # up the current status word. The status word will
        # be used for determining the 402 profile drive state
        self.prev_status_word = self.curr_status_word
        self.curr_status_word = 0
        for key, pin in self.pins_402.items():
            if pin.dir == hal.HAL_IN:
                self.curr_status_word = (self.curr_status_word |
                                         (pin.local_pin_value << pin.bit_pos))

    def calculate_state(self):
        self.prev_state = self.curr_state
        for key, state in self.states_402.items():
            # check if the value after applying the bitmask (value[0])
            # corresponds with the value[1] to determine the current status
            bitmaskvalue = self.curr_status_word & state[0]
            print('--- %s' % key)
            print('{:#010b} '.format(state[0]))
            print('{:#010b} '.format(state[1]))
            print('{:#010b} curr_status_word'.format(self.curr_status_word))
            print('{:#010b} bitmaskvalue'.format(bitmaskvalue))
            if (bitmaskvalue == state[1]):
                self.curr_state = key
            else:
                self.curr_state = 'unknown'

    def publish_state(self):
        if self.drive_state_changed():
            self.topics['status'].publish(self.drive_name, self.curr_state)

    def set_halpins():
        pass

    def enable_drive():
        pass

    def disable_drive():
        pass

    def drive_state_changed(self):
        # only publish drive status if the status has changed
        if (self.prev_state != self.curr_state):
            return True
        else:
            return False

    def publish_drive_error():
        pass
