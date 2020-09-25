import rospy
import hal
import time

# import messages from the ROS node
from hal_402_device_mgr.msg import msg_error, msg_status


class GenericHalPin:
    def __init__(self, name, dir, type):
        self.name = name
        self.dir = dir
        self.type = type
        self.halpin = None
        self.local_pin_value = None

    def set_parent_comp(self, component):
        self.parent_comp = component

    def create_halpin(self):
        self.halpin = self.parent_comp.newpin(self.name, self.type, self.dir)

    def set_local_value(self, value):
        self.local_pin_value = value

    def set_hal_value(self):
        self.halpin.set(self.local_pin_value)

    def get_hal_value(self):
        self.local_pin_value = self.halpin.get()

    def sync_hal(self):
        if self.dir == hal.HAL_OUT:
            self.set_hal_value()
        else:
            self.get_hal_value()


class Pin402(GenericHalPin):
    def __init__(self, name, dir, type, bit_pos):
        super().__init__(name, dir, type)
        self.bit_pos = bit_pos


class StateMachine402:
    states_402 = {
        'NOT READY TO SWITCH ON': [0x4F, 0x00],
        'SWITCH ON DISABLED': [0x4F, 0x40],
        'READY TO SWITCH ON': [0x6F, 0x21],
        'SWITCHED ON': [0x6F, 0x23],
        'OPERATION ENABLED': [0x6F, 0x27],
        'FAULT': [0x4F, 0x08],
        'FAULT REACTION ACTIVE': [0x4F, 0x0F],
        'QUICK STOP ACTIVE': [0x6F, 0x07],
    }
    # this dict shows what the next state should be when from a current
    # state. Follow these steps until the state 'OPERATION ENABLED' has
    # been reached. The vaulue of the dict entry contains a list of the
    # next state, as well as the transition key. NA means "Not
    # Available" because transition happens automatically by the device.
    # These are kept around for keeping the complete picture.
    #
    # NOTE: the "path_to_xxx" dicts must have an entry for every possible drive state.
    path_to_operation_enabled = {
        'NOT READY TO SWITCH ON': ['SWITCH ON DISABLED', 'WAIT'],
        'SWITCH ON DISABLED': ['READY TO SWITCH ON', 'TRANSITION_2'],
        'READY TO SWITCH ON': ['SWITCHED ON', 'TRANSITION_3'],
        'SWITCHED ON': ['OPERATION ENABLED', 'TRANSITION_4'],
        'OPERATION ENABLED': ['OPERATION ENABLED', ''],  # End state
        'FAULT': ['SWITCH ON DISABLED', 'TRANSITION_15'],
        'FAULT REACTION ACTIVE': ['FAULT', 'WAIT'],
        'QUICK STOP ACTIVE': ['SWITCH ON DISABLED', 'WAIT'],
    }
    # these transitions take longer from OPERATION ENABLED -> SWITCH ON DISABLED
    # 'OPERATION ENABLED':        ['SWITCHED ON', 'TRANSITION_5'],
    # 'SWITCHED ON':              ['READY TO SWITCH ON', 'TRANSITION_6'],
    # 'READY TO SWITCH ON':       ['SWITCH ON DISABLED', 'TRANSITION_7']
    # Note that almost all cases are single-step except for the fault cases
    path_to_switch_on_disabled = {
        'NOT READY TO SWITCH ON': ['SWITCH ON DISABLED', 'WAIT'],
        'SWITCH ON DISABLED': ['SWITCH ON DISABLED', ''],  # End State
        'READY TO SWITCH ON': ['SWITCH ON DISABLED', 'TRANSITION_7'],
        'SWITCHED ON': ['SWITCH ON DISABLED', 'TRANSITION_10'],
        'OPERATION ENABLED': ['SWITCH ON DISABLED', 'TRANSITION_9'],
        'FAULT': ['SWITCH ON DISABLED', 'TRANSITION_15'],
        'FAULT REACTION ACTIVE': ['FAULT', 'WAIT'],
        'QUICK STOP ACTIVE': ['SWITCH ON DISABLED', 'WAIT'],
    }

    path_on_fault = {
        'NOT READY TO SWITCH ON': ['SWITCH ON DISABLED', 'WAIT'],
        'SWITCH ON DISABLED': ['SWITCH ON DISABLED', ''],  # End state
        'READY TO SWITCH ON': ['SWITCH ON DISABLED', 'TRANSITION_7'],
        'SWITCHED ON': ['SWITCH ON DISABLED', 'TRANSITION_10'],
        'OPERATION ENABLED': ['SWITCH ON DISABLED', 'TRANSITION_9'],
        'FAULT': ['FAULT', ''],  # End state
        'FAULT REACTION ACTIVE': ['FAULT', 'WAIT'],
        'QUICK STOP ACTIVE': ['SWITCH ON DISABLED', 'WAIT'],
    }

    # the transition dict contains a list of tuples with bits and value
    # to can be set and reset.
    transitions = {
        'TRANSITION_2': [
            ('switch-on', 0),
            ('enable-voltage', 1),
            ('quick-stop', 1),
            ('enable-operation', 0),
        ],
        'TRANSITION_3': [('switch-on', 1)],
        'TRANSITION_4': [('enable-operation', 1)],
        'TRANSITION_5': [('enable-operation', 0)],
        'TRANSITION_6': [('quick-stop', 0), ('enable-voltage', 0)],
        'TRANSITION_7': [
            ('enable-operation', 0),
            ('quick-stop', 0),
            ('enable-voltage', 0),
        ],
        'TRANSITION_8': [('switch-on', 0)],
        'TRANSITION_9': [
            ('enable-operation', 0),
            ('quick-stop', 0),
            ('enable-voltage', 0),
            ('switch-on', 0),
        ],
        'TRANSITION_10': [
            ('quick-stop', 0),
            ('enable-voltage', 0),
            ('switch-on', 0),
        ],
        'TRANSITION_15': [
            ('enable-operation', 0),
            ('quick-stop', 0),
            ('enable-voltage', 0),
            ('switch-on', 0),
            ('fault-reset', 1),
            ('wait', 0.01),
            ('fault-reset', 0),
        ],
        'WAIT': [],
    }


class Drive402:
    GENERIC_ERROR_DESCRIPTION = 'This is an unknown error'
    GENERIC_ERROR_SOLUTION = (
        'Please consult the troubleshooting section of your hardware manual'
    )
    pins_402_spec = [
        # Pin tuple format:
        #   (name, hal_dir, hal_type, bit_num)
        #
        # bits 0-3 and 7 and 8 of the controlword, bit 4-6 and
        # 9 - 15 intentionally not implemented yest
        ('switch-on', hal.HAL_OUT, hal.HAL_BIT, 0),
        ('enable-voltage', hal.HAL_OUT, hal.HAL_BIT, 1),
        ('quick-stop', hal.HAL_OUT, hal.HAL_BIT, 2),
        ('enable-operation', hal.HAL_OUT, hal.HAL_BIT, 3),
        ('fault-reset', hal.HAL_OUT, hal.HAL_BIT, 7),
        ('halt', hal.HAL_OUT, hal.HAL_BIT, 8),
        # the status word pins are not connected anymore
    ]

    def __init__(self, drive_name, drive_type, parent, slave_inst):
        # hal_402_drives_mgr
        self.slave_inst = slave_inst
        self.sim = rospy.get_param("/sim_mode", True)
        self.parent = parent
        self.drive_name = drive_name
        self.drive_type = drive_type
        # bitmask and value
        self.prev_state = 'unknown'
        self.curr_state = 'unknown'
        self.prev_status_word = 0
        self.curr_status_word = 0
        self.prev_error = 0
        self.curr_error_code = 0
        self.active_transition_table = None
        self.pins_402 = dict()
        for pname, pdir, ptype, ppos in self.pins_402_spec:
            self.pins_402[pname] = Pin402(
                f'{self.drive_name}.{pname}', pdir, ptype, ppos
            )
        self.pins_generic = {
            # Pins used by this component
            'error-code': GenericHalPin(
                '%s.error-code' % self.drive_name, hal.HAL_IN, hal.HAL_U32
            ),
            'status-word': GenericHalPin(
                '%s.status-word' % self.drive_name, hal.HAL_IN, hal.HAL_U32
            ),
        }
        self.all_pins = {
            # create dict holding the 2 different classes of pins
            'pins_402': self.pins_402,
            'pins_generic': self.pins_generic,
        }
        self.create_pins()

    def sim_set_status(self, status):
        # bitmask = StateMachine402.states_402[status][0]
        # set pins according entire control word value
        # effectively also resetting pins that are not in bitmask
        statusword = StateMachine402.states_402[status][1]
        self.pins_generic['status-word'].set_local_value(statusword)
        self.pins_generic['status-word'].set_hal_value()
        # give HAL at least 1 cycle to process
        time.sleep(0.002)

    def set_transition_table(self, transition_table):
        self.active_transition_table = transition_table

    def print_debuginfo(self):
        rospy.logwarn(
            "%s: %s encountered statusword: \'%s\', with value: %s"
            % (
                self.parent.compname,
                self.drive_name,
                self.curr_state,
                self.curr_status_word,
            )
        )

    def is_transitionable(self, transition=None):
        transition = (
            transition or self.active_transition_table[self.curr_state][1]
        )
        if transition == 'WAIT':
            return False
        else:
            return True

    def next_transition(self):
        # firstly, we need to wait on a state which has a valid
        # next transition. For example, when a drive is starting up
        # it can be in 'NOT READY TO SWITCH ON' state, from where we
        # need to wait until the drive itself has gotten out of this
        # state.
        #
        # so grab the list containing the next state and next transition
        # if the transition equals 'WAIT' we can't do anything but wait.
        # so return a False to the calling function

        # when called:
        # - look up the self.active_transition_table[self.curr_state]
        # - get transition list[1]
        try:
            next_state = self.active_transition_table.get(self.curr_state, None)
            if next_state is None or len(next_state) < 2:
                rospy.logwarn(
                    "No transition registered for state %s in current transition table"
                    % (self.curr_state,)
                )
                return False
            transition = next_state[1]
            return self.do_transition(transition)
        except KeyError:
            self.print_debuginfo()
            return False

    def do_transition(self, transition):
        if self.is_transitionable(transition):
            # - in sim mode, get the the next state to mimic input pin changes
            next_state = self.active_transition_table[self.curr_state][0]
            rospy.logdebug(
                "%s, attempting transition %s to state %s"
                % (self.drive_name, transition, next_state,)
            )
            # - look up transition in transition_table
            # - get list with tuples containing pin and value to be set
            change_pins_list = StateMachine402.transitions[transition]
            # - for each tuple from list, set pin and value
            for pin_change in change_pins_list:
                # KLUDGE super-secret hal pin for timing
                if pin_change[0] == 'wait':
                    time.sleep(pin_change[1])
                    continue

                self.change_halpin(pin_change)
                # for simulation purpose, set input pins manually according to
                # the next state as if the drive is attached
                if self.sim is True:
                    self.sim_set_status(next_state)
            return True
        # sometimes (as proposed by zultron) we seem to have a race condition where
        # after a read of the state bits, the state is 'unknown' and then the next
        # read will provide a correct status word. If that status word is the
        # target we try to reach, the elif below will return a true so the while
        # loop in hal_402_mgr.py, line 221 can sucessfully exit
        elif (
            self.curr_state == self.active_transition_table[self.curr_state][0]
        ):
            rospy.loginfo(
                "%s: transition %s apparently reached state %s"
                % (self.drive_name, transition, self.curr_state,)
            )
            # current state equals the target state, mimic succesful transition
            return True
        else:
            return False

    def create_pins(self):
        for k, pin_dict in self.all_pins.items():
            for key, pin in pin_dict.items():
                pin.set_parent_comp(self.parent.halcomp)
                pin.create_halpin()

    def create_topics(self):
        # for each drive, an error and status topic are created
        # messages are defined in msg/ directory of this package
        self.topics = {
            'error': rospy.Publisher(
                f'{self.parent.compname}/{self.drive_name}_error',
                msg_error,
                queue_size=1,
                latch=True,
            ),
            'status': rospy.Publisher(
                f'{self.parent.compname}/{self.drive_name}_status',
                msg_status,
                queue_size=1,
                latch=True,
            ),
        }

    def test_publisher(self):
        # iterate dict and send a test message
        for key, topic in self.topics.items():
            message = 'This is a testmessage for the {} channel of {}'.format(
                key, self.drive_name
            )
            if key == 'error':
                topic.publish(
                    msg_error(
                        self.drive_name,
                        self.drive_type,
                        'test: no code',
                        'test: no description',
                        'test: no solution',
                    )
                )
            if key == 'status':
                topic.publish(msg_status(message, 'unknown'))

    def read_halpins(self):
        # get all the status pins, and save their value locally
        self.prev_error = self.curr_error_code
        self.prev_state = self.curr_state
        for k, pin_dict in self.all_pins.items():
            for key, pin in pin_dict.items():
                if pin.dir == hal.HAL_IN:
                    pin.sync_hal()
        # Hex value reported from drive
        self.curr_error_code = self.pins_generic['error-code'].local_pin_value

    def calculate_status_word(self):
        # traverse dict and for the local values do some bitwise operation so
        # that these input pins build up the current status word. The status
        # word will be used for determining the 402 profile drive state.
        self.prev_status_word = self.curr_status_word
        self.curr_status_word = self.pins_generic['status-word'].local_pin_value

    def status_word_changed(self):
        if not (self.prev_status_word == self.curr_status_word):
            return True
        else:
            return False

    def calculate_state(self):
        for key, state in StateMachine402.states_402.items():
            # check if the value after applying the bitmask (value[0])
            # corresponds with the value[1] to determine the current status
            bitmaskvalue = self.curr_status_word & state[0]
            # print('--- %s' % key)
            # print('{:#010b} bitmask'.format(state[0]))
            # print('{:#010b} value'.format(state[1]))
            # print('{:#010b} curr_status_word'.format(self.curr_status_word))
            # print('{:#010b} bitmaskvalue'.format(bitmaskvalue))
            # print('bitmaskvalue == state[1] : %s' % (bitmaskvalue == state[1]))
            if bitmaskvalue == state[1]:
                self.curr_state = key
                # exit when we've reached correct state
                break
            else:
                self.curr_state = 'unknown'

    def update_state(self):
        self.read_halpins()
        self.calculate_status_word()
        self.calculate_state()
        self.publish_state()
        self.publish_fault_state()
        self.publish_error()

    def publish_state(self):
        if self.drive_state_changed():
            self.topics['status'].publish(self.drive_name, self.curr_state)

    def change_halpin(self, pin_change):
        name, val = pin_change
        pin = self.pins_402[name]  # pick the pin from the list
        pin.set_local_value(val)  # and set local value accordingly
        pin.sync_hal()  # then sync the pin local value with HAL

    def drive_state_changed(self):
        # only publish drive status if the status has changed
        if not (self.prev_state == self.curr_state):
            return True
        else:
            return False

    def drive_error_changed(self):
        # only publish drive status if the error has changed
        if not (self.prev_error == self.curr_error_code):
            return True
        else:
            return False

    @staticmethod
    def error_code_as_str(error_code):
        return "0x{:04x}".format(error_code) if error_code else ''

    def current_error_code_str(self):
        return self.error_code_as_str(self.curr_error_code)

    def publish_error(self):
        if self.drive_error_changed():
            err_str = self.current_error_code_str()
            error_info = self.parent.get_error_info(
                self.drive_type, self.curr_error_code
            )
            self.topics['error'].publish(
                self.drive_name,
                self.drive_type,
                err_str,
                error_info.get(
                    'description', Drive402.GENERIC_ERROR_DESCRIPTION
                ),
                error_info.get('solution', Drive402.GENERIC_ERROR_SOLUTION),
            )
            if not self.curr_error_code:
                rospy.loginfo(
                    f"{self.parent.compname}: {self.drive_name} no error"
                )
            else:
                rospy.logerr(
                    '{}: {}, error number: {}, description: {}, solution: {}'.format(
                        self.parent.compname,
                        self.drive_name,
                        err_str,
                        error_info.get(
                            'description', self.GENERIC_ERROR_DESCRIPTION
                        ),
                        error_info.get('solution', self.GENERIC_ERROR_SOLUTION),
                    )
                )

    def publish_fault_state(self):
        if (self.curr_state == 'FAULT') and self.drive_state_changed():
            rospy.logerr(
                "%s: %s entered \'FAULT\' state"
                % (self.parent.compname, self.drive_name)
            )
