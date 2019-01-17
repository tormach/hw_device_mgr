# -*- coding: utf-8 -*-
import rospy
import time
from machinekit import hal as mk_hal
import hal
import os

# import service messages from the ROS node
from hal_402_device_mgr.srv import srv_robot_state
from hal_402_drive import Drive402
from hal_402_drive import GenericHalPin
from hal_402_drive import StateMachine402


class Hal402Mgr(object):
    def __init__(self):
        # Configure sim mode if SIM environment variable is set
        self.sim = os.environ.get("SIM", "false").lower() != "false"

        self.compname = 'hal_402_mgr'
        self.drives = dict()
        self.prev_robot_state = 'unknown'
        self.curr_robot_state = 'unknown'
        self.prev_hal_state_cmd = 0
        self.curr_hal_state_cmd = 0
        # very simple transitions for now
        # unknown -> stopped
        # error -> stopped
        # stopped -> error
        # stopped -> started
        # started -> error
        self.states = {
            'unknown': [
                StateMachine402.path_to_switch_on_disabled,
                'SWITCH ON DISABLED',
            ],
            'started': [
                StateMachine402.path_to_operation_enabled,
                'OPERATION ENABLED',
            ],
            'stopped': [
                StateMachine402.path_to_switch_on_disabled,
                'SWITCH ON DISABLED',
            ],
            'error': [
                StateMachine402.path_to_switch_on_disabled,
                'SWITCH ON DISABLED',
            ],
        }
        self.pins = {
            # Pins used by this component to call the service callbacks
            'state-cmd': GenericHalPin(
                '%s.state-cmd' % self.compname, hal.HAL_IN, hal.HAL_U32
            ),
            'state-fb': GenericHalPin(
                '%s.state-fb' % self.compname, hal.HAL_IN, hal.HAL_S32
            ),
        }
        self.conv_value_to_state = {
            '-2': 'unknown',
            '-1': 'error',
            '0': 'stopped',
            '1': 'started',
        }
        # create ROS node
        rospy.init_node(self.compname)
        rospy.loginfo("%s: Node started" % self.compname)

        # create HAL userland component
        self.halcomp = hal.component(self.compname)
        rospy.loginfo("%s: HAL component created" % self.compname)

        # create drives which create pins
        self.create_drives()
        # create pins for calling service callback
        self.create_pins()
        # done
        self.halcomp.ready()

        # check if we're running real hardware, and properly
        # - set up drives
        # - connect pins and signals
        self.check_for_real_hardware_setup()

        self.create_service()
        self.create_publisher()

    def has_parameters(self, list_of_parameters):
        has_parameters = True
        for parameter in list_of_parameters:
            has_parameters = rospy.has_param(parameter)
            if has_parameters is False:
                # exit this list at first missing parameter
                break
        return has_parameters

    def check_for_real_hardware_setup(self):
        if self.sim:
            self.sim_set_drives_status('SWITCH ON DISABLED')
            rospy.loginfo(
                "%s: no hardware setup detected, default to "
                "simulation mode" % self.compname
            )
        else:
            # Configure real hardware mode
            rospy.loginfo("%s: hardware setup detected" % self.compname)
        # check for parameter existence
        if not self.has_parameters(
            [
                '/hal_402_device_mgr/slaves/name',
                '/hal_402_device_mgr/slaves/instances',
                '/hal_402_device_mgr/slaves/wait_timeout',
                '/hal_402_device_mgr/slaves/wait_on_pinname',
            ]
        ):
            rospy.logerr(
                "%s: no correct /hal_402_device_mgr/slaves params"
                % self.compname
            )
            return

        # get parameters
        self.slaves_name = rospy.get_param('/hal_402_device_mgr/slaves/name')
        self.slaves_instances = rospy.get_param(
            '/hal_402_device_mgr/slaves/instances'
        )
        self.wait_on_pinname = rospy.get_param(
            '/hal_402_device_mgr/slaves/wait_on_pinname'
        )
        last_nr = self.slaves_instances[-1]
        self.timeout = rospy.get_param(
            '/hal_402_device_mgr/slaves/wait_timeout'
        )
        pin_name = '%s.%s.%s' % (
            self.slaves_name,
            last_nr,
            self.wait_on_pinname,
        )
        # check for pin, result -1 then timeout, thus no pin
        if not self.sim and not self.check_hal(pin=pin_name):
            # pin not found
            rospy.logerr("%s: pin %s not available" % (self.compname, pin_name))
        else:
            # we've recognized a pin within the local_timeout
            self.connect_pins_and_signals()

    def check_hal(self, pin=None, signal=None):
        local_timeout = self.timeout
        time_incr = 0.25  # seconds
        if pin is not None:
            (obj_name, hal_objs, obj_type) = (pin, mk_hal.pins, 'pin')
        else:
            (obj_name, hal_objs, obj_type) = (signal, mk_hal.signals, 'signal')
        while obj_name not in hal_objs and local_timeout > 0:
            time.sleep(time_incr)
            local_timeout -= time_incr
        if local_timeout <= 0:
            # a local_timeout occured
            rospy.logerr(
                "%s: %s %s not found after %s seconds"
                % (self.compname, obj_type, obj_name, self.timeout)
            )
            return False
        else:
            rospy.loginfo(
                "%s: %s %s exists" % (self.compname, obj_type, obj_name)
            )
            return True

    def connect_pins_and_signals(self):
        # check for parameters
        # for each drive, connect pins to pins
        # for each drive, connect existing signals to pins
        rospy.loginfo("%s: trying to connect pins" % self.compname)
        if not self.sim:
            for key, drive in self.drives.items():
                drive.connect_pins_and_signals()
        # check for and connect additional signals
        self.additional_signals = rospy.get_param(
            '/hal_402_device_mgr/signals', []
        )

        # hal can be busy setting up, so allow up to self.timeout for
        # all pins and signals to be created, then bomb out
        for pin, signal in self.additional_signals:
            if not self.check_hal(pin=pin) or not self.check_hal(signal=signal):
                raise RuntimeError("Error:  HAL object failed to appear")

        for pin, signal in self.additional_signals:
            # Connect signals to pins
            rospy.loginfo(
                "%s: connecting signal %s to pin %s", self.compname, signal, pin
            )
            mk_hal.Signal(signal).link(mk_hal.Pin(pin))

    def create_drives(self):
        # check if parameters exist:
        if self.has_parameters(
            [
                '/hal_402_device_mgr/drives/name',
                '/hal_402_device_mgr/drives/instances',
                '/hal_402_device_mgr/slaves/instances',
            ]
        ):
            name = rospy.get_param('/hal_402_device_mgr/drives/name')
            drive_instances = rospy.get_param(
                '/hal_402_device_mgr/drives/instances'
            )
            slave_instances = rospy.get_param(
                '/hal_402_device_mgr/slaves/instances'
            )
            # sanity check
            if len(slave_instances) != len(drive_instances):
                rospy.logerr(
                    "%s: nr of drive and slave instances do not match"
                    % self.compname
                )
            else:
                for i in range(0, len(drive_instances)):
                    # create n drives from ROS parameters
                    drivename = name + "_%s" % drive_instances[i]
                    slave_inst = slave_instances[i]
                    self.drives[drivename] = Drive402(
                        drivename, self, slave_inst, self.sim
                    )
                    rospy.loginfo("%s: %s created" % (self.compname, drivename))
        else:
            rospy.logerr(
                "%s: no correct /hal_402_device_mgr/drives params"
                % self.compname
            )

    def create_pins(self):
        for key, pin in self.pins.items():
            pin.set_parent_comp(self.halcomp)
            pin.create_halpin()

    def create_publisher(self):
        # todo, read from ROS param server
        has_update_rate = rospy.has_param('/hal_402_device_mgr/update_rate')
        if has_update_rate:
            self.update_rate = rospy.get_param(
                '/hal_402_device_mgr/update_rate'
            )
            self.rate = rospy.Rate(self.update_rate)
            # create publishers for topics and send out a test message
            for key, drive in self.drives.items():
                drive.create_topics()
                if drive.sim is True:
                    drive.test_publisher()
        else:
            rospy.logerr(
                "%s: no /hal_402_device_mgr/update_rate param found"
                % self.compname
            )

    def all_equal_status(self, status):
        # check if all the drives have the same status
        for key, drive in self.drives.items():
            if not (drive.curr_state == status):
                return False
        return True

    def sim_set_drives_status(self, status):
        for key, drive in self.drives.items():
            drive.sim_set_status(status)

    def cb_robot_state_service(self, req):
        # The service callback
        # the value of the requested state is in req.req_state (string)
        # the return value is the service response (string)
        # check the requested state for validity
        if req.req_state not in self.states:
            rospy.loginfo(
                "%s: request for state failed, %s not a valid state"
                % (self.compname, req.req_state)
            )
            return (
                "request for state %s failed, state not known" % req.req_state
            )
        else:
            if req.req_state != self.curr_robot_state:
                self.process_service(req.req_state)
            return self.curr_robot_state

    def process_service(self, requested_state):
        # pick a transition table for the requested state
        tr_table = self.states[requested_state][0]
        all_target_states = self.states[requested_state][1]
        # set "active" transition table for the drive
        for key, drive in self.drives.items():
            drive.set_transition_table(tr_table)

        # the drive returns success if all the drives are :
        # OPERATION ENABLED or SWITCH ON DISABLED or max_attempts
        i = 0
        max_retries_unreacheable_state = 5000
        max_attempts = len(StateMachine402.states_402)
        while (not self.all_equal_status(all_target_states)) or (
            i < (max_attempts + 1)
        ):
            for key, drive in self.drives.items():
                # traverse states for all drives (parallel)
                # ignore drives which state already is at the target state
                if not (drive.curr_state == all_target_states):
                    retries = max_retries_unreacheable_state
                    # set a timeout for waiting on a drive to have a state
                    # from which _we_ can find a valid transition
                    while (not drive.next_transition()) and (retries > 0):
                        # transition is not possible !
                        # the drive is in a state we cannot solve and need
                        # to wait for the drive to get to a valid state
                        #
                        time.sleep(0.05)
                        self.update_drive_states()
                        retries -= 1
                    if retries < max_retries_unreacheable_state:
                        rospy.loginfo(
                            "%s: %s needed %i retries from state %s"
                            % (
                                self.compname,
                                drive.drive_name,
                                (max_retries_unreacheable_state - retries),
                                drive.curr_state,
                            )
                        )

            # give HAL time to make at least 1 cycle
            time.sleep(0.002)
            self.update_drive_states()
            self.publish_states()
            i += 1

        # when states are successfully reached, update overall state
        # when all statuses have been reached, success
        # otherwise num_states has overflowed
        if self.all_equal_status(all_target_states):
            self.curr_robot_state = requested_state
        else:
            self.curr_robot_state = 'error'
        # make sure we mirror the state in the halpin
        # convert state to number
        for key, val in self.conv_value_to_state.items():
            if val == self.curr_robot_state:
                state_nr = int(key)
                self.pins['state-fb'].set_local_value(state_nr)
                self.pins['state-fb'].set_hal_value()

    def cb_test_service_cb(self, req):
        rospy.loginfo("%s: cb_test_service" % self.compname)
        # the response that's returned is the return value of the callback
        return "test service return string"

    def create_service(self):
        # $ rosservice call /hal_402_drives_mgr abcd
        # will call the function callback, that will receive the
        # service message as an argument
        # testin routine:
        # self.service = rospy.Service('hal_402_drives_mgr',
        #                             srv_robot_state,
        #                             self.cb_test_service)
        self.service = rospy.Service(
            'hal_402_drives_mgr', srv_robot_state, self.cb_robot_state_service
        )
        rospy.loginfo(
            "%s: service %s created"
            % (self.compname, self.service.resolved_name)
        )

    def update_drive_states(self):
        for key, drive in self.drives.items():
            drive.update_state()

        # get the status pins, and save their value locally
        self.prev_hal_state_cmd = self.curr_hal_state_cmd
        for key, pin in self.pins.items():
            if pin.dir == hal.HAL_IN:
                pin.sync_hal()
        self.curr_hal_state_cmd = self.pins['state-cmd'].local_pin_value

    def check_for_state_cmd_change(self):
        if self.state_cmd_changed() and not self.state_cmd_pin_eq_service():
            # state-cmd pin needs to be changed, and the requested state needs
            # to be unequal to the current state.
            #
            # convert value to string
            requested_state = self.conv_value_to_state[
                str(self.curr_hal_state_cmd)
            ]
            self.process_service(requested_state)

    def state_cmd_changed(self):
        if not (self.prev_hal_state_cmd == self.curr_hal_state_cmd):
            return True
        else:
            return False

    def state_cmd_pin_eq_service(self):
        if self.curr_hal_state_cmd == self.curr_robot_state:
            return True
        else:
            return False

    def publish_states(self):
        for key, drive in self.drives.items():
            drive.publish_state()

    def publish_errors(self):
        for key, drive in self.drives.items():
            drive.publish_error()

    def run(self):
        while not rospy.is_shutdown():
            self.update_drive_states()
            self.check_for_state_cmd_change()
            self.publish_errors()
            self.publish_states()
            self.rate.sleep()
