import rospy
import time
import hal
import attr
from fysom import Fysom, FysomError

# import service messages from the ROS node
from hal_402_device_mgr.srv import srv_robot_state
from hal_402_device_mgr.hal_402_drive import (
    Drive402,
    GenericHalPin,
    StateMachine402,
)


@attr.s
class TransitionItem:
    name = attr.ib()
    value = attr.ib()
    transition_cb = attr.ib()


class Hal402Mgr:
    compname = 'hal_402_mgr'

    def __init__(self):
        # create ROS node
        rospy.init_node(self.compname)
        rospy.loginfo("%s: Node started" % self.compname)

        # create HAL userland component
        self.halcomp = hal.component(self.compname)
        rospy.loginfo("%s: HAL component created" % self.compname)

        # Configure sim mode if SIM environment variable is set
        self.sim = rospy.get_param("/sim_mode", True)
        self.drives = []
        self.fsm = Fysom(
            {
                'initial': {'state': 'initial', 'event': 'init', 'defer': True},
                'events': [
                    {
                        'name': 'stop',
                        'src': ['initial', 'fault', 'enabled'],
                        'dst': 'stopping',
                    },
                    {'name': 'start', 'src': 'disabled', 'dst': 'starting'},
                    {'name': 'enable', 'src': 'starting', 'dst': 'enabled'},
                    {'name': 'disable', 'src': 'stopping', 'dst': 'disabled'},
                    {
                        'name': 'error',
                        'src': [
                            'initial',
                            'starting',
                            'enabled',
                            'stopping',
                            'disabled',
                        ],
                        'dst': 'fault',
                    },
                ],
                'callbacks': {
                    'oninitial': self.fsm_in_initial,
                    'onstopping': self.fsm_in_stopping,
                    'onstarting': self.fsm_in_starting,
                    'ondisabled': self.fsm_in_disabled,
                    'onenabled': self.fsm_in_enabled,
                    'onfault': self.fsm_in_fault,
                },
            }
        )

        self.prev_hal_transition_cmd = -2
        self.curr_hal_transition_cmd = -2
        self.prev_hal_reset_pin = 0
        self.curr_hal_reset_pin = 0

        self.transitions = {
            'stop': TransitionItem(
                name='stop', value=0, transition_cb=self.fsm.stop
            ),
            'start': TransitionItem(
                name='start', value=1, transition_cb=self.fsm.start
            ),
            'error': TransitionItem(
                name='error', value=2, transition_cb=self.fsm.error
            ),
            'enable': TransitionItem(
                name='enable', value=3, transition_cb=self.fsm.enable
            ),
            'disable': TransitionItem(
                name='disable', value=4, transition_cb=self.fsm.disable
            ),
        }

        self.pins = {
            # Pins used by this component to call the service callbacks
            'state-cmd': GenericHalPin(
                '%s.state-cmd' % self.compname, hal.HAL_IN, hal.HAL_U32
            ),
            'state-fb': GenericHalPin(
                '%s.state-fb' % self.compname, hal.HAL_OUT, hal.HAL_S32
            ),
            'reset': GenericHalPin(
                '%s.reset' % self.compname, hal.HAL_IN, hal.HAL_BIT
            ),
        }
        self.conv_value_to_state = {
            3: 'initial',
            2: 'fault',
            0: 'disabled',
            1: 'enabled',
            4: 'stopping',
            5: 'starting',
        }

        # read in the error list from parameters
        self.devices_error_list = self.read_device_error_list()
        # create drives which create pins
        self.create_drives()
        # create pins for calling service callback
        self.create_pins()

        # check if we're running real hardware, and set up sim state if
        # applicable
        self.check_for_real_hardware_setup()

        self.get_update_rate()
        self.create_service()
        self.create_publisher()

        # done
        self.halcomp.ready()
        self.fsm.init()

        rospy.loginfo("%s: HAL component ready" % self.compname)

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

    def read_device_error_list(self):
        if self.has_parameters(['/device_fault_code_list']):
            device_err_list = rospy.get_param('/device_fault_code_list')
            return device_err_list
        else:
            rospy.logerr(
                "%s: no /device_fault_code_list params" % self.compname
            )
            return {}

    def create_drives(self):
        if self.has_parameters(
            [
                '/hal_402_device_mgr/drives/name',
                '/hal_402_device_mgr/drives/instances',
                '/hal_402_device_mgr/drives/types',
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
            drive_types = rospy.get_param('/hal_402_device_mgr/drives/types')
            # sanity check
            if (len(slave_instances) != len(drive_instances)) or (
                len(drive_types) != len(drive_instances)
            ):
                rospy.logerr(
                    "number of drive and slave instances or drive types do not match"
                )
            else:
                for i in range(0, len(drive_instances)):
                    # create n drives from ROS parameters
                    drive_name = name + "_%s" % drive_instances[i]
                    slave_inst = slave_instances[i]
                    drive_type = drive_types[i]
                    self.drives.append(
                        Drive402(
                            drive_name=drive_name,
                            drive_type=drive_type,
                            parent=self,
                            slave_inst=slave_inst,
                        )
                    )
                    rospy.loginfo(f"{self.compname}: {drive_name} created")
        else:
            rospy.logerr("no correct /hal_402_device_mgr/drives params")

    def create_pins(self):
        for key, pin in self.pins.items():
            pin.set_parent_comp(self.halcomp)
            pin.create_halpin()

    def get_update_rate(self):
        has_update_rate = rospy.has_param('/hal_402_device_mgr/update_rate')
        if has_update_rate:
            self.update_rate = rospy.get_param(
                '/hal_402_device_mgr/update_rate'
            )
            self.rate = rospy.Rate(self.update_rate)
        else:
            rospy.logerr(
                "%s: no /hal_402_device_mgr/update_rate param found"
                % self.compname
            )

    def get_error_info(self, devicetype, error_code_pair):
        rospy.loginfo(
            "Looking up error code {} for device type {}".format(
                error_code_pair, devicetype
            )
        )
        error_code_list = self.devices_error_list.get(devicetype, {})

        if error_code_pair and len(error_code_pair) >= 2:
            err_key_num = error_code_pair[1] & 0xFFFF
            err_key_lower = f"0x{err_key_num:04x}"
            err_key_upper = f"0x{err_key_num:04X}"
            err_info = error_code_list.get(
                err_key_lower, error_code_list.get(err_key_upper, None)
            )

            if err_info is not None:
                return err_info

        return {
            'description': Drive402.GENERIC_ERROR_DESCRIPTION,
            'solution': Drive402.GENERIC_ERROR_SOLUTION,
        }

    def create_publisher(self):
        # create publishers for topics and send out a test message
        for drive in self.drives:
            drive.create_topics()
            if drive.sim is True:
                drive.test_publisher()

    def all_drives_are_status(self, status):
        # check if all the drives have the same status
        for drive in self.drives:
            if not (drive.curr_state == status):
                return False
        return True

    def all_drives_are_not_status(self, status):
        # check if all the drives have a status other than 'status' argument
        for drive in self.drives:
            if drive.curr_state == status:
                return False
        return True

    def one_drive_has_status(self, status):
        # check if all the drives have the same status
        for drive in self.drives:
            if drive.curr_state == status:
                return True
        return False

    def sim_set_drives_status(self, status):
        for drive in self.drives:
            drive.sim_set_status(status)

    def cb_robot_state_service(self, req):
        # The service callback
        # the requested transition is in req.req_transition (string)
        # the return value for the service response (string) is a message
        # check the requested state for validity
        if req.req_transition not in self.transitions:
            msg = f"Transition request failed: {req.req_transition} is not a valid transition"
            rospy.loginfo(msg)
            return msg
        else:
            rospy.loginfo(f"Transition request {req.req_transition} is valid")
            return self.execute_transition(req.req_transition)

    def missing_transition(self):
        rospy.logwarn("Transition callback is missing")

    def execute_transition(self, transition):
        msg = f"Starting transition '{transition}' from state '{self.fsm.current}'"
        rospy.loginfo(msg)
        try:
            transition_cb = (
                self.transitions[transition].transition_cb
                or self.missing_transition
            )
            transition_cb()
            msg = f"Completed transition '{transition}' to state '{self.fsm.current}'"
            rospy.loginfo(msg)
            return msg
        except FysomError:
            msg = "Transition '{}' not possible from state '{}'".format(
                transition,
                self.fsm.current,
            )
            rospy.logerr(msg)
            return msg

    # enter state callbacks
    def fsm_in_initial(self, e=None):
        # print('in_initial')
        self.update_hal_state_fb()

    def fsm_in_disabled(self, e=None):
        self.update_hal_state_fb()

    def change_drives(self, target_path, target_name):
        self.update_hal_state_fb()
        if self.process_drive_transitions(target_path, target_name):
            return True
        else:
            return False

    def fsm_in_stopping(self, e=None):
        target_path = StateMachine402.path_to_switch_on_disabled
        target_name = 'SWITCH ON DISABLED'
        self.update_hal_state_fb()
        if self.change_drives(target_path, target_name):
            self.execute_transition('disable')
        else:

            self.execute_transition('error')

    def fsm_in_starting(self, e=None):
        target_path = StateMachine402.path_to_operation_enabled
        target_name = 'OPERATION ENABLED'
        self.update_hal_state_fb()
        if self.change_drives(target_path, target_name):
            self.execute_transition('enable')
        else:
            self.execute_transition('error')

    def fsm_in_enabled(self, e=None):
        self.update_hal_state_fb()

    def fsm_in_fault(self, e=None):
        # first try to shut down all the drives, if they are not already off
        # thru the HAL plumbing (quick-stop bit should be low at the moment
        # one of the drive faults).
        target_path = StateMachine402.path_on_fault
        target_name = ('SWITCH ON DISABLED', 'FAULT')
        self.change_drives(target_path, target_name)
        rospy.logerr(
            f"The machine entered 'fault' state, previous state was '{e.src}'"
        )
        self.update_hal_state_fb()

    # make sure we mirror the state in the halpin
    # convert state to number
    def update_hal_state_fb(self):
        for key, val in self.conv_value_to_state.items():
            if val == self.fsm.current:
                state_nr = key
                self.pins['state-fb'].set_local_value(state_nr)
                self.pins['state-fb'].set_hal_value()

    def process_drive_transitions(self, transition_table, target_states):
        # Allow singular target state or multiple
        if isinstance(target_states, str):
            target_states = (target_states,)
        no_error = True
        timeout = 5.0  # seconds to attempt to transition a single drive
        for drive in self.drives:
            # pick a transition table for the requested state
            drive.set_transition_table(transition_table)
            transition_attempts = 0
            t0 = time.time()
            while (time.time() - t0) < timeout:
                transition_attempts += 1
                drive.update_state()
                rospy.logdebug(
                    "%s: %s, try %i: in state %s, 0x%03x"
                    % (
                        self.compname,
                        drive.drive_name,
                        transition_attempts,
                        drive.curr_state,
                        drive.curr_status_word,
                    )
                )
                if drive.curr_state in target_states:
                    break

                if not drive.is_transitionable():
                    # if a drive needs to transition itself
                    time.sleep(0.25)
                else:
                    time.sleep(0.001)
                    if not drive.next_transition():
                        # If we can't transition then the drive state machine is stuck, so bail out
                        break

            if drive.curr_state not in target_states:
                rospy.logerr(
                    f"{drive.drive_name} did not reach target state after {time.time()-t0:0.2f} seconds (current state is {drive.curr_state})"
                )
                no_error = False
                # But continue trying other drives

        self.update_drive_states()
        self.publish_states()
        return no_error

    def create_service(self):
        # $ rosservice call rosservice call /hal_402_drives_mgr "req_transition: ''"
        # will call the function callback, that will receive the service message
        # as an argument. The transition should be added between the single quotes.

        self.service = rospy.Service(
            'hal_402_drives_mgr', srv_robot_state, self.cb_robot_state_service
        )
        rospy.loginfo(
            "%s: service %s created"
            % (self.compname, self.service.resolved_name)
        )

    def update_drive_states(self):
        for drive in self.drives:
            drive.update_state()

        # get the status pins, and save their value locally
        self.prev_hal_transition_cmd = self.curr_hal_transition_cmd
        self.prev_hal_reset_pin = self.curr_hal_reset_pin
        for key, pin in self.pins.items():
            if pin.dir == hal.HAL_IN:
                pin.sync_hal()
        self.curr_hal_transition_cmd = self.pins['state-cmd'].local_pin_value
        self.curr_hal_reset_pin = self.pins['reset'].local_pin_value

    def transition_from_hal(self):
        # get check if the HAL number is one of the transition numbers
        for key, transition in self.transitions.items():
            if transition.value == self.curr_hal_transition_cmd:
                rospy.loginfo(
                    f"Found transition {key} requested from HAL (id is {transition.value})"
                )
                transition_name = key
                break
        else:
            msg = f"HAL request failed, {self.curr_hal_transition_cmd} not a valid transition"
            rospy.loginfo(msg)
            return msg

        self.execute_transition(transition_name)

    def hal_UI_cmd(self):
        if self.transition_cmd_changed():
            self.transition_from_hal()

    def transition_cmd_changed(self):
        return self.prev_hal_transition_cmd != self.curr_hal_transition_cmd

    def reset_pin_changed(self):
        return self.prev_hal_reset_pin != self.curr_hal_reset_pin

    def detect_fault_conditions(self):
        """
        If anything goes wrong with the drives (FAULT state, or a drive isn't
        in the expected state), then transition the whole system to the "fault"
        state for safety.
        """
        one_drive_faulted = self.one_drive_has_status('FAULT')
        all_drives_operational = self.all_drives_are_status('OPERATION ENABLED')
        if self.fsm.current != 'fault':
            if one_drive_faulted:
                self.execute_transition('error')
        if self.fsm.current == 'enabled':
            if not all_drives_operational:
                self.execute_transition('error')

    def on_reset_pin_changed(self):
        # Deliberate fallthrough here to allow 1-click recovery via reset button
        if self.fsm.current == 'fault':
            rospy.loginfo(
                "Reset requested, starting transition sequence to re-enable drives"
            )
            # this will get us in the disabled state again
            self.execute_transition('stop')

        if self.fsm.current == 'disabled':
            # Error was successfully recovered, continue with the transition
            # requested by the reset click
            rospy.loginfo("Recovery successful, continuing with reset request")
            self.transition_from_hal()
        elif self.fsm.current == 'fault':
            rospy.logerr(
                "Unable to come out of Reset. Please ensure that the drives are powered on and the E-stop switch is released."
            )

    def publish_states(self):
        for drive in self.drives:
            drive.publish_state()

    def publish_errors(self):
        for drive in self.drives:
            drive.publish_error()

    def call_cleanup(self):
        # need to unload the userland component here?
        rospy.loginfo("Stopping ...")
        rospy.loginfo("Stopped")

    def run(self):
        rospy.on_shutdown(self.call_cleanup)
        try:
            while not rospy.is_shutdown():
                self.update_drive_states()
                self.detect_fault_conditions()
                self.hal_UI_cmd()
                if self.reset_pin_changed():
                    self.on_reset_pin_changed()
                self.rate.sleep()
        except rospy.ROSInterruptException as e:
            rospy.loginfo(f"ROSInterruptException: {e}")
