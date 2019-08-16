# -*- coding: utf-8 -*-
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
class transition_obj(object):
    name = attr.ib()
    target_path = attr.ib()
    target_name = attr.ib()
    transition = attr.ib()


class Hal402Mgr(object):
    def __init__(self):
        # Configure sim mode if SIM environment variable is set
        self.sim = rospy.get_param("/sim_mode", True)
        self.compname = 'hal_402_mgr'
        self.drives = dict()
        self.fsm = Fysom(
            {
                'initial': {'state': 'initial', 'event': 'init', 'defer': True},
                'events': [
                    {
                        'name': 'stop',
                        'src': ['initial', 'started', 'fault'],
                        'dst': 'stopped',
                    },
                    {'name': 'start', 'src': 'stopped', 'dst': 'started'},
                    {
                        'name': 'error',
                        'src': ['initial', 'started', 'stopped'],
                        'dst': 'fault',
                    },
                ],
                'callbacks': {
                    'oninitial': self.fsm_in_initial,
                    'onstopped': self.fsm_in_stopped,
                    'onstarted': self.fsm_in_started,
                    'onfault': self.fsm_in_fault,
                    'onstop': self.fsm_on_stop,
                    'onstart': self.fsm_on_start,
                    'onerror': self.fsm_on_error,
                },
            }
        )
        self.prev_hal_transition_cmd = -2
        self.curr_hal_transition_cmd = -2

        self.transitions = {
            'start': transition_obj(
                name='start',
                target_path=StateMachine402.path_to_operation_enabled,
                target_name='OPERATION ENABLED',
                transition=self.fsm.start,
            ),
            'stop': transition_obj(
                name='stop',
                target_path=StateMachine402.path_to_switch_on_disabled,
                target_name='SWITCH ON DISABLED',
                transition=self.fsm.stop,
            ),
            'error': transition_obj(
                name='error',
                target_path=StateMachine402.path_to_switch_on_disabled,
                target_name='SWITCH ON DISABLED',
                transition=self.fsm.error,
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
        }
        self.conv_value_to_transition = {2: 'error', 0: 'stop', 1: 'start'}
        self.conv_value_to_state = {
            3: 'initial',
            2: 'fault',
            0: 'stopped',
            1: 'started',
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

        # check if we're running real hardware, and set up sim state if
        # applicable
        self.check_for_real_hardware_setup()

        self.create_service()
        self.create_publisher()

        # done
        self.halcomp.ready()
        self.fsm.init()

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
                        drivename, self, slave_inst
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

    def all_drives_have_status(self, status):
        # check if all the drives have the same status
        for key, drive in self.drives.items():
            if not (drive.curr_state == status):
                return False
        return True

    def one_drive_has_status(self, status):
        # check if all the drives have the same status
        for key, drive in self.drives.items():
            if drive.curr_state == status:
                return True
        return False

    def sim_set_drives_status(self, status):
        for key, drive in self.drives.items():
            drive.sim_set_status(status)

    def cb_robot_state_service(self, req):
        # The service callback
        # the requested transition is in req.req_transition (string)
        # the return value for the service response (string) is a message
        # check the requested state for validity
        if req.req_transition not in self.transitions:
            rospy.loginfo(
                "%s: request failed, %s not a valid transition"
                % (self.compname, req.req_transition)
            )
            return "%s: request failed, %s not a valid transition" % (
                self.compname,
                req.req_transition,
            )
        else:
            return self.execute_transition(req.req_transition)

    def execute_transition(self, transition):
        try:
            f = self.transitions[transition].transition
            f(msg=transition)
            rospy.loginfo(
                "%s: Transition properly executed, current state \'%s\'"
                % (self.compname, self.fsm.current)
            )
            return (
                "Transition properly executed, current state \'%s\'"
                % self.fsm.current
            )
        except FysomError:
            rospy.logwarn(
                "%s: Transition not possible from state \'%s\', with value %s"
                % (self.compname, self.fsm.current, transition)
            )
            return (
                "Transition not possible from state \'%s\'" % self.fsm.current
            )

    # transition callbacks
    def fsm_on_stop(self, e=None):
        self.process_drive_transitions(e.msg)

    def fsm_on_start(self, e=None):
        self.process_drive_transitions(e.msg)

    def fsm_on_error(self, e=None):
        self.process_drive_transitions(e.msg)

    # enter state callbacks
    def fsm_in_initial(self, e=None):
        # print('in_initial')
        self.update_hal_state_fb()

    def fsm_in_stopped(self, e=None):
        # print('in_stopped')
        self.update_hal_state_fb()

    def fsm_in_started(self, e=None):
        # print('in_started')
        self.update_hal_state_fb()

    def fsm_in_fault(self, e=None):
        rospy.logerr("%s: The machine entered \'fault\' state" % self.compname)
        self.update_hal_state_fb()

    # make sure we mirror the state in the halpin
    # convert state to number
    def update_hal_state_fb(self):
        for key, val in self.conv_value_to_state.items():
            if val == self.fsm.current:
                state_nr = key
                self.pins['state-fb'].set_local_value(state_nr)
                self.pins['state-fb'].set_hal_value()

    def process_drive_transitions(self, transition):
        # pick a transition table for the requested state
        tr_table = self.transitions[transition].target_path
        target_states = self.transitions[transition].target_name
        # set "active" transition table for the drive
        for key, drive in self.drives.items():
            drive.set_transition_table(tr_table)

        # the drive returns success if all the drives are :
        # OPERATION ENABLED or SWITCH ON DISABLED or max_attempts
        i = 0
        max_retries_unreacheable_state = 5
        max_attempts = len(StateMachine402.states_402)
        while (not self.all_drives_have_status(target_states)) or (
            i < (max_attempts + 1)
        ):
            for key, drive in self.drives.items():
                # traverse states for all drives (parallel)
                # ignore drives which state already is at the target state
                if not (drive.curr_state == target_states):
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

        # when not all statuses have been reached, we generate an error event
        if not self.all_drives_have_status(target_states):
            self.fsm.error()

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
        self.prev_hal_transition_cmd = self.curr_hal_transition_cmd
        for key, pin in self.pins.items():
            if pin.dir == hal.HAL_IN:
                pin.sync_hal()
        self.curr_hal_transition_cmd = self.pins['state-cmd'].local_pin_value

    def hal_transition_cmd(self):
        if self.transition_cmd_changed():
            if (
                self.curr_hal_transition_cmd
                not in self.conv_value_to_transition
            ):
                rospy.loginfo(
                    "%s: HAL request failed, %s not a valid transition"
                    % (self.compname, self.curr_hal_transition_cmd)
                )
                return "%s: HAL request failed, %s not a valid transition" % (
                    self.compname,
                    self.curr_hal_transition_cmd,
                )
            else:
                transition = self.conv_value_to_transition[
                    self.curr_hal_transition_cmd
                ]
                self.execute_transition(transition)

    def transition_cmd_changed(self):
        if not (self.prev_hal_transition_cmd == self.curr_hal_transition_cmd):
            return True
        else:
            return False

    def check_for_errors(self):
        # when in a certain state, we need to check things so we can initiate
        # a transition to an error state for example
        one_drive_faulted = self.one_drive_has_status('FAULT')
        if self.fsm.current != 'fault':
            if one_drive_faulted:
                self.execute_transition('error')

    def publish_states(self):
        for key, drive in self.drives.items():
            drive.publish_state()

    def publish_errors(self):
        for key, drive in self.drives.items():
            drive.publish_error()

    def run(self):
        while not rospy.is_shutdown():
            self.update_drive_states()
            self.check_for_errors()
            self.hal_transition_cmd()
            self.rate.sleep()
