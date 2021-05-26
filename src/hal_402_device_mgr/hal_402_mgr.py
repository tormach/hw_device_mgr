import rospy
import hal
import traceback
import time
from fysom import FysomGlobalMixin, FysomGlobal, Canceled

from .pins import HALPins

from hal_402_device_mgr.hal_402_drive import Drive402


class Hal402Timeout(RuntimeError):
    pass


class Hal402Mgr(FysomGlobalMixin):
    compname = 'hal_402_mgr'
    default_control_mode = 'MODE_CSP'
    ros_param_base = 'hal_402_device_mgr'
    goal_state_timeout = 5.0  # seconds

    pin_specs = {
        # IO pin for command request and feedback
        'state-cmd': dict(ptype=hal.HAL_U32, pdir=hal.HAL_IO),
        # IN pin set high when external quick stop triggered
        'quick_stop': dict(ptype=hal.HAL_BIT, pdir=hal.HAL_IN),
        # OUT pin set high for two update cycles around drive enable
        # transition
        'reset': dict(ptype=hal.HAL_BIT, pdir=hal.HAL_OUT),
    }

    ####################################################
    # Initialization

    def __init__(self):
        self.state = 'stop_command'
        self.command = 'stop'
        super().__init__()

    # To be called by external code
    def init(self):
        # Init ROS
        self.init_ros()
        # Init HAL component and common pins
        self.hal_comp_init()
        # Init drive objects and drive pins
        self.create_drives()
        # Mark HAL comp ready
        self.hal_comp_ready()

    def init_ros(self):
        # Init ROS node, shutdown callback, rate object
        rospy.init_node(self.compname)
        rospy.on_shutdown(self.call_cleanup)
        self.update_rate = rospy.get_param(
            f'{self.ros_param_base}/update_rate', 10
        )
        self.rate = rospy.Rate(self.update_rate)
        rospy.loginfo(f"Initialized '{self.compname}' ROS node")

    def hal_comp_init(self):
        # Init HAL userland component
        self.comp = hal.component(self.compname)
        rospy.loginfo(f"Initialized '{self.compname}' HAL component")
        self.pins = HALPins(self.comp, self.pin_specs)
        self.pins.init_pins()

    def hal_comp_ready(self):
        self.comp.ready()
        rospy.loginfo("%s: HAL component ready" % self.compname)

    def create_drives(self):
        self.drives = []
        drive_config = rospy.get_param(f'{self.ros_param_base}/drives', None)
        if drive_config is None:
            rospy.logerr(f"No drive config in {self.ros_param_base}/drives")
            return

        self.sim = rospy.get_param("/sim_mode", True)

        num_drives = len(drive_config)
        mode = "sim" if self.sim else "real-hardware"
        rospy.loginfo(f"Configuring {num_drives} {mode}-mode drives")

        drives = {}
        for drive_name in drive_config.keys():
            # Create drives from ROS parameters; read drive attributes
            # one at a time to create meaningful errors without extra
            # code
            base_param = f"{self.ros_param_base}/drives/{drive_name}"
            drive_obj = Drive402(
                drive_name=drive_name,
                drive_type=rospy.get_param(f"{base_param}/type"),
                slave_number=rospy.get_param(f"{base_param}/slave_number"),
                comp=self.comp,
                sim=self.sim,
            )
            drive_obj.init()
            drives[drive_obj.slave_number] = drive_obj
            rospy.loginfo(f"Initialized drive {drive_obj.drive_name}")
        self.drives = [drives[i] for i in sorted(drives.keys())]

    ####################################################
    # Drive state FSM

    GSM = FysomGlobal(
        initial=dict(state='stop', event='stop_command', defer=True),
        events=[
            # Fault state:  From any state
            # - fault:  done
            dict(name='fault_command', src='*', dst='fault_1'),
            dict(name='fault_complete', src='fault_1', dst='fault_complete'),
            # Start state:  From any state but 'start*'
            # - start_1:  Switch all drives to SWITCHED ON
            # - start_2:  Set all drive control modes to default
            # - start_3:  Switch all drives to OPERATION ENABLED
            # - start_complete:  Done
            dict(name='start_command', src='*', dst='start_1'),
            dict(name='start_2', src='start_1', dst='start_2'),
            dict(name='start_3', src='start_2', dst='start_3'),
            dict(name='start_complete', src='start_3', dst='start_complete'),
            # Stop state:  From any state but 'stop*'
            # - stop_1:  Put all drives in SWITCH ON DISABLED and CSP mode
            # - stop_complete:  Done
            dict(name='stop_command', src='*', dst='stop_1'),
            dict(name='stop_complete', src='stop_1', dst='stop_complete'),
            # Home state:  From only 'stop_complete'
            # - home_1:  Set all drive control modes to HM
            # - home_2:  Switch all drives to OPERATION ENABLED
            # - home_3:  Set home flag
            # - home_complete:  Done; issue 'stop' command
            dict(name='home_command', src='*', dst='home_1'),
            dict(name='home_2', src='home_1', dst='home_2'),
            dict(name='home_3', src='home_2', dst='home_3'),
            dict(name='home_complete', src='home_3', dst='home_complete'),
        ],
        state_field='state',
    )

    #
    # Fault command
    #
    def on_before_fault_command(self, e):
        if self.fsm_check_command(e):
            rospy.logerr('Entering fault state')
            return True
        else:
            return False

    def on_enter_fault_1(self, e):
        return self.fsm_set_drive_goal_state(e, 'FAULT')

    def on_before_fault_complete(self, e):
        return self.fsm_check_drive_goal_state(e, 'FAULT')

    def on_enter_fault_complete(self, e):
        self.fsm_finalize_command(e)

    #
    # Start command
    #
    def on_before_start_command(self, e):
        return self.fsm_check_command(e)

    def on_enter_start_1(self, e):
        self.fsm_set_drive_goal_state(e, 'SWITCHED ON')

    def on_before_start_2(self, e):
        if not self.all_drives_status_flags(VOLTAGE_ENABLED=True):
            self.timer_check_overrun("No voltage at drive motor power inputs")
            return False
        return self.fsm_check_drive_goal_state(e, 'SWITCHED ON')

    def on_enter_start_2(self, e):
        mode = self.default_control_mode
        self.fsm_set_drive_control_mode(e, mode)

    def on_before_start_3(self, e):
        mode = self.default_control_mode
        return self.fsm_check_drive_control_mode(e, mode)

    def on_enter_start_3(self, e):
        # Set reset pin during transition to OPERATION ENABLED
        e.reset_pin = True
        self.fsm_set_drive_goal_state(e, 'OPERATION ENABLED')

    def on_before_start_complete(self, e):
        return self.fsm_check_drive_goal_state(e, 'OPERATION ENABLED')

    def on_enter_start_complete(self, e):
        # Clear reset pin in OPERATION ENABLED
        e.reset_pin = False
        self.fsm_finalize_command(e)

    #
    # Stop command
    #
    def on_before_stop_command(self, e):
        return self.fsm_check_command(e)

    def on_enter_stop_1(self, e):
        # Zero out command & feedback differences to give operator
        # confidence turning on machine
        e.reset_pin = True
        return self.fsm_set_drive_goal_state(e, 'SWITCH ON DISABLED')

    def on_before_stop_complete(self, e):
        return self.fsm_check_drive_goal_state(e, 'SWITCH ON DISABLED')

    def on_enter_stop_complete(self, e):
        e.reset_pin = False
        self.fsm_finalize_command(e)

    #
    # Home command
    #
    def on_before_home_command(self, e):
        if not e.src == 'stop_complete':
            rospy.logwarn("Unable to home when drives not stopped")
            return False
        return self.fsm_check_command(e)

    def on_enter_home_1(self, e):
        self.fsm_set_drive_control_mode(e, 'MODE_HM')

    def on_before_home_2(self, e):
        return self.fsm_check_drive_control_mode(e, 'MODE_HM')

    def on_enter_home_2(self, e):
        self.fsm_set_drive_goal_state(e, 'OPERATION ENABLED')

    def on_before_home_3(self, e):
        if not self.all_drives_status_flags(VOLTAGE_ENABLED=True):
            self.timer_check_overrun("No voltage at drive motor power inputs")
            return False
        return self.fsm_check_drive_goal_state(e, 'OPERATION ENABLED')

    def on_enter_home_3(self, e):
        # MODE_HM:  OPERATION_MODE_SPECIFIC_1 = HOMING_START
        self.set_drive_control_flags(OPERATION_MODE_SPECIFIC_1=True)

    def on_before_home_complete(self, e):
        if self.all_drives_status_flags(HOMING_COMPLETED=True):
            return True
        self.timer_check_overrun("waiting on drives HOMING_COMPLETED flags")
        # Cancel event
        rospy.loginfo(
            "Homing sequence waiting on drives HOMING_COMPLETED flags"
        )
        return False

    def on_enter_home_complete(self, e):
        self.fsm_finalize_command(e)
        # Automatically return to SWITCH ON DISABLED after homing
        self.command = 'stop'

    #
    # All states
    #
    def on_change_state(self, e):
        # This runs after every `on_enter_*`; attrs of e set there (or
        # `on_before_*`) will be present here

        # Set/clear reset pin
        self.pins.reset.set(getattr(e, 'reset_pin', False))
        if self.pins.reset.changed:
            rospy.loginfo(f'Reset pin set to {self.pins.reset.get()}')
            self.pins.reset.write()  # Set now to effect reset early

    #
    # Helpers
    #
    cmd_name_to_int_map = {
        'stop': 0,
        'start': 1,
        'home': 2,
        'fault': 3,
    }

    cmd_int_to_name_map = {v: k for k, v in cmd_name_to_int_map.items()}

    def timer_start(self):
        self._timeout = time.time() + self.goal_state_timeout

    def timer_check_overrun(self, msg):
        if not hasattr(self, '_timeout') or time.time() <= self._timeout:
            return

        msg = f'{self.command} timeout:  {msg}'
        self.command = 'fault'
        del self._timeout
        raise Hal402Timeout(msg)

    @classmethod
    def fsm_command_from_event(cls, e):
        return e.dst.split('_')[0]

    def fsm_check_command(self, e):
        cmd_name = self.fsm_command_from_event(e)
        if e.src != f'{cmd_name}_command' and e.src.startswith(cmd_name):
            # Already running
            rospy.logwarn(f"Ignoring {cmd_name} command from state {e.src}")
            return False
        else:
            rospy.loginfo(f"Received {cmd_name} command:  {e.msg}")
            self.command = cmd_name
            self.timer_start()
            return True

    def fsm_check_drive_goal_state(self, e, state):
        if self.all_drives_goal_state_reached(state):
            return True
        self.timer_check_overrun(f"waiting on drives to reach {state}")
        # Cancel event
        return False

    def fsm_set_drive_goal_state(self, e, state):
        cmd_name = self.fsm_command_from_event(e)
        rospy.loginfo(f"{cmd_name} command:  setting drive goal state {state}")
        self.set_drive_goal_state(state)

    def fsm_check_drive_control_mode(self, e, mode):
        if self.all_drives_mode(mode):
            return True
        self.timer_check_overrun(f"waiting on drives to enter mode {mode}")
        # Cancel event
        return False

    def fsm_set_drive_control_mode(self, e, mode):
        cmd_name = self.fsm_command_from_event(e)
        rospy.loginfo(f"{cmd_name} command:  Setting drive mode {mode}")
        self.set_drive_control_mode(mode)

    def fsm_finalize_command(self, e):
        cmd_name = self.fsm_command_from_event(e)
        self.pins.state_cmd.set(self.cmd_name_to_int_map[cmd_name])
        rospy.loginfo(f"Command {cmd_name} completed")

    ####################################################
    # Execution

    def run(self):
        try:
            while not rospy.is_shutdown():
                try:
                    self.update()
                except rospy.exceptions.ROSInterruptException:
                    raise  # Catch again outside the loop
                except Hal402Timeout as e:
                    rospy.logerr(e)
                except Exception:
                    # Ignore other exceptions & enter fault mode in
                    # hopes we can recover
                    rospy.logerr('Ignoring unexpected exception; details:')
                    for line in traceback.format_exc().splitlines():
                        rospy.logerr(line)
                    self.command = 'fault'
                self.rate.sleep()
        except rospy.exceptions.ROSInterruptException as e:
            rospy.loginfo(f"ROSInterruptException: {e}")

    def call_cleanup(self):
        # need to unload the userland component here?
        rospy.loginfo("Stopping ...")
        rospy.loginfo("Stopped")

    fsm_next_state_map = dict(
        # Map current command to dict of {current_state:next_event}
        # names; `None` means arrived
        start=dict(
            start_1='start_2',
            start_2='start_3',
            start_3='start_complete',
            start_complete=None,
        ),
        stop=dict(
            stop_1='stop_complete',
            stop_complete=None,
        ),
        home=dict(
            home_1='home_2',
            home_2='home_3',
            home_3='home_complete',
            home_complete=None,
        ),
        fault=dict(
            fault_1='fault_complete',
            fault_complete=None,
        ),
    )

    def update(self):
        # Read all input pins and update state machine
        self.pins.read_all()
        self.read_drives_state()

        # Check for incoming command on state-cmd pin
        if self.pins.state_cmd.changed:
            # Other commands from state-cmd pin; can't override fault
            cmd = self.cmd_int_to_name_map[self.pins.state_cmd.get()]
            msg = 'state-cmd pin changed'
        else:
            cmd = None

        # Special cases that override with 'fault' command:
        if not self.all_drives_operational():
            # Some drives not online operational
            cmd = 'fault'
            fds = self.drives_operational(negate=True)
            fd_names = ', '.join(d.drive_name for d in fds)
            msg = f'Drives ({fd_names}) not online and operational'
        elif self.any_drives_in_state('FAULT') and cmd not in ('stop', 'start'):
            # Some drives in FAULT but no recovery command
            cmd = 'fault'
            fds = self.drives_in_state('FAULT')
            fd_names = ', '.join(d.drive_name for d in fds)
            msg = f'Drives ({fd_names}) in FAULT state'
        elif self.pins.quick_stop.get() and self.command != 'fault':
            # Quick stop pin high; treat this as a fault command
            cmd = 'fault'
            msg = 'quick_stop pin high'

        if cmd is not None and cmd != self.command:
            # Received new command to stop/start/home/fault.  Try it
            # by triggering the FSM event; a Canceled exception means
            # it can't be done, so ignore it.
            event = f'{cmd}_command'
            try:
                self.trigger(event, msg=msg)
            except Canceled:
                rospy.loginfo(f'Unable to honor {event} command')

        # Attempt automatic transition to next state; if not possible,
        # `on_before_{event}()` method will cause Canceled exception
        event = self.automatic_next_event()
        if event is not None:
            try:
                self.trigger(event, msg='Automatic transition')
            except Canceled:
                rospy.logdebug(f'Cannot transition to next state {event}')

        # Write all output pins and let drives do their thing
        self.pins.write_all()
        self.write_drives_state()

    def automatic_next_event(self):
        state_map = self.fsm_next_state_map[self.command]
        event = state_map.get(self.state, f'{self.command}_command')
        return event

    ####################################################
    # Drive helpers

    def read_drives_state(self):
        for drive in self.drives:
            drive.read_state()

    def write_drives_state(self):
        for drive in self.drives:
            drive.write_state()

    def set_drive_goal_state(self, goal_state):
        for drive in self.drives:
            drive.set_goal_state(goal_state)
            drive.set_control_flags()

    def set_drive_control_mode(self, mode):
        mode = Drive402.normalize_control_mode(mode)
        for drive in self.drives:
            drive.set_control_mode(mode)

    def set_drive_control_flags(self, **flags):
        for drive in self.drives:
            drive.set_control_flags(**flags)

    def all_drives_mode(self, mode):
        mode = Drive402.normalize_control_mode(mode)
        for drive in self.drives:
            if drive.get_control_mode() != mode:
                return False
        return True

    def all_drives_status_flags(self, **flags):
        for drive in self.drives:
            for flag, val in flags.items():
                if drive.get_status_flag(flag) != val:
                    return False
        return True

    def drives_operational(self, negate=False):
        # Return list of operational drives (online & slave-oper)
        status = not negate
        return [d for d in self.drives if d.operational is status]

    def all_drives_operational(self):
        # Check if all drives are operational (no drives not operational!)
        return len(self.drives_operational(negate=True)) == 0

    def drives_in_state(self, state, negate=False):
        # Return list of drives with matching state
        if negate:
            return [d for d in self.drives if d.state != state]
        else:
            return [d for d in self.drives if d.state == state]

    def any_drives_in_state(self, state):
        # check if any drives have the matching state
        return len(self.drives_in_state(state)) > 0

    def all_drives_in_state(self, state):
        # check if all drives have the matching state
        return len(self.drives_in_state(state, negate=True)) == 0

    def all_drives_goal_state_reached(self, state):
        for drive in self.drives:
            if not drive.operational:
                return False
            if not drive.get_goal_state() == state:
                return False
            if not drive.is_goal_state_reached():
                return False
        return True
