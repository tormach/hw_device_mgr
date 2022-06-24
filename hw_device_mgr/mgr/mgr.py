from ..device import Device, SimDevice
from ..cia_402.device import CiA402Device, CiA402SimDevice


import traceback
import time
from fysom import FysomGlobalMixin, FysomGlobal, Canceled


class HWDeviceTimeout(RuntimeError):
    pass


class HWDeviceMgr(FysomGlobalMixin, Device):
    data_type_class = CiA402Device.data_type_class
    device_base_class = CiA402Device
    device_classes = None

    @classmethod
    def device_model_id(cls):
        return cls.name

    feedback_in_defaults = dict(state_cmd=0, quick_stop=0, reset=0)
    feedback_in_data_types = dict(
        state_cmd="uint8", quick_stop="bit", reset="bit"
    )
    feedback_out_defaults = dict(
        state_cmd="init",
        quick_stop=0,
    )

    sim_feedback_defaults = feedback_in_defaults
    sim_feedback_data_types = feedback_in_data_types

    command_in_defaults = dict(
        state_cmd="init",
        state_log="",
        command_complete=False,
        drive_state="SWITCH ON DISABLED",
        drive_control_mode=device_base_class.control_mode_str(
            device_base_class.DEFAULT_CONTROL_MODE
        ),
        drive_control_word_flags=dict(),
        drive_state_flags=dict(),
        reset=False,
    )
    command_out_defaults = dict(state_cmd=0, reset=0)
    command_out_data_types = dict(state_cmd="uint8", reset="bit")

    ####################################################
    # Initialization

    def __init__(self):
        self.state = "init_command"  # Used by FysomGlobalMixin
        super().__init__()
        self.fast_track = False
        self.mgr_config = None
        self.device_config = None
        self.shutdown = False

    def init(self, mgr_config=None, **kwargs):
        """Initialize Manager instance."""
        self.mgr_config = mgr_config
        super().init(**kwargs)
        self.logger.info("Initialization complete")

    def init_devices(
        self, *, device_config, device_init_kwargs=dict(), **kwargs
    ):
        """
        Populate `HWDeviceMgr` instance `devices` attribute devices.

        Scan devices and configure with data in device configuration
        """
        # Pass config to Config class and scan devices
        assert device_config
        self.init_device_classes(device_config=device_config)
        self.devices = self.scan_devices(**kwargs)
        self.init_device_instances(**device_init_kwargs)

    def init_device_classes(self, device_config=None):
        assert device_config
        self.device_config = device_config
        self.device_base_class.set_device_config(device_config)

    @classmethod
    def scan_devices(cls, **kwargs):
        return cls.device_base_class.scan_devices(**kwargs)

    def init_device_instances(self, **kwargs):
        for i, dev in enumerate(self.devices):
            dev.init(index=i, **kwargs)
            self.logger.info(f"Adding device #{i}: {dev}")

    ####################################################
    # Drive state FSM

    GSM = FysomGlobal(
        initial=dict(state="init_command", event="init_command", defer=False),
        events=[
            # Init state:  From initial state
            # - init_1:  (Nothing) Wait for devices to come online
            # - init_2:  Check and update drive params
            # - init_complete:  Done
            dict(name="init_command", src="init_command", dst="init_1"),
            dict(name="init_2", src="init_1", dst="init_2"),
            dict(name="init_complete", src="init_2", dst="init_complete"),
            # Fault state:  From any state
            # - fault:  done
            dict(name="fault_command", src="*", dst="fault_1"),
            dict(name="fault_complete", src="fault_1", dst="fault_complete"),
            # Start state:  From any state but 'start*'
            # - start_1:  Switch all devices to SWITCHED ON
            # - start_2:  Set all drive control modes to default
            # - start_3:  Switch all devices to OPERATION ENABLED
            # - start_complete:  Done
            dict(name="start_command", src="*", dst="start_1"),
            dict(name="start_2", src="start_1", dst="start_2"),
            dict(name="start_3", src="start_2", dst="start_3"),
            dict(name="start_complete", src="start_3", dst="start_complete"),
            # Stop state:  From any state but 'stop*'
            # - stop_1:  Put all devices in SWITCH ON DISABLED and CSP mode
            # - stop_complete:  Done
            dict(name="stop_command", src="*", dst="stop_1"),
            dict(name="stop_complete", src="stop_1", dst="stop_complete"),
            # Home state:  From only 'stop_complete'
            # - home_1:  Set all drive control modes to HM
            # - home_2:  Switch all devices to OPERATION ENABLED
            # - home_3:  Set home flag
            # - home_complete:  Done; issue 'stop' command
            dict(name="home_command", src="*", dst="home_1"),
            dict(name="home_2", src="home_1", dst="home_2"),
            dict(name="home_3", src="home_2", dst="home_3"),
            dict(name="home_complete", src="home_3", dst="home_complete"),
        ],
        state_field="state",
    )

    #
    # Init command
    #
    def on_before_init_command(self, e):
        timeout = self.mgr_config.get("init_timeout", 30)
        return self.fsm_check_command(e, timeout=timeout)

    def on_enter_init_1(self, e):
        self.logger.info("Waiting for devices to come online before init")

    def on_before_init_2(self, e):
        if self.fsm_check_devices_online(e, "INIT"):
            self.logger.info("All devices online; proceeding with init")
            return True
        else:
            return False

    def on_enter_init_2(self, e):
        self.initialize_devices()

    def on_before_init_complete(self, e):
        return self.fsm_check_drive_goal_state(e)

    def on_enter_init_complete(self, e):
        self.fsm_finalize_command(e)
        # Automatically return to SWITCH ON DISABLED after init
        self.command_in.set(
            state_cmd="stop",
            state_log="Automatic 'stop' command at init complete",
        )

    #
    # Fault command
    #
    def on_before_fault_command(self, e):
        if self.fsm_check_command(e):
            self.logger.error("Entering fault state")
            return True
        else:
            return False

    def on_enter_fault_1(self, e):
        self.fsm_set_drive_state_cmd(e, "FAULT")
        return

    def on_before_fault_complete(self, e):
        return self.fsm_check_drive_goal_state(e)

    def on_enter_fault_complete(self, e):
        self.fsm_finalize_command(e)

    #
    # Start command
    #
    def on_before_start_command(self, e):
        return self.fsm_check_command(e)

    def on_enter_start_1(self, e):
        self.fsm_set_required_status_word_flags(e, VOLTAGE_ENABLED=True)
        self.fsm_set_drive_state_cmd(e, "SWITCHED ON")

    def on_before_start_2(self, e):
        return self.fsm_check_drive_goal_state(e)

    def on_enter_start_2(self, e):
        mode = self.device_base_class.DEFAULT_CONTROL_MODE
        self.fsm_set_drive_control_mode(e, mode)

    def on_before_start_3(self, e):
        return self.fsm_check_drive_goal_state(e)

    def on_enter_start_3(self, e):
        # Set reset during transition to OPERATION ENABLED
        e.reset = True
        self.fsm_set_drive_state_cmd(e, "OPERATION ENABLED")

    def on_before_start_complete(self, e):
        return self.fsm_check_drive_goal_state(e)

    def on_enter_start_complete(self, e):
        # Clear reset in OPERATION ENABLED
        e.reset = False
        self.fsm_finalize_command(e)

    #
    # Stop command
    #
    def on_before_stop_command(self, e):
        return self.fsm_check_command(e)

    def on_enter_stop_1(self, e):
        self.fsm_set_required_status_word_flags(e, all_clear=True)
        return self.fsm_set_drive_state_cmd(e, "SWITCH ON DISABLED")

    def on_before_stop_complete(self, e):
        return self.fsm_check_drive_goal_state(e)

    def on_enter_stop_complete(self, e):
        self.fsm_finalize_command(e)

    #
    # Home command
    #
    def on_before_home_command(self, e):
        if not e.src == "stop_complete":
            self.logger.warning("Unable to home when devices not stopped")
            return False
        return self.fsm_check_command(e)

    def on_enter_home_1(self, e):
        self.fsm_set_drive_control_mode(e, "MODE_HM")
        self.fsm_set_required_status_word_flags(e, VOLTAGE_ENABLED=True)

    def on_before_home_2(self, e):
        return self.fsm_check_drive_goal_state(e)

    def on_enter_home_2(self, e):
        self.fsm_set_drive_state_cmd(e, "OPERATION ENABLED")

    def on_before_home_3(self, e):
        return self.fsm_check_drive_goal_state(e)

    def on_enter_home_3(self, e):
        # MODE_HM:  OPERATION_MODE_SPECIFIC_1 = HOMING_START
        # FIXME Is this drive specific?
        self.fsm_set_control_word_flags(e, OPERATION_MODE_SPECIFIC_1=True)
        self.fsm_set_required_status_word_flags(e, HOMING_COMPLETED=True)

    def on_before_home_complete(self, e):
        return self.fsm_check_drive_goal_state(e)

    def on_enter_home_complete(self, e):
        self.fsm_finalize_command(e)
        self.fsm_set_control_word_flags(e, OPERATION_MODE_SPECIFIC_1=False)
        self.fsm_set_required_status_word_flags(e, all_clear=True)
        # Automatically return to SWITCH ON DISABLED after homing
        self.command_in.set(
            state_cmd="stop",
            state_log="Automatic 'stop' command at home complete",
        )

    #
    # All states
    #
    def on_change_state(self, e):
        # This runs after every `on_enter_*`; attrs of e set there (or
        # `on_before_*`) will be present here

        # Set/clear reset command
        reset = getattr(e, "reset", False)
        self.command_in.update(reset=reset)
        if self.command_in.changed("reset"):
            self.logger.info(f"Reset command set to {reset}")

    #
    # Helpers
    #
    cmd_name_to_int_map = {
        "init": 0,
        "stop": 1,
        "start": 2,
        "home": 3,
        "fault": 4,
    }

    cmd_int_to_name_map = {v: k for k, v in cmd_name_to_int_map.items()}

    def timer_start(self, timeout=None):
        if timeout is None:
            timeout = self.mgr_config.get("goal_state_timeout", 30.0)
        self._timeout = time.time() + timeout

    def timer_check_overrun(self, msg):
        if not hasattr(self, "_timeout") or time.time() <= self._timeout:
            return

        msg = f"{self.command_in.get('state_cmd')} timeout:  {msg}"
        self.command_in.set(state_cmd="fault", state_log=msg)
        del self._timeout
        raise HWDeviceTimeout(msg)

    @classmethod
    def fsm_command_from_event(cls, e):
        return e.dst.split("_")[0]

    def fsm_check_devices_online(self, e, state):
        return self.query_devices(oper=False)

    def fsm_check_command(self, e, timeout=None):
        cmd_name = self.fsm_command_from_event(e)
        if (
            e.src.startswith("init") and e.src != "init_complete"
        ) and cmd_name != "init":
            # Don't preempt init (fault)
            msg = f"Ignoring {cmd_name} command in init state {e.src}"
            self.command_in.set(state_cmd="init", state_log=msg)
            if self.command_in.changed("state_cmd"):
                self.logger.warning(msg)
            return False
        elif e.src != f"{cmd_name}_command" and e.src.startswith(cmd_name):
            # Already running
            self.logger.warning(
                f"Ignoring {cmd_name} command from state {e.src}"
            )
            return False
        else:
            self.logger.info(f"Received {cmd_name} command:  {e.msg}")
            self.command_in.set(state_cmd=cmd_name, state_log=e.msg)
            self.command_in.update(command_complete=False)
            self.timer_start(
                timeout=timeout
                or self.mgr_config.get("goal_state_timeout", 5.0)
            )
            return True

    def fsm_check_drive_goal_state(self, e):
        drives_not_reached_goal = self.query_devices(goal_reached=False)
        if not drives_not_reached_goal:
            return True
        # Raise exception if timer expired (pick arbitrary drive for reason)
        drv = drives_not_reached_goal[0]
        reason = drv.feedback_out.get("goal_reason")
        self.timer_check_overrun(f"Drive {drv}:  {reason}")
        # Otherwise, cancel event
        return False

    def fsm_set_drive_state_cmd(self, e, state):
        cmd_name = self.fsm_command_from_event(e)
        self.logger.info(
            f"{cmd_name} command:  Setting drive state command to {state}"
        )
        self.command_in.update(drive_state=state)

    def fsm_set_drive_control_mode(self, e, mode):
        cmd_name = self.fsm_command_from_event(e)
        mode_str = self.device_base_class.control_mode_str(mode)
        self.logger.info(f"{cmd_name} command:  Setting drive mode {mode_str}")
        self.command_in.update(drive_control_mode=mode_str)

    def fsm_set_control_word_flags(self, e, **flags):
        cmd_name = self.fsm_command_from_event(e)
        self.logger.info(
            f"{cmd_name} command:  Setting control word flags {flags}"
        )
        self.command_in.get("drive_control_word_flags").update(flags)

    def fsm_set_required_status_word_flags(self, e, all_clear=False, **flags):
        cmd_name = self.fsm_command_from_event(e)
        if all_clear:
            self.logger.info(
                f"{cmd_name} command:  Clearing required status word flags"
            )
            self.command_in.get("drive_state_flags").clear()
        self.logger.info(
            f"{cmd_name} command:  Setting required status word flags {flags}"
        )
        self.command_in.get("drive_state_flags").update(**flags)

    def fsm_finalize_command(self, e):
        cmd_name = self.fsm_command_from_event(e)
        self.command_in.update(command_complete=True)
        self.logger.info(f"Command {cmd_name} completed")

    ####################################################
    # Execution

    def run_loop(self):
        """Program main loop."""
        update_period = 1.0 / self.mgr_config.get("update_rate", 10.0)
        while not self.shutdown:
            try:
                self.read_update_write()
            except Exception:
                # Ignore other exceptions & enter fault mode in
                # hopes we can recover
                self.logger.error("Ignoring unexpected exception; details:")
                for line in traceback.format_exc().splitlines():
                    self.logger.error(line)
                self.command_in.set(
                    state_cmd="fault", msg_log="Unexpected exception"
                )
            if self.fast_track:
                # This update included a state transition; skip
                # the `sleep()` before the next update
                self.fast_track = False
                continue
            time.sleep(update_period)

    def run(self):
        """Program main."""
        try:
            self.run_loop()
        except KeyboardInterrupt:
            self.logger.info("Exiting at keyboard interrupt")
            return 0
        except Exception:
            self.logger.error("Exiting at unrecoverable exception:")
            for line in traceback.format_exc().splitlines():
                self.logger.error(line)
            return 1
        self.logger.info("Exiting")
        return 0

    def read_update_write(self):
        """
        Read hardware, update controller, write hardware.

        Handle known exceptions
        """
        try:
            self.read()
            self.get_feedback()
            self.set_command()
            self.write()
        except KeyboardInterrupt as e:
            self.logger.info(f"KeyboardInterrupt: {e}")
            self.shutdown = True
        except HWDeviceTimeout as e:
            self.logger.error(str(e))

    fsm_next_state_map = dict(
        # Map current command to dict of {current_state:next_event}
        # names; `None` means arrived
        init=dict(
            init_1="init_2",
            init_2="init_complete",
            init_complete=None,
        ),
        start=dict(
            start_1="start_2",
            start_2="start_3",
            start_3="start_complete",
            start_complete=None,
        ),
        stop=dict(
            stop_1="stop_complete",
            stop_complete=None,
        ),
        home=dict(
            home_1="home_2",
            home_2="home_3",
            home_3="home_complete",
            home_complete=None,
        ),
        fault=dict(
            fault_1="fault_complete",
            fault_complete=None,
        ),
    )

    def read(self):
        """Read manager and device external feedback."""
        super().read()
        for dev in self.devices:
            dev.read()

    def get_feedback(self):
        """Process manager and device external feedback."""
        fb_out = super().get_feedback()

        # Incoming commanded state
        state_cmd = self.cmd_int_to_name_map[self.feedback_in.get("state_cmd")]
        quick_stop = self.feedback_in.get("quick_stop")
        fb_out.update(state_cmd=state_cmd, quick_stop=quick_stop)

        # Get device feedback
        for dev in self.devices:
            dev.get_feedback()

        return fb_out

    def set_command(self):
        """
        Set command for top-level manager and for drives.

        Manager `command_in` is derived from manager & drive
        `feedback_out` rather than being passed down via the controller
        API.
        """
        # Initialize command in/out interfaces with previous cycle values
        cmd_in = self.command_in
        cmd_out = super().set_command(**cmd_in.get())

        # Check for state command from feedback
        if self.feedback_in.changed("state_cmd"):
            # Other commands from state_cmd fb; can't override fault
            cmd_in.update(
                state_cmd=self.feedback_out.get("state_cmd"),
                state_log="state command feedback changed",
            )

        # Special cases where 'fault' overrides current command:
        if self.state.startswith("init"):
            # Fault isn't allowed to override init; don't spam logs about
            # ignoring 'fault' state cmd
            pass
        elif self.query_devices(oper=False):
            # Treat devices not operational as a fault
            fds = self.query_devices(oper=False)
            fd_addrs = ", ".join(str(d.address) for d in fds)
            cmd_in.update(
                state_cmd="fault",
                state_log=f"Devices at ({fd_addrs}) not online and operational",
            )
        elif self.query_devices(state="FAULT") and self.query_devices(
            state="changed"
        ):
            # Devices went into FAULT state since last update
            fds = self.query_devices(state="FAULT")
            fd_addrs = ", ".join(str(d.address) for d in fds)
            cmd_in.update(
                state_cmd="fault",
                state_log=f"Devices at ({fd_addrs}) in FAULT state",
            )
        elif (
            self.feedback_in.get("quick_stop")
            and cmd_in.get("state_cmd") != "fault"
        ):
            # Quick stop feedback high; treat this as a fault command
            cmd_in.update(
                state_cmd="fault",
                state_log="quick_stop pin high",
            )
        elif self.query_devices(
            state_flags=lambda x: not x["VOLTAGE_ENABLED"]
        ) and cmd_in.get("state_cmd") not in ("stop", "fault"):
            # Some devices have no motor power
            cmd_in.update(
                state_cmd="fault",
                state_log="voltage_enabled bit low:  no motor power at drive",
            )

        if cmd_in.changed("state_cmd"):
            # Received new command to stop/start/home/fault.  Try it
            # by triggering the FSM event; a Canceled exception means
            # it can't be done, so ignore it.
            event = f"{cmd_in.get('state_cmd')}_command"
            try:
                self.trigger(event, msg=cmd_in.get("state_log"))
            except Canceled:
                self.logger.warning(f"Unable to honor {event} command")

        # Attempt automatic transition to next state
        event = self.automatic_next_event()
        if event is not None:
            try:
                self.trigger(
                    event, msg=f"Automatic transition from {self.state} state"
                )
            except Canceled:
                # `on_before_{event}()` method returned `False`,
                # causing `fysom.Canceled` exception
                # self.logger.debug(f"Cannot transition to next state {event}")
                pass
            else:
                # State transition succeeded; fast-track the next update
                self.fast_track = True

        # Set command, incl. drive command, and return
        state_cmd = self.cmd_name_to_int_map[cmd_in.get("state_cmd")]
        reset = cmd_in.get("reset")
        cmd_out.update(state_cmd=state_cmd, reset=reset)
        self.set_drive_command()
        return cmd_out

    def write(self):
        """Write manager and device external command."""
        super().write()
        for dev in self.devices:
            dev.write()

    def automatic_next_event(self):
        state_cmd = self.command_in.get("state_cmd")
        state_map = self.fsm_next_state_map[state_cmd]
        event = state_map.get(self.state, f"{state_cmd}_command")
        return event

    ####################################################
    # Drive helpers

    @classmethod
    def init_sim(cls, *, sim_device_data):
        cls.device_base_class.init_sim(sim_device_data=sim_device_data)

    def initialize_devices(self):
        # Ensure drive parameters are up to date
        self.logger.info("Initializing devices")
        self.logger.info("- Setting drive params volatile")
        for d in self.devices:
            d.set_params_volatile()
        self.logger.info("- Updating drive params")
        for d in self.devices:
            d.write_config_param_values()

    def set_drive_command(self):
        for drive in self.devices:
            drive.set_command(
                state=self.command_in.get("drive_state"),
                control_mode=self.command_in.get("drive_control_mode"),
                control_word_flags=self.command_in.get(
                    "drive_control_word_flags"
                ),
                state_flags=self.command_in.get("drive_state_flags"),
            )

    def query_devices(self, **kwargs):
        res = list()
        for dev in self.devices:
            for key, val in kwargs.items():
                if callable(val):
                    if not val(dev.feedback_out.get(key)):
                        break
                elif val == "changed":
                    if not dev.feedback_out.changed(key):
                        break
                else:
                    if dev.feedback_out.get(key) != val:
                        break
            else:
                res.append(dev)
        return res


class SimHWDeviceMgr(HWDeviceMgr, SimDevice):

    device_base_class = CiA402SimDevice

    def set_sim_feedback(self):
        sfb = super().set_sim_feedback()
        sfb.update(
            state_cmd=self.command_out.get("state_cmd"),
            quick_stop=self.feedback_in.get("quick_stop"),
        )
        return sfb
