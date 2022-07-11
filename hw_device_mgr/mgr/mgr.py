from ..device import Device, SimDevice
from ..cia_402.device import CiA402Device, CiA402SimDevice


import traceback
import time
from fysom import FysomGlobalMixin, FysomGlobal, Canceled
from functools import lru_cache


class HWDeviceTimeout(RuntimeError):
    pass


class HWDeviceMgr(FysomGlobalMixin, Device):
    data_type_class = CiA402Device.data_type_class
    device_base_class = CiA402Device
    device_classes = None

    @classmethod
    def device_model_id(cls):
        return cls.name

    command_in_defaults = dict(
        state_cmd="init",
    )
    command_in_data_types = dict(
        state_cmd="str",
    )
    command_out_defaults = dict(
        state_cmd=0,
        state_log="(Uninitialized)",
        command_complete=False,
        reset=0,
        drive_state="SWITCH ON DISABLED",
    )
    command_out_data_types = dict(
        state_cmd="uint8",
        state_log="str",
        command_complete="bit",
        reset="bit",
        drive_state="str",
    )

    ####################################################
    # Initialization

    def __init__(self):
        self.state = "init_command"  # Used by FysomGlobalMixin
        super().__init__()

    def init(self, /, mgr_config, device_config, **kwargs):
        """Initialize Manager instance."""
        self.mgr_config = mgr_config
        # Pass device config to Config class
        assert device_config, "Empty device configuration"
        self.device_config = device_config
        self.device_base_class.set_device_config(device_config)
        # Scan & init devices
        self.init_devices(**kwargs)
        # Init self
        super().init()
        self.logger.info("Initialization complete")

    @classmethod
    def init_sim(cls, **kwargs):
        cls.device_base_class.init_sim(**kwargs)

    def init_devices(
        self,
        /,
        sim_device_data=None,
        device_init_kwargs=dict(),
        device_scan_kwargs=dict(),
    ):
        """
        Initialize devices.

        Populate `HWDeviceMgr` instance `devices` attribute devices with
        bus scans.  Configure devices with data from device
        configuration.
        """
        # Initialize sim device discovery data, if any
        self.init_sim_devices(sim_device_data=sim_device_data)

        # Scan and init devices
        self.devices = self.scan_devices(**device_scan_kwargs)
        for dev in self.devices:
            dev.init(**device_init_kwargs)
            self.logger.info(f"Initialized device {dev}")

    @classmethod
    def scan_devices(cls, **kwargs):
        return cls.device_base_class.scan_devices(**kwargs)

    # Mapping of device interface names translated <-> manager interfaces; value
    # is set of attributes to skip.
    device_translated_interfaces = dict(
        # - goal_reached/goal_reason used by mgr
        feedback_out=set(),
        # - Don't expose device `state` cmd, controlled by manager
        command_in={"state"},
    )

    @lru_cache
    def dev_prefix(self, dev, prefix="d", suffix=""):
        return f"{prefix}{dev.addr_slug}{suffix}"


    def init_interfaces(self):
        """Add per-device interface attributes to feedback & command."""
        super().init_interfaces()
        for dev in self.devices:
            prefix = self.dev_prefix(dev, suffix=dev.slug_separator)
            for name, skip_set in self.device_translated_interfaces.items():
                dev_intf = dev.interface(name)
                mgr_intf = self.interface(name)
                for attr_name, val in dev_intf.get().items():
                    if attr_name in skip_set:
                        continue
                    mgr_attr_name = prefix + attr_name
                    mgr_intf.add_attribute(
                        mgr_attr_name, val, dev_intf.get_data_type(attr_name)
                    )


    ####################################################
    # Drive state FSM

    GSM = FysomGlobal(
        initial=dict(state="init_command", event="init_command", defer=False),
        events=[
            # Init state:  From initial state
            # - init_1:  (Nothing) Wait for devices to come online & self-init
            # - init_complete:  Done
            dict(name="init_command", src="init_command", dst="init_1"),
            dict(name="init_complete", src="init_1", dst="init_complete"),
            # Fault state:  From any state
            # - fault:  done
            dict(name="fault_command", src="*", dst="fault_1"),
            dict(name="fault_complete", src="fault_1", dst="fault_complete"),
            # Start state:  From any state but 'start*'
            # - start_1:  Switch all devices to SWITCHED ON
            # - start_2:  Switch all devices to OPERATION ENABLED
            # - start_complete:  Done
            dict(name="start_command", src="*", dst="start_1"),
            dict(name="start_2", src="start_1", dst="start_2"),
            dict(name="start_complete", src="start_2", dst="start_complete"),
            # Stop state:  From any state but 'stop*'
            # - stop_1:  Put all devices in SWITCH ON DISABLED
            # - stop_complete:  Done
            dict(name="stop_command", src="*", dst="stop_1"),
            dict(name="stop_complete", src="stop_1", dst="stop_complete"),
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

    def on_before_init_complete(self, e):
        if self.fsm_check_devices_online(e, "INIT"):
            return False
        return self.fsm_check_drive_goal_state(e)

    def on_enter_init_complete(self, e):
        self.fsm_finalize_command(e)
        # Automatically return to SWITCH ON DISABLED after init
        self.logger.info("Devices all online; commanding stop state")
        self.command_out.update(
            state_cmd=self.cmd_name_to_int_map["stop"],
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
        self.fsm_set_drive_state_cmd(e, "SWITCHED ON")

    def on_before_start_2(self, e):
        return self.fsm_check_drive_goal_state(e)

    def on_enter_start_2(self, e):
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
        return self.fsm_set_drive_state_cmd(e, "SWITCH ON DISABLED")

    def on_before_stop_complete(self, e):
        return self.fsm_check_drive_goal_state(e)

    def on_enter_stop_complete(self, e):
        self.fsm_finalize_command(e)

    #
    # All states
    #
    def on_change_state(self, e):
        # This runs after every `on_enter_*`; attrs of e set there (or
        # `on_before_*`) will be present here

        # Set/clear reset command
        reset = getattr(e, "reset", False)
        self.command_out.update(reset=reset)
        if self.command_out.changed("reset"):
            self.logger.info(f"Reset command set to {reset}")

    #
    # Helpers
    #
    cmd_name_to_int_map = {
        "init": 0,
        "stop": 1,
        "start": 2,
        "fault": 4,
    }

    cmd_int_to_name_map = {v: k for k, v in cmd_name_to_int_map.items()}

    @property
    def state_cmd_str(self):
        return self.cmd_int_to_name_map[self.command_out.get("state_cmd")]

    def timer_start(self, timeout=None):
        if timeout is None:
            timeout = self.mgr_config.get("goal_state_timeout", 30.0)
        self._timeout = time.time() + timeout

    def timer_check_overrun(self, msg):
        if not hasattr(self, "_timeout") or time.time() <= self._timeout:
            return

        msg = f"{self.state_cmd_str} timeout:  {msg}"
        fault_cmd = self.cmd_name_to_int_map["fault"]
        self.command_out.update(state_cmd=fault_cmd, state_log=msg)
        del self._timeout
        raise HWDeviceTimeout(msg)

    @classmethod
    def fsm_command_from_event(cls, e):
        return e.dst.split("_")[0]

    def fsm_check_devices_online(self, e, state):
        return self.query_devices(oper=False)

    def fsm_check_command(self, e, timeout=None):
        state_cmd_str = self.fsm_command_from_event(e)
        state_cmd = self.cmd_name_to_int_map[state_cmd_str]
        if (
            e.src.startswith("init") and e.src != "init_complete"
        ) and state_cmd_str != "init":
            # Don't preempt init (fault)
            msg = f"Ignoring {state_cmd_str} command in init state {e.src}"
            state_cmd = self.cmd_name_to_int_map["init"]
            self.command_out.update(state_cmd=state_cmd, state_log=msg)
            if self.command_out.changed("state_cmd"):
                self.logger.warning(msg)
            return False
        elif e.src != f"{state_cmd_str}_command" and e.src.startswith(
            state_cmd_str
        ):
            # Already running
            self.logger.warning(
                f"Ignoring {state_cmd_str} command from state {e.src}"
            )
            return False
        else:
            self.logger.info(f"Received {state_cmd_str} command:  {e.msg}")
            self.command_out.update(
                state_cmd=state_cmd, state_log=e.msg, command_complete=False
            )
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
        self.command_out.update(drive_state=state)

    def fsm_finalize_command(self, e):
        cmd_name = self.fsm_command_from_event(e)
        self.command_out.update(command_complete=True)
        self.logger.info(f"Command {cmd_name} completed")

    ####################################################
    # Execution

    def run_loop(self):
        """Program main loop."""
        update_period = 1.0 / self.mgr_config.get("update_rate", 10.0)
        self.fast_track = False
        self.shutdown = False
        while not self.shutdown:
            try:
                self.read_update_write()
            except Exception:
                # Ignore other exceptions & enter fault mode in
                # hopes we can recover
                self.logger.error("Ignoring unexpected exception; details:")
                for line in traceback.format_exc().splitlines():
                    self.logger.error(line)
                self.command_out.update(
                    state_cmd=self.cmd_name_to_int_map["fault"],
                    msg_log="Unexpected exception"
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
            init_1="init_complete",
            init_complete=None,
        ),
        start=dict(
            start_1="start_2",
            start_2="start_complete",
            start_complete=None,
        ),
        stop=dict(
            stop_1="stop_complete",
            stop_complete=None,
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
        mgr_fb_out = super().get_feedback()

        # Get device feedback
        for dev in self.devices:
            dev_fb_out = dev.get_feedback()
            prefix = self.dev_prefix(dev, suffix=dev.slug_separator)
            updates = {
                # Copy device fb_out to mgr fb_out, adding prefix
                f"{prefix}{k}":v
                for k,v in dev_fb_out.get().items()
                # ...but skip these keys
                if k not in self.device_translated_interfaces["feedback_out"]
            }
            mgr_fb_out.update(**updates)

        return mgr_fb_out

    def set_command(self, **kwargs):
        """
        Set command for top-level manager and for drives.
        """
        # Initialize command out interface with previous values; this could
        # clobber parent class updates for regular device classes, but this
        # isn't a regular device and it inherits directly from `Device`
        old_cmd_out = self.command_out.get().copy()
        cmd_out = super().set_command(**kwargs)
        cmd_out.update(**old_cmd_out)

        # Special cases where 'fault' overrides current command:
        if self.state.startswith("init"):
            # Fault isn't allowed to override init; don't spam logs about
            # ignoring 'fault' state cmd
            pass
        elif self.query_devices(oper=False):
            # Treat devices not operational as a fault
            fds = self.query_devices(oper=False)
            fd_addrs = ", ".join(str(d.address) for d in fds)
            cmd_out.update(
                state_cmd=self.cmd_name_to_int_map["fault"],
                state_log=f"Devices at ({fd_addrs}) not online and operational",
            )
        elif self.query_devices(state="FAULT") and self.query_devices(
            state="changed"
        ):
            # Devices went into FAULT state since last update
            fds = self.query_devices(state="FAULT")
            fd_addrs = ", ".join(str(d.address) for d in fds)
            cmd_out.update(
                state_cmd=self.cmd_name_to_int_map["fault"],
                state_log=f"Devices at ({fd_addrs}) in FAULT state"
            )
        elif self.query_devices(fault=True) and self.query_devices(
            fault="changed"
        ):
            # Devices set `fault` since last update
            fds = self.query_devices(fault=True)
            fd_addrs = ", ".join(str(d.address) for d in fds)
            cmd_out.update(
                state_cmd=self.cmd_name_to_int_map["fault"],
                state_log=f"Devices at ({fd_addrs}) set fault"
            )
        elif kwargs.get("state_cmd", None) is None:
            pass  # Use previous state_cmd value
        elif kwargs["state_cmd"] not in self.cmd_name_to_int_map:
            state_cmd = kwargs["state_cmd"]
            self.logger.error(f"Invalid state command, '{state_cmd}'")
            cmd_out.update(
                state_cmd=self.cmd_name_to_int_map["fault"],
                state_log=f"Invalid state command, '{state_cmd}'",
            )
        else:
            # Take state_cmd from kwargs
            state_cmd = kwargs["state_cmd"]
            cmd_out.update(
                state_cmd=self.cmd_name_to_int_map[state_cmd],
            )
            if cmd_out.changed("state_cmd"):
                # Assume external command
                cmd_out.update(
                    state_log=f"External command '{state_cmd}'",
                )

        if cmd_out.changed("state_cmd"):
            # Received new command to stop/start/fault.  Try it
            # by triggering the FSM event; a Canceled exception means
            # it can't be done, so ignore it.
            event = f"{self.state_cmd_str}_command"
            try:
                self.trigger(event, msg=cmd_out.get("state_log"))
            except Canceled:
                self.logger.warning(f"Unable to honor {event} command")
        elif self.automatic_next_event() is not None:
            # Attempt automatic transition to next state
            try:
                self.trigger(
                    self.automatic_next_event(),
                    msg=f"Automatic transition from {self.state} state",
                )
            except Canceled:
                # `on_before_{event}()` method returned `False`,
                # causing `fysom.Canceled` exception
                # self.logger.debug(f"Cannot transition to next state {event}")
                pass
            else:
                # State transition succeeded; fast-track the next update
                self.fast_track = True

        # Set drive command and return
        self.set_drive_command()
        return cmd_out

    def write(self):
        """Write manager and device external command."""
        super().write()
        for dev in self.devices:
            dev.write()

    def automatic_next_event(self):
        state_cmd_str = self.state_cmd_str
        state_map = self.fsm_next_state_map[state_cmd_str]
        event = state_map.get(self.state, f"{state_cmd_str}_command")
        return event

    ####################################################
    # Drive helpers

    @classmethod
    def init_sim_devices(cls, /, sim_device_data=None, **kwargs):
        """
        Run `init_sim()` on devices.

        For configurations that include sim devices (even when the device
        manager itself isn't running in sim mode).
        """
        if sim_device_data is None:
            return  # No sim devices to configure
        cls.device_base_class.init_sim(sim_device_data=sim_device_data, **kwargs)

    def set_drive_command(self):
        mgr_vals = self.command_in.get()
        skip = self.device_translated_interfaces.get("command_in", set())
        for dev in self.devices:
            updates = dict()
            if "command_in" in self.device_translated_interfaces:
                # Copy mgr command_out to matching device command_in
                dev_command_in = dev.interface("command_in")
                prefix = self.dev_prefix(dev, suffix=dev.slug_separator)
                dev.set_command(
                    state=self.command_out.get("drive_state"),
                    **{
                        k:mgr_vals[f"{prefix}{k}"]
                        for k in dev_command_in.keys()
                        if k not in skip
                    }
                )
            else:
                dev.set_command(
                    state=self.command_out.get("drive_state"),
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

    def __str__(self):
        return f"<{self.name}>"


class SimHWDeviceMgr(HWDeviceMgr, SimDevice):

    device_base_class = CiA402SimDevice
