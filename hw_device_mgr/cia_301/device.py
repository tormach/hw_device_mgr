from ..device import Device, SimDevice
from .config import CiA301Config, CiA301SimConfig
from .data_types import CiA301DataType
from functools import cached_property, lru_cache


class CiA301Device(Device):
    """
    Abstract class represting a CAN device.

    Implements NMT state machine.
    """

    vendor_id = None
    product_code = None

    data_type_class = CiA301DataType
    config_class = CiA301Config

    # Parameter update state
    PARAM_STATE_UNKNOWN = 0  # Uninitialized and unchecked before init
    PARAM_STATE_UPDATING = 1  # Currently being checked & updated
    PARAM_STATE_COMPLETE = 2  # Params checked and updated
    PARAM_STATE_ERROR = 3  # Error in param init

    feedback_in_data_types = dict(online="bit", oper="bit")
    feedback_in_defaults = dict(online=False, oper=False)

    feedback_out_data_types = dict(
        param_state="uint8", **feedback_in_data_types
    )
    feedback_out_defaults = dict(
        param_state=PARAM_STATE_UNKNOWN, **feedback_in_defaults
    )

    @classmethod
    def canon_address(cls, address):
        """Canonicalize a device address."""
        return cls.config_class.canon_address(address)

    @property
    def goal_reached_timeout(self):
        """Increase goal_reached timeout before reaching oper state."""
        if not self.feedback_in.get("oper"):
            return 30
        p_state = self.feedback_out.get("param_state")
        if self.config.init_params and p_state != self.PARAM_STATE_COMPLETE:
            return 30
        return 10

    def __init__(
        self, address=None, skip_optional_config_values=True, **kwargs
    ):
        if hasattr(address, "address"):
            # Config passed in instead of address; reuse it
            config = address
            address = config.address
        else:
            config = self.config_class(
                address=address,
                model_id=self.model_id,
                skip_optional_config_values=skip_optional_config_values,
            )
        config.set_name(f"{self.name}_cfg")
        self.config = config
        super().__init__(address=address, **kwargs)

    def clear_cached_properties(self, *args):
        super().clear_cached_properties(*args)
        self.config.clear_cached_properties()

    @classmethod
    @lru_cache
    def device_model_id(cls):
        """
        Return unique device model identifier.

        CiA 301 device models are uniquely identified by vendor ID &
        product code.
        """
        model_id = cls.vendor_id, cls.product_code
        return cls.config_class.format_model_id(model_id)

    @cached_property
    def model_id(self):
        return self.device_model_id()

    @classmethod
    def set_device_config(cls, device_config):
        assert device_config
        cls.config_class.set_device_config(device_config)

    def get_feedback(self):
        fb_out = super().get_feedback()
        if not fb_out.get("goal_reached"):
            # Stop param init
            return fb_out  # Don't clobber lower layers' reasons
        if not self.feedback_in.get("online"):
            fb_out.update(
                goal_reached=False,
                goal_reason="Offline",
                param_state=self.PARAM_STATE_UNKNOWN,
            )
            if self.feedback_in.changed("online"):
                msg = "Drive went offline/non-operational"
                fb_out.update(fault=True, fault_desc=msg)
                self.logger.error(msg)
            elif fb_out.get_old("fault"):
                # Prev update offline; fault probably from going offline
                fb_out.update(
                    fault=True, fault_desc=fb_out.get_old("fault_desc")
                )
            # Stop param init
            return fb_out  # Nothing more to do

        if self.feedback_in.changed("online"):
            self.logger.info("Drive came online")

        # Device online; update CiA301 feedback
        goal_reached, goal_reasons = True, list()

        # Param init:  download param values asynchronously after coming online
        old_ps = fb_out.get_old("param_state")
        p_init_err = self.config.param_init_error
        if not self.config.init_params:
            param_state = self.PARAM_STATE_COMPLETE
        elif p_init_err:
            try:
                errstr = "{1}({2}, {3}): {0}".format(*p_init_err)
            except Exception:
                errstr = str(p_init_err)
            fb_out.update(fault=True, fault_desc=f"param init failed: {errstr}")
            param_state = self.PARAM_STATE_ERROR
        elif self.config.param_init_in_progress:
            goal_reached = False
            goal_reasons.append("updating device params")
            param_state = self.PARAM_STATE_UPDATING
        elif self.command_out.get("init_params"):
            goal_reached = False
            goal_reasons.append("updating device params")
            param_state = self.PARAM_STATE_UPDATING
        elif old_ps in (self.PARAM_STATE_UPDATING, self.PARAM_STATE_COMPLETE):
            # Previously complete, or previously updating but currently not
            param_state = self.PARAM_STATE_COMPLETE
        else:
            # Catch all, esp. after entering online state
            param_state = self.PARAM_STATE_UNKNOWN
            goal_reached = False
            goal_reasons.append("device params unset")

        # Update operational status
        if not self.feedback_in.get("oper"):
            if self.command_in.get("shutdown"):
                if self.feedback_in.changed("oper"):
                    self.logger.info("Drive non-operational, shutdown complete")
                fb_out.update(shutdown_complete=True)
                return fb_out  # goal reached
            goal_reached = False
            goal_reasons.insert(0, "Not operational")

            if self.feedback_in.changed("oper"):
                msg = "Drive went non-operational"
                fb_out.update(fault=True, fault_desc=msg)
                self.logger.error(msg)
            elif fb_out.get_old("fault"):
                # Prev update not oper; fault probably from going not oper
                fb_out.update(
                    fault=True, fault_desc=fb_out.get_old("fault_desc")
                )
        else:  # operational
            if self.feedback_in.changed("oper"):
                self.logger.info("Drive came online/operational")
            if self.command_out.get("shutdown_latch"):
                goal_reached = False
                goal_reasons.insert(0, "Drive operational during shutdown")
                fb_out.update(shutdown_complete=False)

        # Update feedback and return
        goal_reason = "Reached" if goal_reached else ", ".join(goal_reasons)
        fb_out.update(
            goal_reached=goal_reached,
            goal_reason=goal_reason,
            param_state=param_state,
        )
        if fb_out.rising_edge("param_state", self.PARAM_STATE_COMPLETE):
            self.logger.info("Device param init complete")
        return fb_out

    command_out_data_types = dict(
        init_params="bit",
    )

    command_out_defaults = dict(
        init_params=False,
    )

    def set_command(self, **kwargs):
        cmd_out = super().set_command(**kwargs)
        cmd_in = self._interfaces["command_in"]
        init_params_cmd = False
        if cmd_out.get("shutdown_latch"):
            self.config.param_init_stop()
        elif self.feedback_in.rising_edge("online"):
            self.logger.info("Initializing params after coming online")
            init_params_cmd = True
        elif cmd_in.rising_edge("reset_fault") and self.config.param_init_error:
            self.logger.info("Re-initializing params after fault")
            init_params_cmd = True
        if init_params_cmd:
            self.config.initialize_params()
            cmd_out.update(init_params=True)
        return cmd_out

    @classmethod
    def munge_sdo_data(cls, sdo_data):
        # Turn per-model name SDO data from YAML into per-model_id SDO data
        res = dict()
        for model_id, sd in sdo_data.items():
            device_cls = cls.get_model(model_id)
            model_id = device_cls.device_model_id()
            res[model_id] = sd
        assert res
        assert None not in res
        return res

    @classmethod
    def add_device_sdos(cls, sdo_data):
        """
        Configure device SDOs.

        Pass to the `Config` class the information needed to configure
        SDOs for this `model_id`.
        """
        cls.config_class.add_device_sdos(cls.munge_sdo_data(sdo_data))

    @classmethod
    def add_device_dcs(cls, dcs_data):
        """
        Configure device distributed clocks.

        Pass to the `Config` class the information needed to configure
        DCs for this `model_id`.
        """
        cls.config_class.add_device_dcs(dcs_data)

    @classmethod
    def get_device(cls, address=None, **kwargs):
        registry = cls._address_registry.setdefault(cls.name, dict())
        config = address
        address = (
            config.address if isinstance(address, cls.config_class) else address
        )
        if address in registry:
            return registry[address]
        # kwargs will contain skip_optional_config_values at this point, but it
        # will be consumed by __init__ for this class
        device_obj = cls(address=config, **kwargs)
        registry[address] = device_obj
        return device_obj

    @classmethod
    def scan_devices(cls, bus=0, get_device_kwargs=dict(), **kwargs):
        """Scan bus and return a list of device objects."""
        devices = list()
        config_cls = cls.config_class
        # Init actual config class instances here. kwargs will contain
        # skip_optional_config_values which is consumed by CiA301Config.scan_bus
        for config in config_cls.scan_bus(bus=bus, **kwargs):
            device_cls = cls.get_model(config.model_id)
            if device_cls is None:
                raise NotImplementedError(
                    f"Unknown model {config.model_id} at {config.address}"
                )
            dev = device_cls.get_device(config, **get_device_kwargs)
            devices.append(dev)
        return devices

    @classmethod
    def init_class(cls, *, sdo_data, dcs_data, **kwargs):
        super().init_class(**kwargs)
        cls.add_device_sdos(sdo_data)
        cls.add_device_dcs(dcs_data)


class CiA301SimDevice(CiA301Device, SimDevice):
    """Simulated CAN device."""

    config_class = CiA301SimConfig

    sim_feedback_data_types = CiA301Device.feedback_in_data_types
    sim_feedback_defaults = CiA301Device.feedback_in_defaults

    @classmethod
    def set_device_config(cls, config):
        # Configs contain "category"; match those with device classes
        # & add "vendor_id" and "product_code" model ID keys.  This
        # used in tests, where the same device config file is reused
        # for different classes with different model IDs.
        uint16 = cls.data_type_class.by_shared_name("uint16")
        config_cooked = list()
        for c in config:
            if "category" not in c:
                model_id = (uint16(c["vendor_id"]), uint16(c["product_code"]))
                c["vendor_id"], c["product_code"] = model_id
                device_cls = cls.get_model(model_id=model_id)
                c["category"] = device_cls.category
                config_cooked.append(c)
                continue
            device_cls = cls.category_cls(c["category"])
            if device_cls is None:
                # Category may be irrelevant, e.g. IO device in servo tests
                continue
            # Fill in values from the device class
            c["vendor_id"] = device_cls.vendor_id
            c["product_code"] = device_cls.product_code
            config_cooked.append(c)
        assert config_cooked
        super().set_device_config(config_cooked)

    @classmethod
    def sim_device_data_class(cls, sim_device_data):
        model_id = cls.config_class.format_model_id(
            (sim_device_data["vendor_id"], sim_device_data["product_code"])
        )
        model = cls.get_model(model_id)
        assert model, f"Unknown model ID {model_id}"
        return model

    @classmethod
    def init_class(cls, **kwargs):
        super().init_class(**kwargs)
        config_kwargs = dict()
        if issubclass(cls.config_class, CiA301SimConfig):
            sim_device_data = cls._sim_device_data[cls.category]
            config_kwargs = dict(sim_device_data=sim_device_data)
        cls.config_class.init_class(**config_kwargs)

    def set_sim_feedback(self, **kwargs):
        # Automatically step through to online/oper
        sfb = super().set_sim_feedback(**kwargs)
        if self.command_in.get("shutdown"):
            sfb.update(online=True, oper=False)
        elif self.feedback_in.get("online"):
            sfb.update(online=True, oper=True)
        else:
            sfb.update(online=True, oper=False)
        return sfb
