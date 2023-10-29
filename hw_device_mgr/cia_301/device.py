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
        return 10 if self.feedback_in.get("oper") else 30

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
        self.config = config
        super().__init__(address=address, **kwargs)

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

        # Device online; update CiA301 feedback
        goal_reached, goal_reasons = True, list()

        # Param init:  download param values asynchronously after coming online
        if self.feedback_in.changed("online"):
            self.config.initialize_params(restart=True)
            goal_reached = False
            goal_reasons.append("updating device params")
            param_state = self.PARAM_STATE_UPDATING
        elif self.config.initialize_params():
            param_state = self.PARAM_STATE_COMPLETE
        else:
            param_state = self.PARAM_STATE_UPDATING

        # Update operational status
        if not self.feedback_in.get("oper"):
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

        # Update feedback and return
        goal_reason = "Reached" if goal_reached else ", ".join(goal_reasons)
        fb_out.update(
            goal_reached=goal_reached,
            goal_reason=goal_reason,
            param_state=param_state,
        )
        if goal_reached and fb_out.changed("param_state"):
            self.logger.info("Device param init complete")
        if not goal_reached and fb_out.changed("goal_reason"):
            self.logger.info(f"Goal not reached: {goal_reason}")
        return fb_out

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
        address = config.address if hasattr(address, "address") else address
        if address in registry:
            return registry[address]
        # kwargs will contain skip_optional_config_values at this point, but it
        # will be consumed by __init__ for this class
        device_obj = cls(address=config, **kwargs)
        registry[address] = device_obj
        return device_obj

    @classmethod
    def scan_devices(cls, bus=0, **kwargs):
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
            dev = device_cls.get_device(config, **kwargs)
            devices.append(dev)
        return devices


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
                assert device_cls is not None, f"unknown model ID {model_id}"
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
    def init_sim(cls, *, sim_device_data, sdo_data, dcs_data):
        super().init_sim(sim_device_data=sim_device_data)
        sim_device_data = cls._sim_device_data[cls.category]
        cls.add_device_sdos(sdo_data)
        cls.add_device_dcs(dcs_data)
        cls.config_class.init_sim(sim_device_data=sim_device_data)

    def set_sim_feedback(self, **kwargs):
        # Automatically step through to online/oper
        sfb = super().set_sim_feedback(**kwargs)
        if self.feedback_in.get("online"):
            sfb.update(online=True, oper=True)
        else:
            sfb.update(online=True, oper=False)
        return sfb
