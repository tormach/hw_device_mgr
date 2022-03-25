from ..device import Device, SimDevice
from .config import CiA301Config, CiA301SimConfig
from .data_types import CiA301DataType


class CiA301Device(Device):
    """
    Abstract class represting a CAN device.

    Implements NMT state machine.
    """

    vendor_id = None
    product_code = None

    data_type_class = CiA301DataType
    config_class = CiA301Config

    feedback_in_data_types = dict(online="bit", oper="bit")
    feedback_in_defaults = dict(online=False, oper=False)

    def __init__(self, address=None, **kwargs):
        if isinstance(address, self.config_class):
            self.config = address
            address = address.address
        else:
            self.config = self.config_class(
                address=address, model_id=self.model_id
            )
        super().__init__(address=address, **kwargs)

    @classmethod
    def device_model_id(cls):
        """
        Return unique device model identifier.

        CiA 301 device models are uniquely identified by vendor ID &
        product code.
        """
        model_id = cls.vendor_id, cls.product_code
        return cls.config_class.format_model_id(model_id)

    @property
    def model_id(self):
        return self.device_model_id()

    @classmethod
    def set_device_config(cls, device_config):
        assert device_config
        cls.config_class.set_device_config(device_config)

    def write_config_param_values(self):
        self.config.write_config_param_values()

    def get_feedback(self):
        fb_out = super().get_feedback()
        if not fb_out.get("goal_reached"):
            return fb_out  # Don't clobber lower layers' reasons
        if not self.feedback_in.get("online"):
            fb_out.update(goal_reached=False, goal_reason="Offline")
        elif not self.feedback_in.get("oper"):
            fb_out.update(goal_reached=False, goal_reason="Not operational")
        return fb_out

    def log_status(self):
        super().log_status()
        self.log_operational_changes()

    def log_operational_changes(self):
        if self.feedback_in.changed("online"):
            st_new, st_old = self.feedback_in.changed("online", True)
            self.logger.info(f"{self} online status was {st_old}, now {st_new}")
        if self.feedback_in.changed("oper"):
            st_new, st_old = self.feedback_in.changed("oper", True)
            self.logger.info(
                f"{self} operational status was {st_old}, now {st_new}"
            )

    @classmethod
    def munge_sdo_data(cls, sdo_data):
        # Turn per-model name SDO data from YAML into per-model_id SDO data
        res = dict()
        for model_name, sd in sdo_data.items():
            device_cls = cls.get_model_by_name(model_name)
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
    def get_device(cls, address=None, **kwargs):
        registry = cls._address_registry.setdefault(cls.name, dict())
        config = address
        address = (
            config.address if isinstance(address, cls.config_class) else address
        )
        if address in registry:
            return registry[address]
        device_obj = cls(address=config, **kwargs)
        registry[address] = device_obj
        return device_obj

    @classmethod
    def scan_devices(cls, bus=0, **kwargs):
        """Scan bus and return a list of device objects."""
        devices = list()
        config_cls = cls.config_class
        for config in config_cls.scan_bus(bus=bus):
            device_cls = cls.get_model(config.model_id)
            if device_cls is None:
                raise NotImplementedError(
                    f"Unknown model {config.model_id} at {config.address}"
                )
            dev = device_cls.get_device(config.address, **kwargs)
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
    def sim_device_data_address(cls, sim_device_data):
        model_id = sim_device_data["bus"], sim_device_data["position"]
        sim_device_data["model_id"] = model_id
        return model_id

    @classmethod
    def init_sim(cls, *, sim_device_data, sdo_data):
        super().init_sim(sim_device_data=sim_device_data)
        sim_device_data = cls._sim_device_data[cls.category]
        cls.add_device_sdos(sdo_data)
        cls.config_class.init_sim(sim_device_data=sim_device_data)

    def set_sim_feedback(self, **kwargs):
        # Automatically step through to online/oper
        sfb = super().set_sim_feedback(**kwargs)
        if self.feedback_in.get("online"):
            sfb.update(online=True, oper=True)
        else:
            sfb.update(online=True, oper=False)
        return sfb
