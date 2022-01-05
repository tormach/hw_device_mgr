from ..device import Device
from .config import CiA301Config
from .data_types import CiA301DataType


class CiA301Device(Device):
    """Abstract class represting a CAN device

    Implements NMT state machine
    """

    category = "cia_301"
    vendor_id = None
    product_code = None

    data_type_class = CiA301DataType
    config_class = CiA301Config

    feedback_in_data_types = dict(online="bit", oper="bit")

    feedback_in_defaults = dict(online=False, oper=False)

    sim_feedback_data_types = feedback_in_data_types
    sim_feedback_defaults = feedback_in_defaults

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
    def device_type_key(cls):
        """CiA 301 device models are uniquely identified by vendor ID &
        product code"""
        model_id = cls.vendor_id, cls.product_code
        return cls.config_class.format_model_id(model_id)

    @property
    def model_id(self):
        return self.device_type_key()

    @classmethod
    def set_global_device_configuration(cls, config):
        cls.config_class.set_global_device_configuration(config)

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

    def set_sim_feedback(self, **kwargs):
        # Automatically step through to online/oper
        sfb = super().set_sim_feedback(**kwargs)
        if self.feedback_in.get("online"):
            sfb.update(online=True, oper=True)
        else:
            sfb.update(online=True, oper=False)
        return sfb

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
    def add_device_sdos(cls, *args, **kwargs):
        """Pass to the `Config` class the information needed to configure SDOs
        for this `model_id`"""
        cls.config_class.add_device_sdos(*args, **kwargs)

    @classmethod
    def get_device(cls, address=None, sim=False, **kwargs):
        registry = cls._address_registry.setdefault(cls.name, dict())
        config = address
        address = (
            config.address if isinstance(address, cls.config_class) else address
        )
        if address in registry:
            return registry[address]
        device_obj = cls(address=config, sim=sim, **kwargs)
        registry[address] = device_obj
        return device_obj

    @classmethod
    def scan_devices(cls, bus=0, **kwargs):
        """Scan bus and return a list of device objects"""
        devices = list()
        config_cls = cls.config_class
        for config in config_cls.scan_bus(bus=bus):
            try:
                device_cls = cls.get_model(config.model_id)
            except NotImplementedError as e:
                raise NotImplementedError(f"{e} at {config.address}")
            dev = device_cls.get_device(config.address, **kwargs)
            devices.append(dev)
        return devices
