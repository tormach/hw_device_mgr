import abc
from ..cia_301.device import CiA301Device, CiA301SimDevice
from .config import EtherCATConfig, EtherCATSimConfig
from .data_types import EtherCATDataType


class EtherCATDevice(CiA301Device, abc.ABC):
    """
    Abstract class representing an EtherCAT CoE device.

    Device instances are addressed by `(master, position)`.

    Device model subclasses have matching XML description, methods
    (e.g. set params volatile) and other features specific to that
    model.
    """

    # Resource names for locating device description XML and error files
    device_xml_dir = "device_xml"

    # Package and filename of XML description resource
    xml_description_package = None
    xml_description_fname = None

    # Swappable utility classes
    data_type_class = EtherCATDataType
    config_class = EtherCATConfig

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.add_device_sdos_from_esi()

    @property
    def master(self):
        return self.address[0]

    @property
    def position(self):
        return self.address[1]

    @abc.abstractmethod
    def set_params_volatile(self, nv=False):
        """
        Set device params volatile or non-volatile.

        Concrete subclasses may optionally implement this
        """

    @classmethod
    def read_device_sdos_from_esi(cls):
        sdo_data = dict()
        for dev in cls.get_model():
            conf = dev.config_class
            dev_sdo_data = conf.get_device_sdos_from_esi(
                dev.xml_description_package, dev.xml_description_fname
            )
            sdo_data.update(dev_sdo_data)
        return sdo_data

    @classmethod
    def add_device_sdos_from_esi(cls):
        """Read device SDOs from ESI file and add to configuration."""
        sdo_data = cls.read_device_sdos_from_esi()
        cls.add_device_sdos(sdo_data)

    @classmethod
    def munge_sdo_data(cls, sdo_data):
        # SDO data from ESI parser already in correct format
        return sdo_data


class EtherCATSimDevice(EtherCATDevice, CiA301SimDevice):
    config_class = EtherCATSimConfig

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.params_volatile = False

    def set_params_volatile(self, nv=False):
        self.params_volatile = not nv

    @classmethod
    def init_sim(cls, **kwargs):
        """
        Configure device, config, command for sim EtherCAT devices.

        Like parent `CiA301SimDevice.init_sim()`, but parse SDO data
        from EtherCAT ESI description file and pass with sim device data
        to parent class's method.
        """
        sdo_data = cls.read_device_sdos_from_esi()
        super().init_sim(sdo_data=sdo_data, **kwargs)
