import abc
import re
from ..cia_301.device import CiA301Device, CiA301SimDevice
from .config import EtherCATConfig, EtherCATSimConfig
from .data_types import EtherCATDataType
from functools import cached_property


class EtherCATDevice(CiA301Device, abc.ABC):
    """
    Abstract class representing an EtherCAT CoE device.

    Device instances are addressed by `(master, alias, position)`.  For
    no alias, use 0; with alias, use position 0.

    Device model subclasses have matching XML description and other
    features specific to that model.
    """

    # Resource names for locating device description XML and error files
    device_xml_dir = "device_xml"

    # Package and filename of XML description resource
    xml_description_package = None
    xml_description_fname = None

    # Swappable utility classes
    data_type_class = EtherCATDataType
    config_class = EtherCATConfig

    def __init__(self, LcId="1033", **kwargs):
        super().__init__(**kwargs)
        self.add_device_sdos_from_esi(LcId=LcId)
        self.add_device_dcs_from_esi(LcId=LcId)

    @cached_property
    def master(self):
        return self.address[0]

    @cached_property
    def position(self):
        return self.address[1]

    @cached_property
    def alias(self):
        return self.address[2]

    @cached_property
    def addr_slug(self):
        """
        Return a slug generated from the device address.

        EtherCAT slugs have a third alias field.  If alias is non-zero,
        then position field will be zero.  This allows for dynamic
        device positioning, where the alias is known but not the
        position.
        """
        if self.address[2]:  # (0, 14, 4) -> (0, 0, 4)
            address = (self.address[0], 0, self.address[2])
        else:  # (0, 14, 0) -> (0, 14, 0)
            address = self.address
        addr_prefix = re.sub(r"[^0-9]+", self.slug_separator, str(address))
        return addr_prefix.strip(self.slug_separator)

    @classmethod
    def read_device_sdos_from_esi(cls, LcId="1033"):
        sdo_data = dict()
        for dev in cls.get_model():
            conf = dev.config_class
            dev_sdo_data = conf.get_device_sdos_from_esi(
                dev.xml_description_package,
                dev.xml_description_fname,
                LcId=LcId,
            )
            sdo_data.update(dev_sdo_data)
        return sdo_data

    @classmethod
    def add_device_sdos_from_esi(cls, LcId="1033"):
        """Read device SDOs from ESI file and add to configuration."""
        sdo_data = cls.read_device_sdos_from_esi(LcId=LcId)
        cls.add_device_sdos(sdo_data)

    @classmethod
    def munge_sdo_data(cls, sdo_data):
        # SDO data from ESI parser already in correct format
        return sdo_data

    @classmethod
    def read_device_dcs_from_esi(cls, LcId="1033"):
        dcs_data = dict()
        for dev in cls.get_model():
            conf = dev.config_class
            dev_dcs_data = conf.get_device_dcs_from_esi(
                dev.xml_description_package,
                dev.xml_description_fname,
                LcId=LcId,
            )
            dcs_data.update(dev_dcs_data)
        return dcs_data

    @classmethod
    def add_device_dcs_from_esi(cls, LcId="1033"):
        """Read device DCs from ESI file and add to configuration."""
        dcs_data = cls.read_device_dcs_from_esi(LcId=LcId)
        cls.add_device_dcs(dcs_data)


class EtherCATSimDevice(EtherCATDevice, CiA301SimDevice):
    config_class = EtherCATSimConfig

    def __init__(self, **kwargs):
        super().__init__(**kwargs)

    @classmethod
    def init_sim(cls, LcId="1033", **kwargs):
        """
        Configure device, config, command for sim EtherCAT devices.

        Like parent `CiA301SimDevice.init_sim()`, but parse SDO data
        from EtherCAT ESI description file and pass with sim device data
        to parent class's method.
        """
        sdo_data = cls.read_device_sdos_from_esi(LcId=LcId)
        dcs_data = cls.read_device_dcs_from_esi(LcId=LcId)
        super().init_sim(sdo_data=sdo_data, dcs_data=dcs_data, **kwargs)
