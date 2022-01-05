import abc
from ..cia_301 import CiA301Device
from .config import EtherCATConfig
from .data_types import EtherCATDataType


class EtherCATDevice(CiA301Device, abc.ABC):
    """Abstract class representing an EtherCAT CoE device

    Device instances are addressed by `(master, position)`.

    Device model subclasses have matching XML description, methods
    (e.g. set params volatile) and other features specific to that
    model.
    """

    category = "EtherCAT"

    # Resource names for locating device description XML and error files
    device_xml_dir = "device_xml"

    # Filename of XML description
    xml_description_fname = None

    # Swappable utility classes
    data_type_class = EtherCATDataType
    config_class = EtherCATConfig

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.add_device_sdos()

    @property
    def master(self):
        return self.address[0]

    @property
    def position(self):
        return self.address[1]

    @abc.abstractmethod
    def set_params_volatile(self, nv=False):
        """Set device params volatile or non-volatile

        Concrete subclasses may optionally implement this
        """

    @classmethod
    def xml_description_path(cls):
        """Return path to device ESI file

        Path is under the module directory,
        `{device_xml_dir}/{xml_description_fname}`.
        """
        return cls.pkg_path(cls.device_xml_dir) / cls.xml_description_fname

    @classmethod
    def add_device_sdos(cls):
        """Read device SDOs from ESI file and add to configuration"""
        cls.config_class.add_device_sdos(cls.xml_description_path())
