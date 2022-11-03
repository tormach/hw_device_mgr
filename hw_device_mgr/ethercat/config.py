from .sdo import EtherCATSDO
from ..cia_301.config import CiA301Config, CiA301SimConfig
from .data_types import EtherCATDataType
from .xml_reader import EtherCATXMLReader
from .command import EtherCATCommand, EtherCATSimCommand


class EtherCATConfig(CiA301Config):
    """
    EtherCAT drive configuration interface.

    EtherCAT CoE shares much in common with CiA 301, and this class
    subclasses `CiA301Config`.

    Each EtherCAT device comes with an ESI "EtherCAT Slave
    Information" XML file that describes the device's object
    dictionary, sync managers, etc.
    """

    data_type_class = EtherCATDataType
    sdo_class = EtherCATSDO
    esi_reader_class = EtherCATXMLReader
    command_class = EtherCATCommand

    #
    # Object dictionary
    #

    @classmethod
    def get_device_sdos_from_esi(cls, package, fname):
        """Read in device configuration from ESI file at `esi_path`."""
        esi_reader = cls.esi_reader_class()
        if package:
            return esi_reader.add_device_descriptions_from_resource(
                package, fname
            )
        else:
            return esi_reader.add_device_descriptions_from_path(fname)


class EtherCATSimConfig(EtherCATConfig, CiA301SimConfig):
    command_class = EtherCATSimCommand
