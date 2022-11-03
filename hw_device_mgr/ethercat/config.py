from .sdo import EtherCATSDO
from ..cia_301.config import CiA301Config, CiA301SimConfig
from .data_types import EtherCATDataType
from .xml_reader import EtherCATXMLReader
from .command import EtherCATCommand, EtherCATSimCommand
from functools import lru_cache


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
    # Device ESI
    #

    @classmethod
    @lru_cache
    def read_esi(cls, package, fname, LcId="1033"):
        """
        Read ESI XML and return `EtherCATXMLReader` object.

        ESI may be a package resource from `package` and `fname`;
        otherwise, if `package` is `None`, a file from path `fname`.
        """
        esi_reader_class = cls.esi_reader_class
        if package:
            esi_reader = esi_reader_class.read_from_resource(
                package, fname, LcId=LcId
            )
        else:
            esi_reader = esi_reader_class.read_from_path(fname, LcId=LcId)
        return esi_reader

    @classmethod
    @lru_cache
    def get_device_sdos_from_esi(cls, package, fname, LcId="1033"):
        """
        Read in device SDOs from ESI.

        The `package` and `fname` args are supplied to the `read_esi`
        method.
        """
        esi_reader = cls.read_esi(package, fname, LcId=LcId)
        return esi_reader.parse_sdos()

    @classmethod
    @lru_cache
    def get_device_dcs_from_esi(cls, package, fname, LcId="1033"):
        """
        Read in device distributed clocks from ESI.

        The `package` and `fname` args are supplied to the `read_esi`
        method.
        """
        esi_reader = cls.read_esi(package, fname, LcId=LcId)
        return esi_reader.parse_dc_opmodes()


class EtherCATSimConfig(EtherCATConfig, CiA301SimConfig):
    command_class = EtherCATSimCommand
