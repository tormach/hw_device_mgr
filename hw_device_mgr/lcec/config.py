from ..ethercat.config import EtherCATConfig, EtherCATSimConfig
from .data_types import LCECDataType
from .xml_reader import LCECXMLReader
from .sdo import LCECSDO
from .command import LCECCommand, LCECSimCommand


class LCECConfig(EtherCATConfig):
    """Configuration for linuxcnc-ethercat and IgH EtherCAT Master."""

    data_type_class = LCECDataType
    esi_reader_class = LCECXMLReader
    sdo_class = LCECSDO
    command_class = LCECCommand


class LCECSimConfig(LCECConfig, EtherCATSimConfig):
    command_class = LCECSimCommand
