from ..ethercat.config import EtherCATConfig
from .data_types import LCECDataType
from .xml_reader import LCECXMLReader
from .sdo import LCECSDO
from .command import LCECCommand


class LCECConfig(EtherCATConfig):
    """Concrete EtherCAT configuration interface for linuxcnc-ethercat and
    IgH EtherCAT Master"""

    data_type_class = LCECDataType
    esi_reader_class = LCECXMLReader
    sdo_class = LCECSDO
    command_class = LCECCommand
