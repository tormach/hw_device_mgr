from .sdo import LCECSDO
from ..ethercat.xml_reader import EtherCATXMLReader


class LCECXMLReader(EtherCATXMLReader):
    """Parse EtherCAT Slave Information "ESI" XML files

    Use `LCECSDO` for SDO class
    """

    sdo_class = LCECSDO
