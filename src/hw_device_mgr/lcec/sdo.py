from ..ethercat.sdo import EtherCATSDO
from .data_types import LCECDataType


class LCECSDO(EtherCATSDO):
    """SDO class for use with the IgH EtherCAT Master"""

    data_type_class = LCECDataType
