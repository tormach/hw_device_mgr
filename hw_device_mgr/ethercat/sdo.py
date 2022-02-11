from .data_types import EtherCATDataType
from ..cia_301.sdo import CiA301SDO


class EtherCATSDO(CiA301SDO):
    """
    EtherCAT SDOs with ETG.2000 data types.

    The EtherCAT object dictionary is similar to CiA 301, except for
    data types defined by ETG.2000.
    """

    data_type_class = EtherCATDataType
