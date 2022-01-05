from ..data_types import DataType


class CiA301DataType(DataType):
    """Data types defined by CiA 301"""

    subtype_prefix = "CiA301"
    subtype_data = dict(
        bit=dict(name="BOOLEAN"),
        int8=dict(name="INTEGER8"),
        int16=dict(name="INTEGER16"),
        int32=dict(name="INTEGER32"),
        int64=dict(name="INTEGER64"),
        uint8=dict(name="UNSIGNED08"),
        uint16=dict(name="UNSIGNED16"),
        uint32=dict(name="UNSIGNED32"),
        uint64=dict(name="UNSIGNED64"),
        float=dict(name="REAL32"),
        double=dict(name="REAL64"),
        str=dict(name="STRING"),
    )
