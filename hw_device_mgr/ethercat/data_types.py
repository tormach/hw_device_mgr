from ..data_types import DataType


class EtherCATDataType(DataType):
    """Data types defined by ETG.2000."""

    subtype_prefix = "EtherCAT"
    subtype_data = dict(
        bit=dict(
            name="BOOL",
            name_re=r"BOOL|BIT",
        ),
        int8=dict(name="SINT"),
        int16=dict(name="INT"),
        int32=dict(name="DINT"),
        int64=dict(name="LINT"),
        uint8=dict(name="USINT", name_re=r"USINT|BITARR8"),
        uint16=dict(name="UINT"),
        uint32=dict(name="UDINT"),
        uint64=dict(name="ULINT"),
        float=dict(name="REAL"),  # Never seen in the wild
        double=dict(name="LREAL"),  # Never seen in the wild
        str=dict(
            # Sequence of octets
            name="STRING",
            name_re=r"STRING(\([0-9]+\))?",
        ),
    )


# ETG.2000 defines other types
