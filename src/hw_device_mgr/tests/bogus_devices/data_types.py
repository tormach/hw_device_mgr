from ...data_types import DataType


class BogusDataType(DataType):
    subtype_prefix = "Bogus"
    subtype_data = dict(
        bit=dict(name="Bit"),
        int8=dict(name="SInt"),
        int16=dict(name="Int"),
        int32=dict(name="DInt"),
        int64=dict(name="LInt"),
        uint8=dict(name="USInt"),
        uint16=dict(name="UInt"),
        uint32=dict(name="UDInt"),
        uint64=dict(name="ULInt"),
        float=dict(name="Real"),
        double=dict(name="LReal"),
        str=dict(
            # Sequence of octets
            name="String",
            name_re=r"String(\([0-9]+\))?",
        ),
    )
