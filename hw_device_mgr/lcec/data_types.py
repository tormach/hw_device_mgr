from ..ethercat.data_types import EtherCATDataType
from ..hal.data_types import HALDataType


class LCECDataType(EtherCATDataType, HALDataType):
    """Data types for HAL and IgH Master."""

    subtype_prefix = "LCEC"
    subtype_data = dict(
        bit=dict(igh_type="bool"),
        int8=dict(igh_type="int8"),
        int16=dict(igh_type="int16"),
        int32=dict(igh_type="int32"),
        uint8=dict(igh_type="uint8"),
        uint16=dict(igh_type="uint16"),
        uint32=dict(igh_type="uint32"),
        float=dict(igh_type="float"),
        double=dict(igh_type="double"),
        # Strings not usable by `ethercat` tool
    )
    if HALDataType.have_64:
        # Machinekit HAL has 64-bit int types, but not LCNC
        subtype_data.update(
            dict(
                int64=dict(igh_type="int64"),
                uint64=dict(igh_type="uint64"),
            )
        )
