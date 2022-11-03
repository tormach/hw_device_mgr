from ..data_types import DataType
from .base import HALMixin


class HALDataType(DataType, HALMixin):
    """Data type mixin for Machinekit HAL types."""

    subtype_prefix = "HAL"
    subtype_data = dict(
        bit=dict(hal_type=HALMixin.HAL_BIT),
        int8=dict(hal_type=HALMixin.HAL_S32),
        int16=dict(hal_type=HALMixin.HAL_S32),
        int32=dict(hal_type=HALMixin.HAL_S32),
        uint8=dict(hal_type=HALMixin.HAL_U32),
        uint16=dict(hal_type=HALMixin.HAL_U32),
        uint32=dict(hal_type=HALMixin.HAL_U32),
        float=dict(hal_type=HALMixin.HAL_FLOAT),
        double=dict(hal_type=HALMixin.HAL_FLOAT),
        # No HAL_STR type
    )
    have_64 = hasattr(HALMixin, "HAL_S64")
    if have_64:
        # Machinekit HAL has 64-bit int types, but not LCNC
        subtype_data.update(
            dict(
                int64=dict(hal_type=HALMixin.HAL_S64),
                uint64=dict(hal_type=HALMixin.HAL_U64),
            )
        )

    @classmethod
    def hal_type_str(cls):
        return cls.hal_enum_str(cls.hal_type)
