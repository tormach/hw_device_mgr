from ..data_types import DataType
from .base import HALMixin


class HALDataType(DataType, HALMixin):
    """Data type mixin for Machinekit HAL types"""

    subtype_prefix = "HAL"
    subtype_data = dict(
        bit=dict(hal_type=HALMixin.HAL_BIT),
        int8=dict(hal_type=HALMixin.HAL_S32),
        int16=dict(hal_type=HALMixin.HAL_S32),
        int32=dict(hal_type=HALMixin.HAL_S32),
        int64=dict(hal_type=HALMixin.HAL_S64),
        uint8=dict(hal_type=HALMixin.HAL_U32),
        uint16=dict(hal_type=HALMixin.HAL_U32),
        uint32=dict(hal_type=HALMixin.HAL_U32),
        uint64=dict(hal_type=HALMixin.HAL_U64),
        float=dict(hal_type=HALMixin.HAL_FLOAT),
        double=dict(hal_type=HALMixin.HAL_FLOAT),
        # No HAL_STR type
    )

    @classmethod
    def hal_type_str(cls):
        return cls.hal_enum_str(cls.hal_type)
