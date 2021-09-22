from ..data_types import CiA301DataType
from ...tests.test_data_types import TestDataType as _TestDataType


class TestCiA301DataType(_TestDataType):
    tc_base = CiA301DataType
    defined_shared_types = {
        "bit",
        "int8",
        "int16",
        "int32",
        "int64",
        "uint8",
        "uint16",
        "uint32",
        "uint64",
        "float",
        "double",
        "str",
    }
