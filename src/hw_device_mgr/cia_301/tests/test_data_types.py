from ..data_types import CiA301DataType
from ...tests.test_data_types import TestDataType


class TestCiA301DataType(TestDataType):
    tc_base = CiA301DataType
    defined_shared_types = {
        "int8",
        "int16",
        "int32",
        "uint8",
        "uint16",
        "uint32",
        "float",
        "str",
    }
