from ...tests.test_data_types import TestDataType as _TestDataType
from .base_test_class import BaseCiA301TestClass


class TestCiA301DataType(BaseCiA301TestClass, _TestDataType):
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
