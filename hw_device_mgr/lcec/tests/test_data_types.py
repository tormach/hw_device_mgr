from ...ethercat.tests.test_data_types import (
    TestEtherCATDataType as _TestEtherCATDataType,
)
from ...hal.tests.test_data_types import TestHALDataType as _TestHALDataType
from .base_test_class import BaseLCECTestClass


class TestLCECDataType(
    BaseLCECTestClass, _TestEtherCATDataType, _TestHALDataType
):
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
    }

    def test_igh_type_attr(self):
        for shared_name in self.defined_shared_types:
            if shared_name not in self.data_type_class.subtype_data:
                # LinuxCNC HAL doesn't have 64-bit int types
                assert shared_name.endswith("64")
                print(f"Skipping 64-bit int type {shared_name}")
                continue
            cls = self.data_type_class.by_shared_name(shared_name)
            print("cls:", cls)
            assert hasattr(cls, "igh_type")
