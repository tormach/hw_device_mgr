from .base_test_class import BaseHALTestClass
from ...tests.test_data_types import TestDataType as _TestDataType


class TestHALDataType(BaseHALTestClass, _TestDataType):
    sname_to_typestr = dict(
        bit="HAL_BIT",
        int8="HAL_S32",
        int16="HAL_S32",
        int32="HAL_S32",
        int64="HAL_S64",
        uint8="HAL_U32",
        uint16="HAL_U32",
        uint32="HAL_U32",
        uint64="HAL_U64",
        float="HAL_FLOAT",
        double="HAL_FLOAT",
    )
    defined_shared_types = set(sname_to_typestr.keys())

    def test_hal_type_str(self):
        for shared_name, exp_str in self.sname_to_typestr.items():
            if shared_name not in self.data_type_class.subtype_data:
                # LinuxCNC HAL doesn't have 64-bit int types
                assert shared_name.endswith("64")
                print(f"Skipping 64-bit int type {shared_name}")
                continue
            cls = self.data_type_class.by_shared_name(shared_name)
            exp_int = self.data_type_class.hal_enum(exp_str[4:])
            cls_str, cls_int = (cls.hal_type_str(), cls.hal_type)
            print(f"{shared_name}:  {exp_str}/{exp_int} ?= {cls_str}/{cls_int}")
            assert cls_str == exp_str
            assert cls_int == exp_int
