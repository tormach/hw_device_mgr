from .base_test_class import BaseDevicesTestClass
from ...ethercat.tests.test_device_read_update_write import (
    TestEtherCATDeviceRUW as _TestEtherCATDeviceRUW,
)
import re
from functools import lru_cache


class TestDevicesRUW(BaseDevicesTestClass, _TestEtherCATDeviceRUW):

    error_code_re = re.compile(r"0x([0-9A-F]{4})[0-9A-F]{4}")
    expected_error_code_attrs = ("fault_desc", "description", "goal_reason")

    @classmethod
    @lru_cache
    def fix_expected_str(cls, expected):
        return cls.error_code_re.sub(r"0x\1", expected)

    def check_interface_values(self, interface, indent=4, expected=None):
        if self.obj.product_code == 0x000C010D and interface == "feedback_out":
            # Hack test data expected error_code value for SV660N
            expected = self.test_data[interface].copy()
            assert "error_code" in expected
            expected["error_code"] = expected["error_code"] >> 16
            for attr in self.expected_error_code_attrs:
                if attr in expected:
                    expected[attr] = self.fix_expected_str(expected[attr])
        return super().check_interface_values(
            interface, indent=indent, expected=expected
        )
