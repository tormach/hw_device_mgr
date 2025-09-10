from .base_test_class import BaseDevicesTestClass
from ...ethercat.tests.test_device import (
    TestEtherCATDevice as _TestEtherCATDevice,
)


class TestDevices(BaseDevicesTestClass, _TestEtherCATDevice):
    expected_mro = [
        c
        for c in _TestEtherCATDevice.expected_mro
        if c != "RelocatableESIDevice"
    ]

    def test_sv660_error_code_str(self, obj):
        if obj.product_code != 0x000C010D:
            return
        print(obj.feedback_out.data_types)
        assert obj.feedback_out_data_types["error_code"] == "uint16"
        assert (
            obj.feedback_out.get_data_type("error_code").shared_name == "uint16"
        )
