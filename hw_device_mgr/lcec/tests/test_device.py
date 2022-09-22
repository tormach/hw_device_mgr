import pytest
from ...ethercat.tests.test_device import (
    TestEtherCATDevice as _TestEtherCATDevice,
)
from ...hal.tests.test_device import (
    TestHALDevice as _TestHALDevice,
)
from .base_test_class import BaseLCECTestClass


class TestLCECDevice(BaseLCECTestClass, _TestEtherCATDevice, _TestHALDevice):

    expected_mro = [
        "LCECSimDevice",
        "LCECDevice",
        *_TestHALDevice.expected_mro[:2],  # HALPinSimDevice...HALPinDevice
        *_TestEtherCATDevice.expected_mro,  # RelocatableESIDevice...ABC
        _TestHALDevice.expected_mro[-1],  # HALMixin
    ]

    @pytest.fixture
    def obj(self, sim_device_data, mock_halcomp):
        self.obj = self.device_model_cls(
            address=sim_device_data["test_address"]
        )
        self.obj.init(comp=mock_halcomp)
        yield self.obj
