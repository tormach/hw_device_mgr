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
        "BogusLCECDevice",
        "LCECDevice",
        "BogusEtherCATDevice",
        "EtherCATDevice",
        "BogusCiA301Device",
        "CiA301Device",
        "HALPinDevice",
        "Device",
        "ABC",
        "HALMixin",
        "object",
    ]

    @pytest.fixture
    def obj(self, device_cls, device_data, sdo_data, mock_halcomp):
        self.obj = self.device_model_cls(
            address=device_data["address"], sim=self.sim
        )
        self.obj.init(mock_halcomp)
        yield self.obj
