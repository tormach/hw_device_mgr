import pytest
from ...ethercat.tests.test_device_read_update_write import (
    TestEtherCATDeviceRUW as _TestEtherCATDeviceRUW,
)
from ...hal.tests.test_device_read_update_write import (
    TestHALDeviceRUW as _TestHALDeviceRUW,
)
from .base_test_class import BaseLCECTestClass


class TestLCECDeviceRUW(
    BaseLCECTestClass, _TestEtherCATDeviceRUW, _TestHALDeviceRUW
):
    @pytest.fixture
    def obj(self, sim_device_data, mock_halcomp, device_cls):
        self.obj = self.device_model_cls(address=sim_device_data["address"])
        self.obj.config.init_params = False
        self.obj.init(comp=mock_halcomp)
        yield self.obj
