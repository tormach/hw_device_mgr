from .base_test_class import BaseCiA301TestClass
from ...tests.test_device import TestDevice as _TestDevice
import pytest


class TestCiA301Device(BaseCiA301TestClass, _TestDevice):
    expected_mro = [
        "CiA301SimDevice",
        "CiA301Device",
        *_TestDevice.expected_mro,
    ]

    # Test CiA NMT init:  online & operational status
    read_update_write_yaml = "cia_301/tests/read_update_write.cases.yaml"

    @pytest.fixture
    def obj(self, device_cls, sim_device_data):
        self.obj = self.device_model_cls(
            address=sim_device_data["test_address"]
        )
        self.obj.init()
        yield self.obj
