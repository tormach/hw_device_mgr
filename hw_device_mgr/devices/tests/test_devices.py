from .base_test_class import BaseDevicesTestClass
from ...lcec.tests.test_device import TestLCECDevice as _TestLCECDevice


class TestDevices(BaseDevicesTestClass, _TestLCECDevice):
    expected_mro = [
        c for c in _TestLCECDevice.expected_mro if c != "RelocatableESIDevice"
    ]
