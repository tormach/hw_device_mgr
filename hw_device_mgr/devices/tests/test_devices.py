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
