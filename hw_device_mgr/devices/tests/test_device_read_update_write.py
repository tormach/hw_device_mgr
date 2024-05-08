from .base_test_class import BaseDevicesTestClass
from ...ethercat.tests.test_device_read_update_write import (
    TestEtherCATDeviceRUW as _TestEtherCATDeviceRUW,
)


class TestDevicesRUW(BaseDevicesTestClass, _TestEtherCATDeviceRUW):
    pass
