from ...cia_402.tests.test_device_read_update_write import (
    TestCiA402DeviceRUW as _TestCiA402DeviceRUW,
)
from .base_test_class import BaseEtherCATTestClass


class TestEtherCATDeviceRUW(BaseEtherCATTestClass, _TestCiA402DeviceRUW):
    pass
