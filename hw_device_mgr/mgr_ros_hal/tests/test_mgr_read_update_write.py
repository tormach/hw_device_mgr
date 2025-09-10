from .base_test_class import BaseROSHALMgrTestClass
from ...mgr_ros.tests.test_mgr_read_update_write import (
    TestROSHWDeviceMgrRUW as _TestROSHWDeviceMgrRUW,
)
from ...mgr_hal.tests.test_mgr_read_update_write import (
    TestHALHWDeviceMgrRUW as _TestHALHWDeviceMgrRUW,
)


class TestROSHWDeviceMgrRUW(
    BaseROSHALMgrTestClass, _TestHALHWDeviceMgrRUW, _TestROSHWDeviceMgrRUW
):
    pass
