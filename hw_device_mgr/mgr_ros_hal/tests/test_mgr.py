from .base_test_class import BaseROSHALMgrTestClass
from ...mgr_ros.tests.test_mgr import TestROSDeviceMgr as _TestROSDeviceMgr
from ...mgr_hal.tests.test_mgr import TestHALHWDeviceMgr as _TestHALHWDeviceMgr


class TestROSDeviceMgr(
    BaseROSHALMgrTestClass, _TestROSDeviceMgr, _TestHALHWDeviceMgr
):

    expected_mro = [
        "ROSHALHWDeviceMgr",
        "ROSHWDeviceMgr",
        "HALHWDeviceMgr",
        "HWDeviceMgr",
        "FysomGlobalMixin",
        "HALCompDevice",
        "HALPinDevice",
        "Device",
        "ABC",
        "HALMixin",
        "object",
    ]
