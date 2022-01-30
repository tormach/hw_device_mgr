from .base_test_class import BaseROSHALMgrTestClass
from ...mgr_ros.tests.test_mgr import TestROSHWDeviceMgr as _TestROSHWDeviceMgr
from ...mgr_hal.tests.test_mgr import TestHALHWDeviceMgr as _TestHALHWDeviceMgr


class TestROSHWDeviceMgr(
    BaseROSHALMgrTestClass, _TestROSHWDeviceMgr, _TestHALHWDeviceMgr
):

    expected_mro = [
        "BogusHALROSHWDeviceMgr",
        "SimROSEtherCATHWDeviceMgr",
        "ROSHALHWDeviceMgr",
        "SimROSHWDeviceMgr",
        "ROSHWDeviceMgr",
        "HALHWDeviceMgr",
        "SimHWDeviceMgr",
        "HWDeviceMgr",
        "FysomGlobalMixin",
        "HALCompDevice",
        "HALPinDevice",
        "SimDevice",
        "Device",
        "ABC",
        "HALMixin",
        "object",
    ]
