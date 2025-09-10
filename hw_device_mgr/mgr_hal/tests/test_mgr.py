from .base_test_class import BaseHALMgrTestClass
from ...mgr.tests.test_mgr import TestHWDeviceMgr as _TestHWDeviceMgr
from ...hal.tests.test_device import TestHALDevice as _TestHALDevice


class TestHALHWDeviceMgr(BaseHALMgrTestClass, _TestHWDeviceMgr, _TestHALDevice):
    halcomp_name = "hal_mgr"
    expected_mro = [
        "HALHWDeviceMgrTestCategory",
        "HALSimHWDeviceMgr",
        "HALHWDeviceMgr",
        "HALCompDevice",  # HAL comp (this should be tested, too!)
        _TestHWDeviceMgr.expected_mro[1],  # SimHWDeviceMgr
        *_TestHALDevice.expected_mro[:2],  # HALPinSimDevice...HALPinDevice
        *_TestHWDeviceMgr.expected_mro[2:],  # HWDeviceMgr...ABC
        _TestHALDevice.expected_mro[-1],  # HalMixin (skip CiA301, etc.)
    ]
