from .base_test_class import BaseROSHALMgrTestClass
from ...mgr_ros.tests.test_mgr import TestROSHWDeviceMgr as _TestROSHWDeviceMgr
from ...mgr_hal.tests.test_mgr import TestHALHWDeviceMgr as _TestHALHWDeviceMgr


class TestROSHWDeviceMgr(
    BaseROSHALMgrTestClass, _TestHALHWDeviceMgr, _TestROSHWDeviceMgr
):

    expected_mro = [
        "ROSHWDeviceMgrTestCategory",
        "ROSHALSimHWDeviceMgr",
        "ROSHALHWDeviceMgr",
        *_TestROSHWDeviceMgr.expected_mro[1:3],  # ROS{Sim...}HWDeviceMgr
        *_TestHALHWDeviceMgr.expected_mro[1:-1],  # HALSimHWDeviceMgr...ABC
        "ConfigIO",
        _TestHALHWDeviceMgr.expected_mro[-1],  # HALMixin
    ]
